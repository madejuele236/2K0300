#include "runtime/loops/control_loop.hpp"

// 主控制循环实现。
// 负责运动控制的核心循环：构建控制门输入 → 运动监督 → PID 转向 →
// 速度计算 → 执行器输出。管理各阶段状态机（arming/disarmed/active）和紧急停止。

#include <algorithm>
#include <cmath>
#include <string>

#include "control/ml_speed_policy.hpp"
#include "estimation/vehicle_pose_delta_estimator.hpp"
#include "reference/reference_control_readiness.hpp"
#include "reference/reference_lateral_error.hpp"
#include "reference/reference_tracking_geometry.hpp"
#include "reference/reference_usability.hpp"
#include "port/perf_counter.hpp"
#include "reference/reference_time_alignment.hpp"

namespace ls2k::runtime {

using safety::ControlApplyOutcome;
using safety::ControlCycleObservation;
using safety::ControlGateDecision;
using safety::ControlGateInputs;
using safety::ControlVetoReason;
using safety::EvaluateControlGate;
using safety::ObserveControlCycle;
using control::MotionDecision;
using control::MotionIntent;
using control::MotionPhase;
using control::MotionSupervisorInputs;
using control::MotionSupervisorState;
using control::ResolveRuntimeSpeedTarget;
using control::SelectPerceptionSpeedTarget;
using control::RuntimeTuningOverrideActiveAt;
using control::RuntimeTuningSnapshot;
using control::SnapshotRuntimeTuningState;
using observability::ControlDebugSnapshot;

namespace {

// 构建控制门输入 —— 汇总感知、IMU、编码器、低电压、时间戳等状态
ControlGateInputs BuildControlGateInputs(const port::PerceptionResult& perception,
                                         const port::ImuSample& imu,
                                         const port::EncoderDelta& encoder,
                                         MotionPhase motion_phase,
                                         const MotionIntent& motion_intent,
                                         bool low_voltage_emergency,
                                         uint64_t now_ms,
                                         const port::RuntimeParameters& params) {
    ControlGateInputs inputs{};
    inputs.perception_published = perception.published;
    inputs.perception_fresh = perception.fresh;
    inputs.perception_capture_time_ms = perception.capture_time_ms;
    inputs.perception_publish_time_ms = perception.publish_time_ms;
    inputs.perception_projector_ok = perception.perception_health.projector_ok;
    inputs.reference_control_ready = perception.reference_control.ready;
    inputs.reference_control_degraded = perception.reference_control.degraded;
    inputs.initial_start_boundary =
        (motion_phase == MotionPhase::kDisarmed || motion_phase == MotionPhase::kStartRequested) &&
        motion_intent.start_requested && !motion_intent.stop_requested;
    inputs.low_voltage_emergency = low_voltage_emergency;
    inputs.imu_valid = imu.valid && std::isfinite(imu.gyro_z) &&
                       std::abs(imu.gyro_z) <= port::kMaximumProducedGyroMagnitudeRadPerSec;
    inputs.encoder_valid = encoder.valid;
    inputs.now_ms = now_ms;
    inputs.perception_stale_ms = params.perception_stale_ms;
    return inputs;
}

// 构建运动监督器输入 —— 组合门控决策、运动意图、编码器状态等
MotionSupervisorInputs BuildMotionSupervisorInputs(bool startup_complete,
                                                   const MotionSupervisorState& motion_state,
                                                   const MotionIntent& motion_intent,
                                                   const RuntimeTuningSnapshot& tuning_snapshot,
                                                   const port::PerceptionResult& perception,
                                                   const ControlGateDecision& gate,
                                                   const port::EncoderDelta& encoder,
                                                   uint64_t now_ms,
                                                   const port::RuntimeParameters& params) {
    MotionSupervisorInputs inputs{};
    inputs.state = motion_state;
    inputs.intent = motion_intent;
    inputs.startup_complete = startup_complete;
    inputs.gate_clear = !gate.veto_active;
    inputs.now_ms = now_ms;
    const double default_speed_target = SelectPerceptionSpeedTarget(perception, params);
    inputs.running_speed_target =
        ResolveRuntimeSpeedTarget(tuning_snapshot, default_speed_target, now_ms);
    inputs.encoder_mean_abs = (std::abs(encoder.left) + std::abs(encoder.right)) / 2;
    inputs.motion_unveto_confirm_cycles = params.motion_unveto_confirm_cycles;
    inputs.motion_spinup_ms = params.motion_spinup_ms;
    inputs.motion_turn_limit_spinup = params.motion_turn_limit_spinup;
    inputs.motion_stop_ms = params.motion_stop_ms;
    inputs.motion_stop_encoder_threshold = params.motion_stop_encoder_threshold;
    inputs.motion_fault_rearm_hold_ms = params.motion_fault_rearm_hold_ms;
    inputs.shaped_command_zero = motion_state.last_shaped_command_zero;
    return inputs;
}

// 输出门控 veto 诊断 —— 仅在有 veto 时限频发射
void EmitVetoDiagnostics(port::DiagnosticSink& diagnostics,
                         const ControlGateDecision& decision,
                         uint64_t now_ms) {
    if (!decision.veto_active) {
        return;
    }

    const std::string reason = ToString(decision.veto_reason);
    port::EmitRateLimited(diagnostics,
                          {port::DiagnosticLevel::kFailSafe,
                           "control.veto",
                           "vetoing actuator update due to " + reason,
                           now_ms},
                          1000);
    port::EmitRateLimited(diagnostics,
                          {port::DiagnosticLevel::kFailSafe,
                           ToDiagnosticCode(decision.veto_reason),
                           "dominant control veto reason: " + reason,
                           now_ms},
                          1000);
}

// 跟踪门控状态区间并报告持续 veto/非 veto 时长
void EmitGateIntervalDiagnostics(port::DiagnosticSink& diagnostics,
                                 bool& have_gate_interval,
                                 bool& last_gate_veto,
                                 ControlVetoReason& last_gate_reason,
                                 uint64_t& gate_interval_start_ms,
                                 bool& gate_interval_reported,
                                 const ControlGateDecision& current,
                                 uint64_t now_ms) {
    if (!have_gate_interval) {
        have_gate_interval = true;
        last_gate_veto = current.veto_active;
        last_gate_reason = current.veto_reason;
        gate_interval_start_ms = now_ms;
        gate_interval_reported = false;
        return;
    }

    const bool state_changed = current.veto_active != last_gate_veto ||
                               (current.veto_active && current.veto_reason != last_gate_reason);
    if (state_changed) {
        const uint64_t duration_ms = now_ms >= gate_interval_start_ms ? now_ms - gate_interval_start_ms : 0;
        const std::string previous_reason = last_gate_veto ? ToString(last_gate_reason) : "none";
        diagnostics.Emit({last_gate_veto ? port::DiagnosticLevel::kInfo : port::DiagnosticLevel::kWarning,
                          last_gate_veto ? "control.veto.interval.closed" : "control.unveto.interval.closed",
                          std::string("previous ") + (last_gate_veto ? "veto" : "non-veto") +
                              " interval closed after " + std::to_string(duration_ms) +
                              " ms (reason=" + previous_reason + ")",
                          now_ms});
        last_gate_veto = current.veto_active;
        last_gate_reason = current.veto_reason;
        gate_interval_start_ms = now_ms;
        gate_interval_reported = false;
    }

    if (!gate_interval_reported) {
        const uint64_t duration_ms = now_ms >= gate_interval_start_ms ? now_ms - gate_interval_start_ms : 0;
        if (duration_ms >= 200) {
            diagnostics.Emit({current.veto_active ? port::DiagnosticLevel::kWarning
                                                  : port::DiagnosticLevel::kInfo,
                              current.veto_active ? "control.veto.sustained" : "control.unveto.sustained",
                              current.veto_active
                                  ? "control remained vetoed for " + std::to_string(duration_ms) +
                                        " ms with dominant reason=" + ToString(current.veto_reason)
                                  : "control remained non-veto for " + std::to_string(duration_ms) +
                                        " ms; actuator path stayed eligible for lifecycle evaluation",
                              now_ms});
            gate_interval_reported = true;
        }
    }
}

// 将有限转向输出限制在生产共享上界内。任何无效输入必须在 round/cast 前失败。
bool ClampTurnOutput(float turn_output,
                     double turn_limit_scale,
                     int configured_turn_limit,
                     int& clamped_output) noexcept {
    clamped_output = 0;
    if (!std::isfinite(turn_output) || !std::isfinite(turn_limit_scale) ||
        turn_limit_scale < 0.0 || turn_limit_scale > 1.0 || configured_turn_limit < 0) {
        return false;
    }
    const double bounded_configured_limit = static_cast<double>(std::min(
        configured_turn_limit, control::kAppliedTurnOutputMagnitudeLimit));
    const double turn_limit = turn_limit_scale * bounded_configured_limit;
    if (!std::isfinite(turn_limit) || turn_limit < 0.0 ||
        turn_limit > control::kAppliedTurnOutputMagnitudeLimit) {
        return false;
    }
    const double bounded_output = std::clamp(static_cast<double>(turn_output),
                                             -turn_limit,
                                             turn_limit);
    if (!std::isfinite(bounded_output)) {
        return false;
    }
    const double rounded_output = std::round(bounded_output);
    if (!std::isfinite(rounded_output) ||
        std::abs(rounded_output) > control::kAppliedTurnOutputMagnitudeLimit) {
        return false;
    }
    clamped_output = static_cast<int>(rounded_output);
    return true;
}

control::WheelTargetMixerParameters BuildWheelTargetMixerParameters(
    const port::RuntimeParameters& params) {
    return {params.wheel_turn_accel_delta_scale, params.wheel_turn_decel_delta_scale};
}

// 构建快照轮速目标（停止/禁止行驶时返回零值）
control::WheelSpeedTargets BuildSnapshotWheelTargets(
    const MotionDecision& decision,
    int applied_turn_output,
    const control::WheelTargetMixer& mixer,
    const control::WheelTargetMixerParameters& mixer_params) {
    if (decision.require_emergency_stop || decision.hold_disarmed || !decision.allow_drive) {
        return {};
    }

    const int snapshot_turn_output = decision.state.phase == MotionPhase::kStopping ? 0 : applied_turn_output;
    return mixer.Compute(decision.effective_speed_target, snapshot_turn_output, mixer_params);
}

/// 控制调试快照构建输入结构 —— 聚合所有用于构建快照的数据源
struct ControlDebugSnapshotInputs {
    const port::PerceptionResult& perception;                    ///< 感知结果
    const port::EncoderDelta& encoder;                           ///< 编码器差值
    const port::ActuatorCommand& command;                        ///< 执行器命令
    const control::ActuatorCommandShapingResult& command_shaping;
    const control::WheelPidComputation& left_pid_computation;
    const control::WheelPidComputation& right_pid_computation;
    const control::WheelPidCommitResult& left_pid_commit;
    const control::WheelPidCommitResult& right_pid_commit;
    const ControlCycleObservation& observation;                ///< 本周期请求结果与最后确认硬件状态
    const ControlGateDecision& gate;                             ///< 门控决策
    const MotionDecision& final_motion;                          ///< 最终运动决策
    const RuntimeTuningSnapshot& tuning_snapshot;                ///< 运行时调参快照
    const control::WheelSpeedTargets& snapshot_wheel_targets;     ///< 快照轮速目标
    const control::TurnOutputTargetComputation& turn_output_target_result;  ///< 转向输出目标计算结果
    const control::GyroTurnComputation& gyro_turn;                ///< 陀螺仪转向计算结果
    uint64_t now_ms = 0;                           ///< 当前时间戳（ms）
    uint64_t cycle_count = 0;                      ///< 控制周期计数
    int raw_turn_output = 0;                       ///< 原始转向输出
    int applied_turn_output = 0;                   ///< 应用后的转向输出
    control::SteeringYawStatus yaw_fault_status = control::SteeringYawStatus::kNotComputed;
    bool steering_terms_valid = false;              ///< 转向项是否有效
};

port::PerceptionResult BuildControlTimePerception(const port::PerceptionResult& perception,
                                                  const port::MotionHistory& motion_history,
                                                  const port::ControlCommandHistory& command_history,
                                                  uint64_t now_ms,
                                                  const port::RuntimeParameters& params) {
    const uint64_t effective_delay_ms =
        static_cast<uint64_t>(std::max(0, params.reference_time_alignment.effective_delay_ms));
    const uint64_t control_effective_time_ms = now_ms + effective_delay_ms;
    if (!perception.published || !perception.fresh || !params.reference_time_alignment.enabled) {
        port::PerceptionResult out = perception;
        out.reference_time_alignment.enabled = params.reference_time_alignment.enabled;
        out.reference_time_alignment.valid = !params.reference_time_alignment.enabled;
        out.reference_time_alignment.reason =
            params.reference_time_alignment.enabled ? "perception_unavailable" : "disabled";
        out.reference_time_alignment.reference_capture_time_ms = perception.reference_capture_time_ms;
        out.reference_time_alignment.control_time_ms = now_ms;
        out.reference_time_alignment.control_effective_time_ms = control_effective_time_ms;
        if (params.reference_time_alignment.enabled) {
            out.reference_usability = {};
            out.reference_usability.reason = "reference_time_alignment_perception_unavailable";
            out.reference_lateral_error = {};
            out.reference_lateral_error.reason = out.reference_usability.reason;
            out.reference_tracking_geometry = {};
            out.reference_tracking_geometry.reason = out.reference_usability.reason;
            out.reference_control = {};
            out.reference_control.reason = out.reference_usability.reason;
        }
        return out;
    }

    port::PerceptionResult out = perception;
    const port::VehiclePoseDelta pose_delta =
        estimation::EstimateVehiclePoseDelta(perception.reference_capture_time_ms,
                                             now_ms,
                                             control_effective_time_ms,
                                             motion_history,
                                             command_history,
                                             params.reference_time_alignment,
                                             params.motion_odometry);
    const reference::ReferenceTimeAlignmentResult alignment =
        reference::AlignReferencePathToVehiclePoseDelta(perception.reference_path,
                                                       perception.reference_capture_time_ms,
                                                       now_ms,
                                                       control_effective_time_ms,
                                                       pose_delta,
                                                       params);
    out.reference_time_alignment = alignment.facts;
    out.reference_path = alignment.reference_path;
    if (!alignment.facts.valid) {
        out.reference_usability = {};
        out.reference_usability.reason = "reference_time_alignment_" + alignment.facts.reason;
        out.reference_lateral_error = {};
        out.reference_lateral_error.reason = out.reference_usability.reason;
        out.reference_tracking_geometry = {};
        out.reference_tracking_geometry.reason = out.reference_usability.reason;
        out.reference_control = {};
        out.reference_control.reason = out.reference_usability.reason;
        return out;
    }

    out.reference_usability = reference::EvaluateReferenceUsability(alignment.reference_path, params);
    out.reference_lateral_error =
        reference::ComputeReferenceLateralError(alignment.reference_path,
                                                out.reference_usability,
                                                params);
    out.reference_tracking_geometry =
        reference::ComputeReferenceTrackingGeometry(alignment.reference_path,
                                                    out.reference_usability,
                                                    params.bev_control_model);
    out.reference_control =
        reference::EvaluateReferenceControlReadiness(out.reference_usability,
                                                     out.reference_tracking_geometry,
                                                     perception.reference_control.degraded);
    return out;
}

/// 构建控制调试快照：从各数据源组装完整的 ControlDebugSnapshot
/// @param inputs  构建快照所需的全部输入
/// @return        填充好的 ControlDebugSnapshot
// NOLINTNEXTLINE(readability-function-size)
ControlDebugSnapshot BuildControlDebugSnapshot(const ControlDebugSnapshotInputs& inputs) {
    const port::PerceptionResult& perception = inputs.perception;
    ControlDebugSnapshot debug_snapshot{};
    const bool override_active = RuntimeTuningOverrideActiveAt(inputs.tuning_snapshot, inputs.now_ms);
    debug_snapshot.valid = true;
    debug_snapshot.cycle_count = inputs.cycle_count;
    debug_snapshot.timestamp_ms = inputs.now_ms;
    debug_snapshot.motion_phase = inputs.final_motion.state.phase;
    debug_snapshot.veto_active = inputs.gate.veto_active;
    debug_snapshot.veto_reason = inputs.gate.veto_reason;
    debug_snapshot.tuning_mode_enabled = inputs.tuning_snapshot.tuning_mode_enabled;
    debug_snapshot.turn_suppressed =
        inputs.tuning_snapshot.tuning_mode_enabled && inputs.tuning_snapshot.turn_suppressed;
    debug_snapshot.target_speed_override_enabled = override_active;
    debug_snapshot.target_speed_override_value =
        override_active ? inputs.tuning_snapshot.target_speed_override_value : 0.0;
    debug_snapshot.effective_speed_target = inputs.final_motion.effective_speed_target;
    debug_snapshot.left_speed_target = inputs.snapshot_wheel_targets.left;
    debug_snapshot.right_speed_target = inputs.snapshot_wheel_targets.right;
    debug_snapshot.left_measured_speed = static_cast<double>(inputs.encoder.left);
    debug_snapshot.right_measured_speed = static_cast<double>(inputs.encoder.right);
    debug_snapshot.raw_turn_output = inputs.raw_turn_output;
    debug_snapshot.applied_turn_output = inputs.applied_turn_output;
    debug_snapshot.left_drive_pwm_command = inputs.command.left_drive_pwm;
    debug_snapshot.right_drive_pwm_command = inputs.command.right_drive_pwm;
    debug_snapshot.left_brushless_pwm_command = inputs.command.left_brushless_pwm;
    debug_snapshot.right_brushless_pwm_command = inputs.command.right_brushless_pwm;
    debug_snapshot.left_drive_pwm_unconstrained = inputs.left_pid_computation.unconstrained_output;
    debug_snapshot.right_drive_pwm_unconstrained = inputs.right_pid_computation.unconstrained_output;
    debug_snapshot.left_drive_pwm_requested = inputs.command_shaping.left.requested_pwm;
    debug_snapshot.right_drive_pwm_requested = inputs.command_shaping.right.requested_pwm;
    debug_snapshot.left_drive_pwm_desired = inputs.command_shaping.left.desired_pwm;
    debug_snapshot.right_drive_pwm_desired = inputs.command_shaping.right.desired_pwm;
    debug_snapshot.left_drive_pwm_step_limited = inputs.command_shaping.left.step_limited;
    debug_snapshot.right_drive_pwm_step_limited = inputs.command_shaping.right.step_limited;
    debug_snapshot.left_drive_pwm_reverse_suppressed =
        inputs.command_shaping.left.reverse_suppressed;
    debug_snapshot.right_drive_pwm_reverse_suppressed =
        inputs.command_shaping.right.reverse_suppressed;
    debug_snapshot.left_drive_pwm_floor_adjusted = inputs.command_shaping.left.floor_adjusted;
    debug_snapshot.right_drive_pwm_floor_adjusted = inputs.command_shaping.right.floor_adjusted;
    debug_snapshot.left_pid_error = inputs.left_pid_computation.error;
    debug_snapshot.right_pid_error = inputs.right_pid_computation.error;
    debug_snapshot.left_pid_integral = inputs.left_pid_commit.integral;
    debug_snapshot.right_pid_integral = inputs.right_pid_commit.integral;
    debug_snapshot.left_pid_integral_candidate = inputs.left_pid_computation.candidate_integral;
    debug_snapshot.right_pid_integral_candidate = inputs.right_pid_computation.candidate_integral;
    debug_snapshot.left_pid_anti_windup_active = inputs.left_pid_commit.anti_windup_active;
    debug_snapshot.right_pid_anti_windup_active = inputs.right_pid_commit.anti_windup_active;
    debug_snapshot.left_pid_anti_windup_reason = inputs.left_pid_commit.reason;
    debug_snapshot.right_pid_anti_windup_reason = inputs.right_pid_commit.reason;
    debug_snapshot.apply_outcome = inputs.observation.apply_outcome;
    debug_snapshot.actuators_armed = inputs.observation.actuators_armed;
    debug_snapshot.last_confirmed_left_drive_pwm = inputs.observation.applied_left_drive_pwm;
    debug_snapshot.last_confirmed_right_drive_pwm = inputs.observation.applied_right_drive_pwm;
    debug_snapshot.last_confirmed_left_brushless_pwm = inputs.observation.applied_left_brushless_pwm;
    debug_snapshot.last_confirmed_right_brushless_pwm = inputs.observation.applied_right_brushless_pwm;
    debug_snapshot.emergency_stop = inputs.command.emergency_stop;
    debug_snapshot.steering.valid = inputs.steering_terms_valid;
    debug_snapshot.steering.frame_id = perception.frame_id;
    debug_snapshot.steering.capture_time_ms = perception.capture_time_ms;
    debug_snapshot.steering.otsu = perception.otsu;
    debug_snapshot.steering.perception_tag = perception.perception_tag;
    debug_snapshot.steering.boundary_row_count = perception.boundary_row_count;
    debug_snapshot.steering.boundary_jump_count = perception.boundary_jump_count;
    debug_snapshot.steering.boundary_span_count = perception.boundary_span_count;
    debug_snapshot.steering.ml = perception.ml;
    debug_snapshot.steering.speed_selection_source =
        override_active ? "runtime_override"
                        : (perception.ml.active ? "ml_maneuver" : "running_default");
    debug_snapshot.steering.effective_speed_target = inputs.final_motion.effective_speed_target;
    debug_snapshot.steering.perception_health.projector_ok = perception.perception_health.projector_ok;
    debug_snapshot.steering.perception_health.reason = perception.perception_health.reason;
    debug_snapshot.steering.element_evidence = perception.element_evidence;
    debug_snapshot.steering.circle_v2 = perception.circle_v2;
    debug_snapshot.steering.visual_reference.present = perception.visual_reference_selection.present;
    debug_snapshot.steering.visual_reference.source = perception.visual_reference_selection.source;
    debug_snapshot.steering.visual_reference.reason = perception.visual_reference_selection.reason;
    debug_snapshot.steering.visual_reference.candidate_count =
        perception.visual_reference_selection.candidate_count;
    debug_snapshot.steering.visual_reference.rejected_candidate_reason =
        perception.visual_reference_selection.rejected_candidate_reason;
    debug_snapshot.steering.visual_reference.candidate_paths =
        perception.visual_reference_candidate_paths;
    debug_snapshot.steering.reference.mode = perception.reference_mode;
    debug_snapshot.steering.reference.source = perception.reference_source;
    debug_snapshot.steering.reference.control_path = perception.reference_path;
    debug_snapshot.steering.eligibility.usable = perception.reference_usability.usable;
    debug_snapshot.steering.eligibility.leading_usable_samples =
        perception.reference_usability.leading_usable_samples;
    debug_snapshot.steering.eligibility.leading_min_forward_m =
        perception.reference_usability.leading_min_forward_m;
    debug_snapshot.steering.eligibility.leading_max_forward_m =
        perception.reference_usability.leading_max_forward_m;
    debug_snapshot.steering.eligibility.reason = perception.reference_usability.reason;
    debug_snapshot.steering.lateral_error.computed = perception.reference_lateral_error.computed;
    debug_snapshot.steering.lateral_error.weighted_lateral_error_m =
        perception.reference_lateral_error.weighted_lateral_error_m;
    debug_snapshot.steering.lateral_error.weighted_sample_count =
        perception.reference_lateral_error.weighted_sample_count;
    debug_snapshot.steering.lateral_error.weight_sum = perception.reference_lateral_error.weight_sum;
    debug_snapshot.steering.lateral_error.reason = perception.reference_lateral_error.reason;
    debug_snapshot.steering.reference_tracking_geometry.computed =
        perception.reference_tracking_geometry.computed;
    debug_snapshot.steering.reference_tracking_geometry.lateral_offset_m =
        perception.reference_tracking_geometry.lateral_offset_m;
    debug_snapshot.steering.reference_tracking_geometry.heading_error_rad =
        perception.reference_tracking_geometry.heading_error_rad;
    debug_snapshot.steering.reference_tracking_geometry.curvature_m_inv =
        perception.reference_tracking_geometry.curvature_m_inv;
    debug_snapshot.steering.reference_tracking_geometry.sample_count =
        perception.reference_tracking_geometry.sample_count;
    debug_snapshot.steering.reference_tracking_geometry.reason =
        perception.reference_tracking_geometry.reason;
    debug_snapshot.steering.reference_time_alignment.enabled =
        perception.reference_time_alignment.enabled;
    debug_snapshot.steering.reference_time_alignment.valid =
        perception.reference_time_alignment.valid;
    debug_snapshot.steering.reference_time_alignment.reason =
        perception.reference_time_alignment.reason;
    debug_snapshot.steering.reference_time_alignment.age_ms =
        perception.reference_time_alignment.age_ms;
    debug_snapshot.steering.reference_time_alignment.reference_capture_time_ms =
        perception.reference_time_alignment.reference_capture_time_ms;
    debug_snapshot.steering.reference_time_alignment.control_time_ms =
        perception.reference_time_alignment.control_time_ms;
    debug_snapshot.steering.reference_time_alignment.control_effective_time_ms =
        perception.reference_time_alignment.control_effective_time_ms;
    debug_snapshot.steering.reference_time_alignment.measured_until_ms =
        perception.reference_time_alignment.measured_until_ms;
    debug_snapshot.steering.reference_time_alignment.predicted_ms =
        perception.reference_time_alignment.predicted_ms;
    debug_snapshot.steering.reference_time_alignment.delta_forward_m =
        perception.reference_time_alignment.delta_forward_m;
    debug_snapshot.steering.reference_time_alignment.delta_lateral_m =
        perception.reference_time_alignment.delta_lateral_m;
    debug_snapshot.steering.reference_time_alignment.delta_yaw_rad =
        perception.reference_time_alignment.delta_yaw_rad;
    debug_snapshot.steering.reference_time_alignment.measured_forward_mps =
        perception.reference_time_alignment.measured_forward_mps;
    debug_snapshot.steering.reference_time_alignment.measured_yaw_rate_radps =
        perception.reference_time_alignment.measured_yaw_rate_radps;
    debug_snapshot.steering.reference_time_alignment.predicted_forward_mps =
        perception.reference_time_alignment.predicted_forward_mps;
    debug_snapshot.steering.reference_time_alignment.predicted_yaw_rate_radps =
        perception.reference_time_alignment.predicted_yaw_rate_radps;
    debug_snapshot.steering.reference_time_alignment.used_encoder_forward =
        perception.reference_time_alignment.used_encoder_forward;
    debug_snapshot.steering.reference_time_alignment.used_imu_yaw =
        perception.reference_time_alignment.used_imu_yaw;
    debug_snapshot.steering.reference_time_alignment.used_wheel_yaw =
        perception.reference_time_alignment.used_wheel_yaw;
    debug_snapshot.steering.reference_time_alignment.used_command_prediction =
        perception.reference_time_alignment.used_command_prediction;
    debug_snapshot.steering.reference_time_alignment.input_sample_count =
        perception.reference_time_alignment.input_sample_count;
    debug_snapshot.steering.reference_time_alignment.aligned_sample_count =
        perception.reference_time_alignment.aligned_sample_count;
    debug_snapshot.steering.reference_control.ready = perception.reference_control.ready;
    debug_snapshot.steering.reference_control.reason = perception.reference_control.reason;
    debug_snapshot.steering.safety_gate.veto_active = inputs.gate.veto_active;
    debug_snapshot.steering.safety_gate.reason = ToString(inputs.gate.veto_reason);
    debug_snapshot.steering.degraded.active = perception.reference_control.degraded;
    debug_snapshot.steering.degraded.reason =
        perception.reference_control.degraded ? perception.reference_control.reason : "none";
    debug_snapshot.steering.yaw_control.valid =
        inputs.yaw_fault_status == control::SteeringYawStatus::kNotComputed &&
        inputs.turn_output_target_result.valid && inputs.gyro_turn.valid;
    debug_snapshot.steering.yaw_control.reason =
        inputs.yaw_fault_status != control::SteeringYawStatus::kNotComputed
            ? control::ToString(inputs.yaw_fault_status)
            : !inputs.turn_output_target_result.valid
            ? control::ToString(inputs.turn_output_target_result.status)
            : control::ToString(inputs.gyro_turn.status);
    debug_snapshot.steering.yaw_control.turn_output_target =
        inputs.turn_output_target_result.turn_output_target;
    debug_snapshot.steering.yaw_control.lateral_term =
        inputs.turn_output_target_result.lateral_term;
    debug_snapshot.steering.yaw_control.heading_term =
        inputs.turn_output_target_result.heading_term;
    debug_snapshot.steering.yaw_control.curvature_term =
        inputs.turn_output_target_result.curvature_term;
    debug_snapshot.steering.actuator.raw_turn_output = inputs.raw_turn_output;
    debug_snapshot.steering.actuator.applied_turn_output = inputs.applied_turn_output;
    debug_snapshot.steering.actuator.left_drive_pwm_command = inputs.command.left_drive_pwm;
    debug_snapshot.steering.actuator.right_drive_pwm_command = inputs.command.right_drive_pwm;
    debug_snapshot.steering.actuator.left_brushless_pwm_command = inputs.command.left_brushless_pwm;
    debug_snapshot.steering.actuator.right_brushless_pwm_command = inputs.command.right_brushless_pwm;
    debug_snapshot.steering.actuator.left_drive_pwm_unconstrained =
        debug_snapshot.left_drive_pwm_unconstrained;
    debug_snapshot.steering.actuator.right_drive_pwm_unconstrained =
        debug_snapshot.right_drive_pwm_unconstrained;
    debug_snapshot.steering.actuator.left_drive_pwm_requested =
        debug_snapshot.left_drive_pwm_requested;
    debug_snapshot.steering.actuator.right_drive_pwm_requested =
        debug_snapshot.right_drive_pwm_requested;
    debug_snapshot.steering.actuator.left_drive_pwm_desired =
        debug_snapshot.left_drive_pwm_desired;
    debug_snapshot.steering.actuator.right_drive_pwm_desired =
        debug_snapshot.right_drive_pwm_desired;
    debug_snapshot.steering.actuator.left_drive_pwm_step_limited =
        debug_snapshot.left_drive_pwm_step_limited;
    debug_snapshot.steering.actuator.right_drive_pwm_step_limited =
        debug_snapshot.right_drive_pwm_step_limited;
    debug_snapshot.steering.actuator.left_drive_pwm_reverse_suppressed =
        debug_snapshot.left_drive_pwm_reverse_suppressed;
    debug_snapshot.steering.actuator.right_drive_pwm_reverse_suppressed =
        debug_snapshot.right_drive_pwm_reverse_suppressed;
    debug_snapshot.steering.actuator.left_drive_pwm_floor_adjusted =
        debug_snapshot.left_drive_pwm_floor_adjusted;
    debug_snapshot.steering.actuator.right_drive_pwm_floor_adjusted =
        debug_snapshot.right_drive_pwm_floor_adjusted;
    debug_snapshot.steering.actuator.left_pid_error = debug_snapshot.left_pid_error;
    debug_snapshot.steering.actuator.right_pid_error = debug_snapshot.right_pid_error;
    debug_snapshot.steering.actuator.left_pid_integral = debug_snapshot.left_pid_integral;
    debug_snapshot.steering.actuator.right_pid_integral = debug_snapshot.right_pid_integral;
    debug_snapshot.steering.actuator.left_pid_integral_candidate =
        debug_snapshot.left_pid_integral_candidate;
    debug_snapshot.steering.actuator.right_pid_integral_candidate =
        debug_snapshot.right_pid_integral_candidate;
    debug_snapshot.steering.actuator.left_pid_anti_windup_active =
        debug_snapshot.left_pid_anti_windup_active;
    debug_snapshot.steering.actuator.right_pid_anti_windup_active =
        debug_snapshot.right_pid_anti_windup_active;
    debug_snapshot.steering.actuator.left_pid_anti_windup_reason =
        debug_snapshot.left_pid_anti_windup_reason;
    debug_snapshot.steering.actuator.right_pid_anti_windup_reason =
        debug_snapshot.right_pid_anti_windup_reason;
    debug_snapshot.steering.actuator.apply_outcome = inputs.observation.apply_outcome;
    debug_snapshot.steering.actuator.actuators_armed = inputs.observation.actuators_armed;
    debug_snapshot.steering.actuator.last_confirmed_left_drive_pwm =
        inputs.observation.applied_left_drive_pwm;
    debug_snapshot.steering.actuator.last_confirmed_right_drive_pwm =
        inputs.observation.applied_right_drive_pwm;
    debug_snapshot.steering.actuator.last_confirmed_left_brushless_pwm =
        inputs.observation.applied_left_brushless_pwm;
    debug_snapshot.steering.actuator.last_confirmed_right_brushless_pwm =
        inputs.observation.applied_right_brushless_pwm;

    debug_snapshot.steering_internal.valid = inputs.steering_terms_valid;
    debug_snapshot.steering_internal.frame_id = perception.frame_id;
    debug_snapshot.steering_internal.capture_time_ms = perception.capture_time_ms;
    debug_snapshot.steering_internal.lateral_offset_gain =
        inputs.turn_output_target_result.lateral_offset_gain;
    debug_snapshot.steering_internal.heading_error_gain =
        inputs.turn_output_target_result.heading_error_gain;
    debug_snapshot.steering_internal.curvature_gain =
        inputs.turn_output_target_result.curvature_gain;
    debug_snapshot.steering_internal.speed_scale =
        inputs.turn_output_target_result.speed_scale;
    debug_snapshot.steering_internal.turn_output_candidate =
        inputs.turn_output_target_result.turn_output_candidate;
    debug_snapshot.steering_internal.gyro_z = inputs.gyro_turn.gyro_z;
    debug_snapshot.steering_internal.gyro_error = inputs.gyro_turn.gyro_error;
    debug_snapshot.steering_internal.gyro_p_term = inputs.gyro_turn.gyro_p_term;
    debug_snapshot.steering_internal.gyro_d_term = inputs.gyro_turn.gyro_d_term;
    return debug_snapshot;
}

// 全局重置控制器，并请求 perception owner 在下一帧重置自己的记忆。
void ResetControllerState(control::SteeringYawController& yaw_controller,
                          control::WheelPidController& left_wheel_pid,
                          control::WheelPidController& right_wheel_pid,
                          port::SteeringControlMemory& control_memory,
                          RuntimeState& state) {
    yaw_controller.Reset();
    left_wheel_pid.Reset();
    right_wheel_pid.Reset();
    ResetSteeringControlMemory(control_memory);
    state.perception_memory_reset_generation.fetch_add(1);
    state.command_history.Clear();
}

// 发射运动阶段诊断 —— 阶段转换、启动阻塞、故障复位准备状态
void EmitMotionDiagnostics(port::DiagnosticSink& diagnostics,
                           const MotionDecision& decision,
                           const ControlGateDecision& gate,
                           bool& motion_reset_ready_reported,
                           uint64_t now_ms) {
    if (decision.phase_changed) {
        diagnostics.Emit({port::DiagnosticLevel::kInfo,
                          "motion.phase.transition",
                          std::string("motion phase transitioned from ") + ToString(decision.previous_phase) +
                              " to " + ToString(decision.state.phase),
                          now_ms});
        switch (decision.state.phase) {
            case MotionPhase::kSpinup:
                diagnostics.Emit({port::DiagnosticLevel::kInfo,
                                  "motion.spinup.enter",
                                  "motion start request cleared the gate-confirmation boundary and entered spinup",
                                  now_ms});
                break;
            case MotionPhase::kRunning:
                diagnostics.Emit({port::DiagnosticLevel::kInfo,
                                  "motion.spinup.complete",
                                  "spinup timer completed and running authority is now released",
                                  now_ms});
                break;
            case MotionPhase::kDisarmed:
                if (decision.previous_phase == MotionPhase::kStopping) {
                    diagnostics.Emit({port::DiagnosticLevel::kInfo,
                                      "motion.stop.complete",
                                      "controlled stop completed and runtime returned to DISARMED",
                                      now_ms});
                } else if (decision.previous_phase == MotionPhase::kFailSafeLatched) {
                    diagnostics.Emit({port::DiagnosticLevel::kInfo,
                                      "motion.failsafe.rearmed",
                                      "fail-safe latch cleared after explicit reset intent and hold time",
                                      now_ms});
                }
                break;
            case MotionPhase::kFailSafeLatched:
                diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                                  "motion.failsafe.latched",
                                  "runtime fault interrupted motion and entered FAIL_SAFE_LATCHED (reason=" +
                                      std::string(ToString(gate.veto_reason)) + ")",
                                  now_ms});
                break;
            case MotionPhase::kStartRequested:
            case MotionPhase::kStopping:
                break;
        }
    }

    if (decision.blocked_start) {
        port::EmitRateLimited(diagnostics,
                              {port::DiagnosticLevel::kWarning,
                               "motion.start.blocked",
                               std::string("start request remains blocked while gate reason=") +
                                   ToString(gate.veto_reason),
                               now_ms},
                              1000);
    }

    if (decision.reset_ready && !motion_reset_ready_reported) {
        diagnostics.Emit({port::DiagnosticLevel::kInfo,
                          "motion.failsafe.reset_ready",
                          "fail-safe latch hold completed; explicit reset intent may now re-arm the runtime",
                          now_ms});
        motion_reset_ready_reported = true;
    } else if (!decision.reset_ready) {
        motion_reset_ready_reported = false;
    }
}

// 发射控制周期观测诊断 —— 输出应用结果、arming 转换等信息
void EmitObservationDiagnostics(port::DiagnosticSink& diagnostics,
                                const ControlCycleObservation& previous,
                                const ControlCycleObservation& current,
                                uint64_t now_ms) {
    if (current.requested_nonzero_output) {
        port::EmitRateLimited(diagnostics,
                              {port::DiagnosticLevel::kInfo,
                               "control.command.requested_nonzero",
                               "non-zero differential-drive command requested",
                               now_ms},
                              1000);
    }

    switch (current.apply_outcome) {
        case ControlApplyOutcome::kSuppressedByProfile:
            port::EmitRateLimited(diagnostics,
                                  {port::DiagnosticLevel::kWarning,
                                   "control.apply.suppressed",
                                   "drive command suppressed because the active actuator profile is diagnostics-only",
                                   now_ms},
                                  1000);
            break;
        case ControlApplyOutcome::kHeldDisarmedApplied:
            port::EmitRateLimited(diagnostics,
                                  {port::DiagnosticLevel::kInfo,
                                   "control.apply.hold_disarmed",
                                   std::string("gate is clear but runtime is intentionally held disarmed in phase ") +
                                       ToString(current.motion_phase),
                                   now_ms},
                                  1000);
            break;
        case ControlApplyOutcome::kDriveCommandApplied:
            port::EmitRateLimited(diagnostics,
                                  {port::DiagnosticLevel::kInfo,
                                   "control.apply.drive",
                                   "drive command applied to actuator adapter",
                                   now_ms},
                                  1000);
            port::EmitRateLimited(diagnostics,
                                  {port::DiagnosticLevel::kInfo,
                                   "control.apply.command",
                                   "actuator command left_drive_pwm=" +
                                       std::to_string(current.applied_left_drive_pwm) +
                                       " right_drive_pwm=" + std::to_string(current.applied_right_drive_pwm) +
                                       " left_brushless_pwm=" +
                                       std::to_string(current.applied_left_brushless_pwm) +
                                       " right_brushless_pwm=" +
                                       std::to_string(current.applied_right_brushless_pwm),
                                   now_ms},
                                  1000);
            break;
        case ControlApplyOutcome::kEmergencyStopApplied:
            port::EmitRateLimited(diagnostics,
                                  {port::DiagnosticLevel::kFailSafe,
                                   "control.apply.emergency_stop",
                                   "emergency-stop command applied to keep actuators fail-safe",
                                   now_ms},
                                  1000);
            break;
        case ControlApplyOutcome::kApplyFailed:
            port::EmitRateLimited(diagnostics,
                                  {port::DiagnosticLevel::kFailSafe,
                                   "control.apply.failed",
                                   "actuator adapter rejected the current control command",
                                   now_ms},
                                  1000);
            break;
        case ControlApplyOutcome::kZeroCommandApplied:
        case ControlApplyOutcome::kNotRequested:
            break;
    }

    if (current.arming_transition) {
        const std::string message =
            std::string("actuator armed state transitioned from ") +
            (previous.actuators_armed ? "armed" : "disarmed") + " to " +
            (current.actuators_armed ? "armed" : "disarmed") +
            " (apply_outcome=" + ToString(current.apply_outcome) +
            ", phase=" + ToString(current.motion_phase) + ")";
        diagnostics.Emit({current.actuators_armed ? port::DiagnosticLevel::kInfo
                                                  : port::DiagnosticLevel::kFailSafe,
                          "control.arm.transition",
                          message,
                          now_ms});
    }
}

}  // namespace

/// 构造控制循环：保存平台、配置、状态和诊断的引用
/// @param platform     平台适配器集合
/// @param profile      硬件配置文件
/// @param state        运行时状态
/// @param diagnostics  诊断输出接口
ControlLoop::ControlLoop(port::PlatformBundle& platform,
                         const port::HardwareProfile& profile,
                         RuntimeState& state,
                         port::DiagnosticSink& diagnostics)
    : platform_(platform), profile_(profile), state_(state), diagnostics_(diagnostics) {}

/// 启动控制循环：校验状态 → 配置 PID/混合器 → 注册定时器 → 初始化运动状态
/// @param params  运行时参数
/// @return        是否成功启动
bool ControlLoop::Start(const port::RuntimeParameters& params) {
    if (running_) {
        return true;
    }
    if (!state_.startup_complete) {
        diagnostics_.Emit({port::DiagnosticLevel::kFailSafe,
                           "control.startup.incomplete",
                           "control loop refused to start before startup completion",
                           port::NowMs()});
        return false;
    }
    if (!platform_.actuator) {
        diagnostics_.Emit({port::DiagnosticLevel::kFailSafe,
                           "control.actuator.not_ready",
                           "control loop refused to start because actuator adapter is missing",
                           port::NowMs()});
        return false;
    }

    const bool degraded_actuator_disabled = state_.degraded_startup &&
                                            profile_.actuator.mode == port::SubsystemMode::kDisabled;
    if (!platform_.actuator->Ready() && !degraded_actuator_disabled) {
        diagnostics_.Emit({port::DiagnosticLevel::kFailSafe,
                           "control.actuator.not_ready",
                           "control loop refused to arm because actuator adapter is not ready",
                           port::NowMs()});
        return false;
    }
    if (degraded_actuator_disabled) {
        diagnostics_.Emit({port::DiagnosticLevel::kWarning,
                           "control.start.degraded.actuator_disabled",
                           "degraded startup keeps control loop running with actuator disabled; actuators stay disarmed",
                           port::NowMs()});
    }

    if (!yaw_controller_.Configure(params)) {
        diagnostics_.Emit({port::DiagnosticLevel::kFailSafe,
                           "control.yaw.invalid_configuration",
                           "control loop refused to start because yaw controller parameters violate the production float arithmetic contract",
                           port::NowMs()});
        return false;
    }
    params_ = params;
    yaw_controller_.Reset();
    latched_yaw_fault_status_ = control::SteeringYawStatus::kNotComputed;
    left_wheel_pid_.Configure(params_.left_wheel_pid);
    right_wheel_pid_.Configure(params_.right_wheel_pid);
    left_wheel_pid_.Reset();
    right_wheel_pid_.Reset();
    debug_reporter_.Configure(params_);
    debug_reporter_.Reset();

    state_.timer_started = false;
    state_.actuators_armed = false;
    state_.last_command = {};
    state_.control_observation = {};
    state_.control_debug_snapshot = {};
    state_.command_history.Clear();
    ResetSteeringControlMemory(steering_control_memory_);
    state_.perception_memory_reset_generation.fetch_add(1);
    state_.motion_state.phase = MotionPhase::kDisarmed;
    state_.motion_state.phase_entry_ms = port::NowMs();
    state_.motion_state.fail_safe_latched_at_ms = 0;
    state_.motion_state.clean_gate_cycles = 0;

    motion_reset_ready_reported_ = false;
    timer_failure_.store(false);
    actuator_disable_ok_.store(true);
    running_ = true;
    const bool timer_ok = platform_.timer->Start(
        profile_.timer,
        static_cast<uint32_t>(std::max(1, params_.control_period_ms)),
        [this]() { Tick(); },
        [this]() { HandleTimerFailure(); },
        diagnostics_);
    if (!timer_ok) {
        running_ = false;
        diagnostics_.Emit({port::DiagnosticLevel::kFailSafe,
                           "control.timer.start",
                           "control timer failed to start; actuators remain disarmed",
                           port::NowMs()});
        return false;
    }

    state_.timer_started = true;
    const bool low_voltage_block = state_.low_voltage_emergency.load();
    const bool actuator_hook_block = profile_.actuator.mode == port::SubsystemMode::kAdaptationHook;
    const bool actuator_disabled_block = profile_.actuator.mode == port::SubsystemMode::kDisabled;
    if (low_voltage_block) {
        diagnostics_.Emit({port::DiagnosticLevel::kFailSafe,
                           "control.arm.low_voltage",
                           "low-voltage emergency active; actuators stay disarmed until cleared",
                           port::NowMs()});
    }
    if (actuator_hook_block) {
        diagnostics_.Emit({port::DiagnosticLevel::kFailSafe,
                           "control.arm.actuator_hook",
                           "actuator adaptation hook has no implementation; actuators stay disarmed",
                           port::NowMs()});
    }
    if (actuator_disabled_block) {
        diagnostics_.Emit({port::DiagnosticLevel::kFailSafe,
                           "control.arm.actuator_disabled",
                           "actuator profile is disabled; control loop stays diagnostics-only with actuators disarmed",
                           port::NowMs()});
    }
    diagnostics_.Emit({port::DiagnosticLevel::kInfo,
                       "control.start",
                       "control loop started with gate -> motion supervisor -> shaping/apply sequencing",
                       port::NowMs()});
    return true;
}

// 停止控制循环：停止定时器 → 禁用电机 → 复位运行时状态
ControlLoopTerminalResult ControlLoop::Stop() {
    if (!running_.exchange(false)) {
        return {timer_failure_.load(), actuator_disable_ok_.load()};
    }
    platform_.timer->Stop(diagnostics_);
    bool disable_ok = true;
    if (platform_.actuator) {
        disable_ok = platform_.actuator->Disable(diagnostics_);
        if (!disable_ok) {
            actuator_disable_ok_.store(false);
            diagnostics_.Emit({port::DiagnosticLevel::kFailSafe,
                               "control.stop.disable_failed",
                               "control loop stop could not confirm safe actuator disable",
                               port::NowMs()});
        }
    }
    if (disable_ok && actuator_disable_ok_.load()) {
        ResetDisarmedControlState();
    } else {
        ResetControllerState(yaw_controller_,
                             left_wheel_pid_,
                             right_wheel_pid_,
                             steering_control_memory_,
                             state_);
        latched_yaw_fault_status_ = control::SteeringYawStatus::kNotComputed;
        PreserveTerminalActuatorFailure(state_, MotionPhase::kDisarmed, port::NowMs());
    }
    return {timer_failure_.load(), actuator_disable_ok_.load()};
}

// 处理定时器故障 —— 进入 FAIL_SAFE_LATCHED 并请求退出
void ControlLoop::HandleTimerFailure() {
    if (!running_.exchange(false)) {
        return;
    }

    const uint64_t now_ms = port::NowMs();
    timer_failure_.store(true);
    diagnostics_.Emit({port::DiagnosticLevel::kFailSafe,
                       "control.timer.runtime_failure",
                       "control timer stopped unexpectedly; runtime entered FAIL_SAFE_LATCHED and will shut down",
                       now_ms});
    if (platform_.actuator) {
        if (!platform_.actuator->Disable(diagnostics_)) {
            actuator_disable_ok_.store(false);
            diagnostics_.Emit({port::DiagnosticLevel::kFailSafe,
                               "control.timer.disable_failed",
                               "timer failure handler could not confirm safe actuator disable",
                               now_ms});
        }
    }
    if (actuator_disable_ok_.load()) {
        LatchTimerFailureState(now_ms);
    } else {
        ResetControllerState(yaw_controller_,
                             left_wheel_pid_,
                             right_wheel_pid_,
                             steering_control_memory_,
                             state_);
        latched_yaw_fault_status_ = control::SteeringYawStatus::kNotComputed;
        PreserveTerminalActuatorFailure(state_, MotionPhase::kFailSafeLatched, now_ms);
        std::lock_guard<std::mutex> lock(state_.shared_mutex);
        state_.motion_intent.start_requested = false;
        state_.motion_intent.stop_requested = false;
        state_.motion_intent.reset_fault_requested = false;
    }
    state_.exit_requested.store(true);
    state_.stop_requested.store(true);
}

// 复位到 DISARMED 状态 —— 清理所有运行时状态和指令
void ControlLoop::ResetDisarmedControlState() {
    state_.timer_started = false;
    std::lock_guard<std::mutex> lock(state_.shared_mutex);
    state_.actuators_armed = false;
    state_.last_command = {};
    state_.control_observation = {};
    state_.control_debug_snapshot = {};
    state_.command_history.Clear();
    ResetSteeringControlMemory(steering_control_memory_);
    latched_yaw_fault_status_ = control::SteeringYawStatus::kNotComputed;
    state_.perception_memory_reset_generation.fetch_add(1);
    state_.motion_state.phase = MotionPhase::kDisarmed;
    state_.motion_state.phase_entry_ms = port::NowMs();
    state_.motion_state.fail_safe_latched_at_ms = 0;
    state_.motion_state.clean_gate_cycles = 0;
}

// 锁存定时器故障状态 —— 设置 FAIL_SAFE_LATCHED 并清零所有输出
void ControlLoop::LatchTimerFailureState(uint64_t now_ms) {
    state_.timer_started = false;
    motion_reset_ready_reported_ = false;
    std::lock_guard<std::mutex> lock(state_.shared_mutex);
    state_.actuators_armed = false;
    state_.last_command = {};
    state_.control_observation = {};
    state_.control_observation.motion_phase = MotionPhase::kFailSafeLatched;
    state_.control_observation.hold_disarmed = true;
    state_.control_observation.motion_reset_ready = false;
    state_.control_debug_snapshot = {};
    state_.command_history.Clear();
    ResetSteeringControlMemory(steering_control_memory_);
    latched_yaw_fault_status_ = control::SteeringYawStatus::kNotComputed;
    state_.perception_memory_reset_generation.fetch_add(1);
    state_.control_debug_snapshot.valid = true;
    state_.control_debug_snapshot.timestamp_ms = now_ms;
    state_.control_debug_snapshot.motion_phase = MotionPhase::kFailSafeLatched;
    state_.control_debug_snapshot.emergency_stop = true;
    state_.motion_state.phase = MotionPhase::kFailSafeLatched;
    state_.motion_state.phase_entry_ms = now_ms;
    state_.motion_state.fail_safe_latched_at_ms = now_ms;
    state_.motion_state.clean_gate_cycles = 0;
    state_.motion_intent.start_requested = false;
    state_.motion_intent.stop_requested = false;
    state_.motion_intent.reset_fault_requested = false;
}

// 控制定时器心跳 —— 主控制循环单次迭代：
// 采样（低电压/IMU/编码器/感知）→ 门控评估 → 运动监督 →
// 转向计算（PID + 陀螺仪）→ 执行器命令组合 → 诊断输出
// NOLINTNEXTLINE(readability-function-size): control tick is the single ordered owner for gate, motion, actuator, and snapshot publication.
void ControlLoop::Tick() {
    LS2K_PERF_SCOPE(port::PerfStage::kControlTick);
    if (!running_) {
        return;
    }

    // --- 第 1 阶段：传感器采样 ---
    const bool low_voltage_emergency = state_.low_voltage_emergency.load();
    port::ImuSample imu{};
    {
        LS2K_PERF_SCOPE(port::PerfStage::kControlImuRead);
        imu = platform_.imu->Read(diagnostics_);
    }
    port::EncoderDelta encoder{};
    {
        LS2K_PERF_SCOPE(port::PerfStage::kControlEncoderRead);
        encoder = platform_.encoder->ReadDelta(diagnostics_);
    }

    const uint64_t now_ms = port::NowMs();
    port::PerceptionResult perception{};
    port::ActuatorCommand previous_command{};
    MotionSupervisorState previous_motion_state{};
    MotionIntent motion_intent{};
    RuntimeTuningSnapshot tuning_snapshot{};
    ControlCycleObservation previous_observation{};
    port::MotionHistory motion_history{};
    port::ControlCommandHistory command_history{};
    {
        std::lock_guard<std::mutex> lock(state_.shared_mutex);
        state_.imu = imu;
        state_.encoder = encoder;
        state_.motion_history.Push({now_ms,
                                    imu.valid,
                                    imu.gyro_z,
                                    encoder.valid,
                                    encoder.left,
                                    encoder.right});
        perception = state_.perception;
        motion_history = state_.motion_history;
        command_history = state_.command_history;
        previous_command = state_.last_command;
        previous_motion_state = state_.motion_state;
        motion_intent = state_.motion_intent;
        tuning_snapshot = SnapshotRuntimeTuningState(state_.tuning_state);
        previous_observation = state_.control_observation;
    }

    perception = BuildControlTimePerception(perception, motion_history, command_history, now_ms, params_);
    // --- 第 2 阶段：门控评估 ---
    ControlGateDecision gate{};
    gate =
        EvaluateControlGate(BuildControlGateInputs(perception,
                                                   imu,
                                                   encoder,
                                                   previous_motion_state.phase,
                                                   motion_intent,
                                                   low_voltage_emergency,
                                                   now_ms,
                                                   params_));

    // --- 第 3 阶段：运动监督 ---
    MotionDecision motion = motion_supervisor_.Evaluate(BuildMotionSupervisorInputs(
        state_.startup_complete, previous_motion_state, motion_intent, tuning_snapshot,
        perception, gate, encoder, now_ms, params_));

    if (motion.reset_controllers) {
        ResetControllerState(yaw_controller_,
                             left_wheel_pid_,
                             right_wheel_pid_,
                             steering_control_memory_,
                             state_);
    }

    port::ActuatorCommand command{};
    control::ActuatorCommandShapingResult command_shaping{};
    control::WheelPidComputation left_pid_computation{};
    control::WheelPidComputation right_pid_computation{};
    control::WheelPidCommitResult left_pid_commit{};
    control::WheelPidCommitResult right_pid_commit{};
    bool wheel_pid_evaluated = false;
    control::WheelSpeedTargets wheel_targets{};
    bool hold_disarmed = false;
    int raw_turn_output = 0;
    int applied_turn_output = 0;
    control::TurnOutputTargetComputation turn_output_target_result{};
    control::GyroTurnComputation gyro_turn{};
    control::SteeringYawStatus yaw_fault_status = control::SteeringYawStatus::kNotComputed;
    bool steering_terms_valid = perception.published && perception.fresh &&
                               perception.frame_id != 0 && perception.capture_time_ms != 0;
    // --- 第 4 阶段：转向计算与执行器命令 ---
    if (gate.veto_active || motion.require_emergency_stop) {
        command = {};
    } else if (motion.hold_disarmed || !motion.allow_drive) {
        hold_disarmed = true;
        command = {0, 0, 0, 0, false};
    } else if (motion.state.phase == MotionPhase::kStopping) {
        command_shaping = control::ShapeActuatorCommand(previous_command,
                                                        {0, 0, 0, 0, false},
                                                        params_.pwm_limit,
                                                        params_.pwm_floor,
                                                        params_.prohibit_reverse_pwm,
                                                        params_.drive_pwm_step_limit);
        command = command_shaping.command;
    } else {
        const double constrained_speed_target = motion.effective_speed_target;
        turn_output_target_result =
            yaw_controller_.ComputeTurnOutputTarget(perception.reference_tracking_geometry,
                                                    constrained_speed_target,
                                                    steering_control_memory_.controller_memory);
        if (!turn_output_target_result.valid) {
            yaw_fault_status = turn_output_target_result.status;
        } else {
            gyro_turn = yaw_controller_.ComputeGyroTurn(turn_output_target_result.turn_output_target,
                                                        imu.gyro_z,
                                                        steering_control_memory_.controller_memory);
            if (!gyro_turn.valid) {
                yaw_fault_status = gyro_turn.status;
            } else if (!ClampTurnOutput(gyro_turn.raw_turn_output,
                                        motion.turn_limit_scale,
                                        params_.raw_turn_output_limit,
                                        raw_turn_output)) {
                yaw_fault_status = control::SteeringYawStatus::kNonFiniteArithmetic;
            }
        }

        if (yaw_fault_status == control::SteeringYawStatus::kNotComputed) {
            applied_turn_output = raw_turn_output;
            if (tuning_snapshot.tuning_mode_enabled && tuning_snapshot.turn_suppressed) {
                applied_turn_output = 0;
            }
            const control::WheelTargetMixerParameters mixer_params = BuildWheelTargetMixerParameters(params_);
            wheel_targets =
                wheel_target_mixer_.Compute(constrained_speed_target, applied_turn_output, mixer_params);
            left_pid_computation =
                left_wheel_pid_.Evaluate(wheel_targets.left, encoder.left, params_.pwm_limit);
            right_pid_computation =
                right_wheel_pid_.Evaluate(wheel_targets.right, encoder.right, params_.pwm_limit);
            wheel_pid_evaluated = true;
            const int brushless_pwm =
                params_.brushless_debug_fixed_pwm_enabled ? params_.brushless_debug_fixed_pwm : 0;
            const port::ActuatorCommand requested_command = actuator_command_builder_.Compose(
                left_pid_computation.requested_pwm,
                right_pid_computation.requested_pwm,
                brushless_pwm,
                brushless_pwm,
                false,
                params_.pwm_limit,
                1000);
            command_shaping = control::ShapeActuatorCommand(previous_command,
                                                            requested_command,
                                                            params_.pwm_limit,
                                                            params_.pwm_floor,
                                                            params_.prohibit_reverse_pwm,
                                                            params_.drive_pwm_step_limit);
            command = command_shaping.command;
        }
    }

    if (yaw_fault_status != control::SteeringYawStatus::kNotComputed) {
        if (latched_yaw_fault_status_ == control::SteeringYawStatus::kNotComputed) {
            latched_yaw_fault_status_ = yaw_fault_status;
        }
        gate = {true, ControlVetoReason::kYawControlInvalid};
        motion = motion_supervisor_.Evaluate(BuildMotionSupervisorInputs(
            state_.startup_complete, previous_motion_state, motion_intent, tuning_snapshot,
            perception, gate, encoder, now_ms, params_));
        if (motion.reset_controllers) {
            ResetControllerState(yaw_controller_,
                                 left_wheel_pid_,
                                 right_wheel_pid_,
                                 steering_control_memory_,
                                 state_);
        }
        command = {};
        command_shaping = {};
        wheel_targets = {};
        raw_turn_output = 0;
        applied_turn_output = 0;
        wheel_pid_evaluated = false;
        steering_terms_valid = false;
        port::EmitRateLimited(diagnostics_,
                              {port::DiagnosticLevel::kFailSafe,
                               "control.yaw.invalid",
                               std::string("yaw controller rejected the drive computation before wheel mixing: ") +
                                   control::ToString(yaw_fault_status),
                               now_ms},
                              1000);
    }

    EmitVetoDiagnostics(diagnostics_, gate, now_ms);
    EmitGateIntervalDiagnostics(diagnostics_,
                                have_gate_interval_,
                                last_gate_veto_,
                                last_gate_reason_,
                                gate_interval_start_ms_,
                                gate_interval_reported_,
                                gate,
                                now_ms);
    // --- 第 5 阶段：执行器施加 + 诊断输出 ---
    const bool diagnostics_only_actuator =
        profile_.actuator.mode == port::SubsystemMode::kAdaptationHook ||
        profile_.actuator.mode == port::SubsystemMode::kDisabled;
    const bool current_requested_command_zero =
        !command.emergency_stop && command.left_drive_pwm == 0 && command.right_drive_pwm == 0 &&
        command.left_brushless_pwm == 0 && command.right_brushless_pwm == 0;
    const bool current_effective_command_zero =
        diagnostics_only_actuator || hold_disarmed ||
        (!command.emergency_stop && command.left_drive_pwm == 0 && command.right_drive_pwm == 0 &&
         command.left_brushless_pwm == 0 && command.right_brushless_pwm == 0);

    MotionDecision final_motion = motion;
    if (motion.state.phase == MotionPhase::kStopping) {
        MotionSupervisorInputs stop_completion_inputs = BuildMotionSupervisorInputs(
            state_.startup_complete, motion.state, motion_intent, tuning_snapshot,
            perception, gate, encoder, now_ms, params_);
        // Lifecycle stop completion is keyed to the command that can still reach the actuator path,
        // not the controller's diagnostic-only requested PWM.
        stop_completion_inputs.shaped_command_zero = current_effective_command_zero;
        final_motion = motion_supervisor_.Evaluate(stop_completion_inputs);
        if (final_motion.reset_controllers && !motion.reset_controllers) {
            ResetControllerState(yaw_controller_,
                                 left_wheel_pid_,
                                 right_wheel_pid_,
                                 steering_control_memory_,
                                 state_);
        }
    }
    EmitMotionDiagnostics(diagnostics_, final_motion, gate, motion_reset_ready_reported_, now_ms);
    const control::WheelSpeedTargets snapshot_wheel_targets =
        BuildSnapshotWheelTargets(final_motion,
                                  applied_turn_output,
                                  wheel_target_mixer_,
                                  BuildWheelTargetMixerParameters(params_));

    // The second STOPPING evaluation can complete the lifecycle in this same
    // cycle. All actuator disposition and downstream facts must consume that
    // final decision, rather than the first evaluation's hold state.
    const bool final_hold_disarmed = final_motion.hold_disarmed;
    bool apply_ok = true;
    port::ActuatorCommand applied_command{};
    if (diagnostics_only_actuator || final_hold_disarmed) {
        apply_ok = platform_.actuator->Disable(diagnostics_);
        if (!apply_ok) {
            actuator_disable_ok_.store(false);
            state_.exit_requested.store(true);
            state_.stop_requested.store(true);
        }
    } else {
        LS2K_PERF_SCOPE(port::PerfStage::kControlApply);
        const port::ActuatorApplyResult apply_result =
            platform_.actuator->Apply(command, diagnostics_);
        apply_ok = apply_result.ok;
        applied_command = apply_result.applied_command;
        if (apply_ok) {
            control::ReconcileAppliedDrivePwm(command_shaping.left,
                                              applied_command.left_drive_pwm);
            control::ReconcileAppliedDrivePwm(command_shaping.right,
                                              applied_command.right_drive_pwm);
        }
    }

    const bool actuator_applied =
        apply_ok && !diagnostics_only_actuator && !final_hold_disarmed && !command.emergency_stop;
    if (wheel_pid_evaluated) {
        left_pid_commit = left_wheel_pid_.CommitAppliedOutput(
            left_pid_computation, command_shaping.left, actuator_applied);
        right_pid_commit = right_wheel_pid_.CommitAppliedOutput(
            right_pid_computation, command_shaping.right, actuator_applied);
    } else {
        // STOPPING may complete through the second MotionSupervisor::Evaluate above,
        // which resets both controllers after this cycle's command reaches zero.
        // Snapshot facts must be sampled after that final decision/reset, not cached
        // before command shaping.
        left_pid_commit.integral = left_wheel_pid_.integral();
        right_pid_commit.integral = right_wheel_pid_.integral();
    }

    ControlCycleObservation observation = ObserveControlCycle(
        {gate,
         command,
         final_motion.state.phase,
         apply_ok,
         diagnostics_only_actuator,
         final_hold_disarmed,
         previous_command,
         applied_command,
         previous_observation.actuators_armed});
    observation.motion_reset_ready = final_motion.reset_ready;
    EmitObservationDiagnostics(diagnostics_, previous_observation, observation, now_ms);

    if (final_motion.state.phase != MotionPhase::kFailSafeLatched) {
        latched_yaw_fault_status_ = control::SteeringYawStatus::kNotComputed;
    }
    const control::SteeringYawStatus snapshot_yaw_fault_status =
        latched_yaw_fault_status_ != control::SteeringYawStatus::kNotComputed
            ? latched_yaw_fault_status_
            : yaw_fault_status;
    const ControlDebugSnapshot debug_snapshot =
        BuildControlDebugSnapshot({perception,
                                   encoder,
                                   command,
                                   command_shaping,
                                   left_pid_computation,
                                   right_pid_computation,
                                   left_pid_commit,
                                   right_pid_commit,
                                   observation,
                                   gate,
                                   final_motion,
                                   tuning_snapshot,
                                   snapshot_wheel_targets,
                                   turn_output_target_result,
                                   gyro_turn,
                                   now_ms,
                                   state_.control_cycle_count.load() + 1,
                                   raw_turn_output,
                                   applied_turn_output,
                                   snapshot_yaw_fault_status,
                                   steering_terms_valid});
    debug_reporter_.MaybeEmit(debug_snapshot, diagnostics_);

    port::ControlCommandHistorySample command_history_sample{};
    command_history_sample.time_ms = now_ms;
    command_history_sample.valid = true;
    command_history_sample.actuator_request_succeeded = apply_ok;
    command_history_sample.actuator_applied = actuator_applied;
    command_history_sample.diagnostics_only = diagnostics_only_actuator;
    command_history_sample.hold_disarmed = final_hold_disarmed;
    command_history_sample.emergency_stop = command.emergency_stop;
    command_history_sample.raw_turn_output = raw_turn_output;
    command_history_sample.applied_turn_output = applied_turn_output;
    command_history_sample.left_wheel_target = wheel_targets.left;
    command_history_sample.right_wheel_target = wheel_targets.right;
    command_history_sample.left_drive_pwm = command.left_drive_pwm;
    command_history_sample.right_drive_pwm = command.right_drive_pwm;
    command_history_sample.left_brushless_pwm = command.left_brushless_pwm;
    command_history_sample.right_brushless_pwm = command.right_brushless_pwm;
    command_history_sample.actuators_armed = observation.actuators_armed;
    command_history_sample.last_confirmed_left_drive_pwm = observation.applied_left_drive_pwm;
    command_history_sample.last_confirmed_right_drive_pwm = observation.applied_right_drive_pwm;
    command_history_sample.last_confirmed_left_brushless_pwm = observation.applied_left_brushless_pwm;
    command_history_sample.last_confirmed_right_brushless_pwm = observation.applied_right_brushless_pwm;

    {
        std::lock_guard<std::mutex> lock(state_.shared_mutex);
        state_.motion_state = final_motion.state;
        state_.motion_state.last_shaped_command_zero = current_requested_command_zero;
        if (final_motion.consume_reset_request) {
            state_.motion_intent.reset_fault_requested = false;
        }
        state_.last_command = {observation.applied_left_drive_pwm,
                               observation.applied_right_drive_pwm,
                               observation.applied_left_brushless_pwm,
                               observation.applied_right_brushless_pwm,
                               false};
        state_.control_observation = observation;
        state_.control_debug_snapshot = debug_snapshot;
        state_.actuators_armed = observation.actuators_armed;
        state_.command_history.Push(command_history_sample);
    }
    ++state_.control_cycle_count;
}

}  // namespace ls2k::runtime
