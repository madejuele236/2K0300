#ifndef LS2K_OBSERVABILITY_ASSISTANT_TELEMETRY_VIEW_HPP
#define LS2K_OBSERVABILITY_ASSISTANT_TELEMETRY_VIEW_HPP

#include "transport/assistant_protocol.hpp"
#include "observability/control_debug_snapshot.hpp"

namespace ls2k::observability {

/// 构建辅助遥测视图：将 ControlDebugSnapshot 中的字段映射到 AssistantTelemetryView
/// @param snapshot  控制调试快照（包含运动、感知、转向等全部调试信息）
/// @return          填充好的 AssistantTelemetryView 结构体
inline transport::AssistantTelemetryView BuildAssistantTelemetryView(
    const ControlDebugSnapshot& snapshot) {
    transport::AssistantTelemetryView telemetry{};
    telemetry.motion_phase = ToString(snapshot.motion_phase);
    telemetry.perception_tag = snapshot.steering.perception_tag;
    telemetry.binary_model = snapshot.steering.binary_model;
    telemetry.boundary_row_count = snapshot.steering.boundary_row_count;
    telemetry.boundary_jump_count = snapshot.steering.boundary_jump_count;
    telemetry.boundary_span_count = snapshot.steering.boundary_span_count;
    telemetry.ml = snapshot.steering.ml;
    telemetry.speed_selection_source = snapshot.steering.speed_selection_source;
    telemetry.element_evidence = snapshot.steering.element_evidence;
    telemetry.circle_v2 = snapshot.steering.circle_v2;
    telemetry.visual_reference.present = snapshot.steering.visual_reference.present;
    telemetry.visual_reference.source = snapshot.steering.visual_reference.source;
    telemetry.visual_reference.reason = snapshot.steering.visual_reference.reason;
    telemetry.visual_reference.candidate_count =
        snapshot.steering.visual_reference.candidate_count;
    telemetry.visual_reference.rejected_candidate_reason =
        snapshot.steering.visual_reference.rejected_candidate_reason;
    telemetry.reference.mode = snapshot.steering.reference.mode;
    telemetry.reference.source = snapshot.steering.reference.source;
    telemetry.eligibility.usable = snapshot.steering.eligibility.usable;
    telemetry.eligibility.leading_usable_samples =
        snapshot.steering.eligibility.leading_usable_samples;
    telemetry.eligibility.leading_min_forward_m =
        snapshot.steering.eligibility.leading_min_forward_m;
    telemetry.eligibility.leading_max_forward_m =
        snapshot.steering.eligibility.leading_max_forward_m;
    telemetry.eligibility.reason = snapshot.steering.eligibility.reason;
    telemetry.lateral_error.computed = snapshot.steering.lateral_error.computed;
    telemetry.lateral_error.weighted_lateral_error_m =
        snapshot.steering.lateral_error.weighted_lateral_error_m;
    telemetry.lateral_error.weighted_sample_count =
        snapshot.steering.lateral_error.weighted_sample_count;
    telemetry.lateral_error.weight_sum = snapshot.steering.lateral_error.weight_sum;
    telemetry.lateral_error.reason = snapshot.steering.lateral_error.reason;
    telemetry.reference_tracking_geometry.computed = snapshot.steering.reference_tracking_geometry.computed;
    telemetry.reference_tracking_geometry.lateral_offset_m =
        snapshot.steering.reference_tracking_geometry.lateral_offset_m;
    telemetry.reference_tracking_geometry.heading_error_rad =
        snapshot.steering.reference_tracking_geometry.heading_error_rad;
    telemetry.reference_tracking_geometry.curvature_m_inv =
        snapshot.steering.reference_tracking_geometry.curvature_m_inv;
    telemetry.reference_tracking_geometry.sample_count =
        snapshot.steering.reference_tracking_geometry.sample_count;
    telemetry.reference_tracking_geometry.reason = snapshot.steering.reference_tracking_geometry.reason;
    telemetry.perception_health.projector_ok =
        snapshot.steering.perception_health.projector_ok;
    telemetry.perception_health.reason = snapshot.steering.perception_health.reason;
    telemetry.reference_control.ready = snapshot.steering.reference_control.ready;
    telemetry.reference_control.reason = snapshot.steering.reference_control.reason;
    telemetry.safety_gate.veto_active = snapshot.steering.safety_gate.veto_active;
    telemetry.safety_gate.reason = snapshot.steering.safety_gate.reason;
    telemetry.degraded.active = snapshot.steering.degraded.active;
    telemetry.degraded.reason = snapshot.steering.degraded.reason;
    telemetry.yaw_control.valid = snapshot.steering.yaw_control.valid;
    telemetry.yaw_control.reason = snapshot.steering.yaw_control.reason;
    telemetry.yaw_control.turn_output_target =
        snapshot.steering.yaw_control.turn_output_target;
    telemetry.yaw_control.lateral_term = snapshot.steering.yaw_control.lateral_term;
    telemetry.yaw_control.heading_term = snapshot.steering.yaw_control.heading_term;
    telemetry.yaw_control.curvature_term = snapshot.steering.yaw_control.curvature_term;
    telemetry.tuning_mode_enabled = snapshot.tuning_mode_enabled;
    telemetry.turn_suppressed = snapshot.turn_suppressed;
    telemetry.target_speed_override_enabled = snapshot.target_speed_override_enabled;
    telemetry.target_speed_override_value = snapshot.target_speed_override_value;
    telemetry.effective_speed_target = snapshot.steering.effective_speed_target;
    telemetry.left_speed_target = snapshot.left_speed_target;
    telemetry.right_speed_target = snapshot.right_speed_target;
    telemetry.left_measured_speed = snapshot.left_measured_speed;
    telemetry.right_measured_speed = snapshot.right_measured_speed;
    telemetry.raw_turn_output = snapshot.raw_turn_output;
    telemetry.applied_turn_output = snapshot.applied_turn_output;
    telemetry.left_drive_pwm_command = snapshot.left_drive_pwm_command;
    telemetry.right_drive_pwm_command = snapshot.right_drive_pwm_command;
    telemetry.left_brushless_pwm_command = snapshot.left_brushless_pwm_command;
    telemetry.right_brushless_pwm_command = snapshot.right_brushless_pwm_command;
    telemetry.left_drive_pwm_unconstrained = snapshot.left_drive_pwm_unconstrained;
    telemetry.right_drive_pwm_unconstrained = snapshot.right_drive_pwm_unconstrained;
    telemetry.left_drive_pwm_requested = snapshot.left_drive_pwm_requested;
    telemetry.right_drive_pwm_requested = snapshot.right_drive_pwm_requested;
    telemetry.left_drive_pwm_desired = snapshot.left_drive_pwm_desired;
    telemetry.right_drive_pwm_desired = snapshot.right_drive_pwm_desired;
    telemetry.left_drive_pwm_step_limited = snapshot.left_drive_pwm_step_limited;
    telemetry.right_drive_pwm_step_limited = snapshot.right_drive_pwm_step_limited;
    telemetry.left_drive_pwm_reverse_suppressed = snapshot.left_drive_pwm_reverse_suppressed;
    telemetry.right_drive_pwm_reverse_suppressed = snapshot.right_drive_pwm_reverse_suppressed;
    telemetry.left_drive_pwm_floor_adjusted = snapshot.left_drive_pwm_floor_adjusted;
    telemetry.right_drive_pwm_floor_adjusted = snapshot.right_drive_pwm_floor_adjusted;
    telemetry.left_pid_error = snapshot.left_pid_error;
    telemetry.right_pid_error = snapshot.right_pid_error;
    telemetry.left_pid_integral = snapshot.left_pid_integral;
    telemetry.right_pid_integral = snapshot.right_pid_integral;
    telemetry.left_pid_integral_candidate = snapshot.left_pid_integral_candidate;
    telemetry.right_pid_integral_candidate = snapshot.right_pid_integral_candidate;
    telemetry.left_pid_anti_windup_active = snapshot.left_pid_anti_windup_active;
    telemetry.right_pid_anti_windup_active = snapshot.right_pid_anti_windup_active;
    telemetry.left_pid_anti_windup_reason = control::ToString(snapshot.left_pid_anti_windup_reason);
    telemetry.right_pid_anti_windup_reason = control::ToString(snapshot.right_pid_anti_windup_reason);
    telemetry.actuator_apply_outcome = ToString(snapshot.apply_outcome);
    telemetry.actuators_armed = snapshot.actuators_armed;
    telemetry.last_confirmed_left_drive_pwm = snapshot.last_confirmed_left_drive_pwm;
    telemetry.last_confirmed_right_drive_pwm = snapshot.last_confirmed_right_drive_pwm;
    telemetry.last_confirmed_left_brushless_pwm = snapshot.last_confirmed_left_brushless_pwm;
    telemetry.last_confirmed_right_brushless_pwm = snapshot.last_confirmed_right_brushless_pwm;
    return telemetry;
}

}  // namespace ls2k::observability

#endif  // LS2K_OBSERVABILITY_ASSISTANT_TELEMETRY_VIEW_HPP
