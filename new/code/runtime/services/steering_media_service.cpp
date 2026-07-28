#include "runtime/services/steering_media_service.hpp"

/// 转向媒体服务实现 —— 将调试快照与相机帧打包为媒体帧并发布。
/// 通过 SteeringMediaLink 发送到外部媒体接收端（如远程监控）。

#include <algorithm>
#include <optional>
#include <sstream>
#include <utility>

#include "port/perf_counter.hpp"
#include "vision/image/luma_sampler.hpp"

namespace ls2k::runtime {

using control::MotionPhase;
using observability::ControlDebugSnapshot;
using observability::SteeringDebugSnapshot;

namespace {

}  // namespace

/// 构造转向媒体服务并绑定媒体链路
/// @param link  媒体链路实例（支持移动语义）
SteeringMediaService::SteeringMediaService(transport::SteeringMediaLink link)
    : link_(std::move(link)) {}

/// 启动媒体服务：配置参数、初始化媒体链路（若启用）
/// @param params       运行时参数
/// @param diagnostics  诊断输出接口
void SteeringMediaService::Start(const port::RuntimeParameters& params, port::DiagnosticSink& diagnostics) {
    configured_ = true;
    enabled_ = params.steering_media_enabled;
    config_sent_ = false;
    publish_disarmed_ = params.steering_media_publish_disarmed;
    publish_latest_frame_ = params.steering_media_publish_latest_frame;
    downsample_ = std::clamp(params.steering_media_downsample, 1, 8);
    gray_bits_ = params.steering_media_gray_bits == 1 ||
                         params.steering_media_gray_bits == 2 ||
                         params.steering_media_gray_bits == 4
                     ? params.steering_media_gray_bits
                     : 8;
    publish_interval_ms_ = std::max(0, params.steering_media_publish_interval_ms);
    last_image_publish_ms_ = 0;
    last_image_frame_id_ = 0;
    last_summary_ms_ = port::NowMs();
    ResetWindowStats();
    params_ = params;
    if (!enabled_) {
        diagnostics.Emit({port::DiagnosticLevel::kInfo,
                          "steering_media.disabled",
                          "steering media sidecar disabled by runtime parameters",
                          port::NowMs()});
        return;
    }
    (void)link_.Initialize(params, diagnostics);
}

/// 重置窗口统计（清空所有计数器）
void SteeringMediaService::ResetWindowStats() {
    window_stats_ = WindowStats{};
}

/// 按每秒间隔发射窗口统计摘要：汇总各状态计数并输出诊断
/// @param now_ms      当前时间戳（ms）
/// @param diagnostics 诊断输出接口
void SteeringMediaService::MaybeEmitWindowSummary(std::uint64_t now_ms,
                                                  port::DiagnosticSink& diagnostics) {
    if (last_summary_ms_ == 0) {
        last_summary_ms_ = now_ms;
        return;
    }
    if (now_ms < last_summary_ms_ || now_ms - last_summary_ms_ < 1000U) {
        return;
    }
    last_summary_ms_ = now_ms;
    if (!diagnostics.ShouldEmit(port::DiagnosticLevel::kInfo, "steering_media.summary")) {
        ResetWindowStats();
        return;
    }

    std::ostringstream message;
    message << "ticks=" << window_stats_.ticks
            << " ready=" << (link_.Ready() ? "true" : "false")
            << " config_sent=" << (config_sent_ ? "true" : "false")
            << " publish_interval_ms=" << publish_interval_ms_
            << " gray_bits=" << gray_bits_
            << " latest_frame=" << (publish_latest_frame_ ? "true" : "false")
            << " last_frame_id=" << last_image_frame_id_
            << " not_ready=" << window_stats_.not_ready
            << " pending_flush_sent=" << window_stats_.pending_flush_sent
            << " config_attempts=" << window_stats_.config_attempts
            << " config_sent_count=" << window_stats_.config_sent
            << " config_wait=" << window_stats_.config_wait
            << " skip_no_capture=" << window_stats_.skip_no_capture
            << " skip_zero_frame=" << window_stats_.skip_zero_frame
            << " skip_disarmed=" << window_stats_.skip_disarmed
            << " skip_duplicate=" << window_stats_.skip_duplicate
            << " skip_interval=" << window_stats_.skip_interval
            << " image_sent=" << window_stats_.image_sent
            << " image_queued=" << window_stats_.image_queued
            << " image_unavailable=" << window_stats_.image_unavailable;
    diagnostics.Emit({port::DiagnosticLevel::kInfo,
                      "steering_media.summary",
                      message.str(),
                      now_ms});
    ResetWindowStats();
}

/// 构建参数配置快照 —— 导出当前运行时参数到媒体协议格式
transport::SteeringMediaConfigSnapshot SteeringMediaService::BuildConfigSnapshot(std::uint64_t now_ms) const {
    transport::SteeringMediaConfigSnapshot snapshot{};
    snapshot.publish_time_ms = now_ms;
    snapshot.media_publish_interval_ms = publish_interval_ms_;
    snapshot.param_snapshot.running_speed_target = params_.running_speed_target;
    snapshot.param_snapshot.yaw_rate_pid_p = params_.yaw_rate_pid_p;
    snapshot.param_snapshot.yaw_rate_pid_i = params_.yaw_rate_pid_i;
    snapshot.param_snapshot.yaw_rate_pid_d = params_.yaw_rate_pid_d;
    snapshot.param_snapshot.control_period_ms = params_.control_period_ms;
    snapshot.param_snapshot.low_voltage_sample_interval_ms = params_.low_voltage_sample_interval_ms;
    snapshot.param_snapshot.low_voltage_raw_threshold = params_.low_voltage_raw_threshold;
    snapshot.param_snapshot.raw_turn_output_limit = params_.raw_turn_output_limit;
    snapshot.param_snapshot.pwm_limit = params_.pwm_limit;
    snapshot.param_snapshot.pwm_floor = params_.pwm_floor;
    snapshot.param_snapshot.prohibit_reverse_pwm = params_.prohibit_reverse_pwm;
    snapshot.param_snapshot.drive_pwm_step_limit = params_.drive_pwm_step_limit;
    snapshot.param_snapshot.left_wheel_pid = params_.left_wheel_pid;
    snapshot.param_snapshot.right_wheel_pid = params_.right_wheel_pid;
    snapshot.param_snapshot.wheel_turn_accel_delta_scale = params_.wheel_turn_accel_delta_scale;
    snapshot.param_snapshot.wheel_turn_decel_delta_scale = params_.wheel_turn_decel_delta_scale;
    snapshot.param_snapshot.bev_projector = params_.bev_projector;
    snapshot.param_snapshot.bev_geometry = params_.bev_geometry;
    snapshot.param_snapshot.bev_classification = params_.bev_classification;
    snapshot.param_snapshot.bev_control_model = params_.bev_control_model;
    snapshot.param_snapshot.bev_element = params_.bev_element;
    snapshot.param_snapshot.reference_time_alignment = params_.reference_time_alignment;
    snapshot.param_snapshot.motion_odometry = params_.motion_odometry;
    snapshot.param_snapshot.ml = params_.ml;
    return snapshot;
}

// 构建转向快照视图 —— 将 DebugSnapshot 转换为媒体协议视图
/// 将 SteeringDebugSnapshot 转换为媒体协议视图（SteeringMediaSnapshotView）
/// @param snapshot  转向调试快照
/// @return          媒体协议格式的快照视图
transport::SteeringMediaSnapshotView SteeringMediaService::BuildSnapshotView(
    const SteeringDebugSnapshot& snapshot) const {
    transport::SteeringMediaSnapshotView view{};
    view.binary_model = snapshot.binary_model;
    view.perception_tag = snapshot.perception_tag;
    view.boundary_row_count = snapshot.boundary_row_count;
    view.boundary_jump_count = snapshot.boundary_jump_count;
    view.boundary_span_count = snapshot.boundary_span_count;
    view.ml = snapshot.ml;
    view.speed_selection_source = snapshot.speed_selection_source;
    view.effective_speed_target = snapshot.effective_speed_target;
    view.perception_health.projector_ok = snapshot.perception_health.projector_ok;
    view.perception_health.reason = snapshot.perception_health.reason;
    view.element_evidence = snapshot.element_evidence;
    view.circle_v2 = snapshot.circle_v2;
    view.visual_reference.present = snapshot.visual_reference.present;
    view.visual_reference.source = snapshot.visual_reference.source;
    view.visual_reference.reason = snapshot.visual_reference.reason;
    view.visual_reference.candidate_count = snapshot.visual_reference.candidate_count;
    view.visual_reference.rejected_candidate_reason =
        snapshot.visual_reference.rejected_candidate_reason;
    view.visual_reference.candidate_paths = snapshot.visual_reference.candidate_paths;
    view.reference.mode = snapshot.reference.mode;
    view.reference.source = snapshot.reference.source;
    view.reference.control_path = snapshot.reference.control_path;
    view.eligibility.usable = snapshot.eligibility.usable;
    view.eligibility.leading_usable_samples = snapshot.eligibility.leading_usable_samples;
    view.eligibility.leading_min_forward_m = snapshot.eligibility.leading_min_forward_m;
    view.eligibility.leading_max_forward_m = snapshot.eligibility.leading_max_forward_m;
    view.eligibility.reason = snapshot.eligibility.reason;
    view.lateral_error.computed = snapshot.lateral_error.computed;
    view.lateral_error.weighted_lateral_error_m = snapshot.lateral_error.weighted_lateral_error_m;
    view.lateral_error.weighted_sample_count = snapshot.lateral_error.weighted_sample_count;
    view.lateral_error.weight_sum = snapshot.lateral_error.weight_sum;
    view.lateral_error.reason = snapshot.lateral_error.reason;
    view.reference_tracking_geometry.computed = snapshot.reference_tracking_geometry.computed;
    view.reference_tracking_geometry.lateral_offset_m = snapshot.reference_tracking_geometry.lateral_offset_m;
    view.reference_tracking_geometry.heading_error_rad = snapshot.reference_tracking_geometry.heading_error_rad;
    view.reference_tracking_geometry.curvature_m_inv = snapshot.reference_tracking_geometry.curvature_m_inv;
    view.reference_tracking_geometry.sample_count = snapshot.reference_tracking_geometry.sample_count;
    view.reference_tracking_geometry.reason = snapshot.reference_tracking_geometry.reason;
    view.reference_time_alignment.enabled = snapshot.reference_time_alignment.enabled;
    view.reference_time_alignment.valid = snapshot.reference_time_alignment.valid;
    view.reference_time_alignment.reason = snapshot.reference_time_alignment.reason;
    view.reference_time_alignment.age_ms = snapshot.reference_time_alignment.age_ms;
    view.reference_time_alignment.reference_capture_time_ms =
        snapshot.reference_time_alignment.reference_capture_time_ms;
    view.reference_time_alignment.control_time_ms =
        snapshot.reference_time_alignment.control_time_ms;
    view.reference_time_alignment.control_effective_time_ms =
        snapshot.reference_time_alignment.control_effective_time_ms;
    view.reference_time_alignment.measured_until_ms =
        snapshot.reference_time_alignment.measured_until_ms;
    view.reference_time_alignment.predicted_ms = snapshot.reference_time_alignment.predicted_ms;
    view.reference_time_alignment.delta_forward_m =
        snapshot.reference_time_alignment.delta_forward_m;
    view.reference_time_alignment.delta_lateral_m =
        snapshot.reference_time_alignment.delta_lateral_m;
    view.reference_time_alignment.delta_yaw_rad = snapshot.reference_time_alignment.delta_yaw_rad;
    view.reference_time_alignment.measured_forward_mps =
        snapshot.reference_time_alignment.measured_forward_mps;
    view.reference_time_alignment.measured_yaw_rate_radps =
        snapshot.reference_time_alignment.measured_yaw_rate_radps;
    view.reference_time_alignment.predicted_forward_mps =
        snapshot.reference_time_alignment.predicted_forward_mps;
    view.reference_time_alignment.predicted_yaw_rate_radps =
        snapshot.reference_time_alignment.predicted_yaw_rate_radps;
    view.reference_time_alignment.used_encoder_forward =
        snapshot.reference_time_alignment.used_encoder_forward;
    view.reference_time_alignment.used_imu_yaw = snapshot.reference_time_alignment.used_imu_yaw;
    view.reference_time_alignment.used_wheel_yaw = snapshot.reference_time_alignment.used_wheel_yaw;
    view.reference_time_alignment.used_command_prediction =
        snapshot.reference_time_alignment.used_command_prediction;
    view.reference_time_alignment.input_sample_count =
        snapshot.reference_time_alignment.input_sample_count;
    view.reference_time_alignment.aligned_sample_count =
        snapshot.reference_time_alignment.aligned_sample_count;
    view.reference_control.ready = snapshot.reference_control.ready;
    view.reference_control.reason = snapshot.reference_control.reason;
    view.safety_gate.veto_active = snapshot.safety_gate.veto_active;
    view.safety_gate.reason = snapshot.safety_gate.reason;
    view.degraded.active = snapshot.degraded.active;
    view.degraded.reason = snapshot.degraded.reason;
    view.yaw_control.valid = snapshot.yaw_control.valid;
    view.yaw_control.reason = snapshot.yaw_control.reason;
    view.yaw_control.turn_output_target = snapshot.yaw_control.turn_output_target;
    view.yaw_control.lateral_term = snapshot.yaw_control.lateral_term;
    view.yaw_control.heading_term = snapshot.yaw_control.heading_term;
    view.yaw_control.curvature_term = snapshot.yaw_control.curvature_term;
    view.actuator.raw_turn_output = snapshot.actuator.raw_turn_output;
    view.actuator.applied_turn_output = snapshot.actuator.applied_turn_output;
    view.actuator.left_drive_pwm_command = snapshot.actuator.left_drive_pwm_command;
    view.actuator.right_drive_pwm_command = snapshot.actuator.right_drive_pwm_command;
    view.actuator.left_brushless_pwm_command = snapshot.actuator.left_brushless_pwm_command;
    view.actuator.right_brushless_pwm_command = snapshot.actuator.right_brushless_pwm_command;
    view.actuator.left_drive_pwm_unconstrained = snapshot.actuator.left_drive_pwm_unconstrained;
    view.actuator.right_drive_pwm_unconstrained = snapshot.actuator.right_drive_pwm_unconstrained;
    view.actuator.left_drive_pwm_requested = snapshot.actuator.left_drive_pwm_requested;
    view.actuator.right_drive_pwm_requested = snapshot.actuator.right_drive_pwm_requested;
    view.actuator.left_drive_pwm_desired = snapshot.actuator.left_drive_pwm_desired;
    view.actuator.right_drive_pwm_desired = snapshot.actuator.right_drive_pwm_desired;
    view.actuator.left_drive_pwm_step_limited = snapshot.actuator.left_drive_pwm_step_limited;
    view.actuator.right_drive_pwm_step_limited = snapshot.actuator.right_drive_pwm_step_limited;
    view.actuator.left_drive_pwm_reverse_suppressed = snapshot.actuator.left_drive_pwm_reverse_suppressed;
    view.actuator.right_drive_pwm_reverse_suppressed = snapshot.actuator.right_drive_pwm_reverse_suppressed;
    view.actuator.left_drive_pwm_floor_adjusted = snapshot.actuator.left_drive_pwm_floor_adjusted;
    view.actuator.right_drive_pwm_floor_adjusted = snapshot.actuator.right_drive_pwm_floor_adjusted;
    view.actuator.left_pid_error = snapshot.actuator.left_pid_error;
    view.actuator.right_pid_error = snapshot.actuator.right_pid_error;
    view.actuator.left_pid_integral = snapshot.actuator.left_pid_integral;
    view.actuator.right_pid_integral = snapshot.actuator.right_pid_integral;
    view.actuator.left_pid_integral_candidate = snapshot.actuator.left_pid_integral_candidate;
    view.actuator.right_pid_integral_candidate = snapshot.actuator.right_pid_integral_candidate;
    view.actuator.left_pid_anti_windup_active = snapshot.actuator.left_pid_anti_windup_active;
    view.actuator.right_pid_anti_windup_active = snapshot.actuator.right_pid_anti_windup_active;
    view.actuator.left_pid_anti_windup_reason = control::ToString(snapshot.actuator.left_pid_anti_windup_reason);
    view.actuator.right_pid_anti_windup_reason = control::ToString(snapshot.actuator.right_pid_anti_windup_reason);
    view.actuator.apply_outcome = ToString(snapshot.actuator.apply_outcome);
    view.actuator.actuators_armed = snapshot.actuator.actuators_armed;
    view.actuator.last_confirmed_left_drive_pwm = snapshot.actuator.last_confirmed_left_drive_pwm;
    view.actuator.last_confirmed_right_drive_pwm = snapshot.actuator.last_confirmed_right_drive_pwm;
    view.actuator.last_confirmed_left_brushless_pwm = snapshot.actuator.last_confirmed_left_brushless_pwm;
    view.actuator.last_confirmed_right_brushless_pwm = snapshot.actuator.last_confirmed_right_brushless_pwm;
    return view;
}

/// 填充图像帧数据：从相机像素帧读取 luma，降采样后填入媒体帧结构
/// @param capture_frame  原始相机像素帧
/// @param frame          输出：媒体图像帧（含降采样后的像素数据）
void SteeringMediaService::FillImageFrame(const port::CameraPixelFrameView& capture_frame,
                                          transport::SteeringMediaImageFrame& frame) {
    frame.source_width = capture_frame.width;
    frame.source_height = capture_frame.height;
    frame.downsample = downsample_;
    auto set_payload = [&](const std::uint8_t* pixels, std::size_t pixel_count) {
        if (gray_bits_ < 8) {
            const std::size_t packed_bits = pixel_count * static_cast<std::size_t>(gray_bits_);
            gray_pack_buffer_.assign((packed_bits + 7U) / 8U, 0);
            const int max_level = (1 << gray_bits_) - 1;
            for (std::size_t index = 0; index < pixel_count; ++index) {
                const int level =
                    std::min(max_level, (static_cast<int>(pixels[index]) * max_level + 127) / 255);
                const std::size_t bit_index = index * static_cast<std::size_t>(gray_bits_);
                const std::size_t byte_index = bit_index / 8U;
                const int bit_offset = static_cast<int>(bit_index % 8U);
                const int shift = 8 - gray_bits_ - bit_offset;
                gray_pack_buffer_[byte_index] =
                    static_cast<std::uint8_t>(gray_pack_buffer_[byte_index] |
                                              static_cast<std::uint8_t>(level << shift));
            }
            frame.pixel_format = gray_bits_ == 4 ? "gray4" : (gray_bits_ == 2 ? "gray2" : "gray1");
            frame.pixel_data = gray_pack_buffer_.data();
            frame.pixel_size = gray_pack_buffer_.size();
            return;
        }
        frame.pixel_format = "gray8";
        frame.pixel_data = pixels;
        frame.pixel_size = pixel_count;
    };

    if (downsample_ <= 1) {
        frame.width = capture_frame.width;
        frame.height = capture_frame.height;
        const std::size_t pixel_count =
            static_cast<std::size_t>(capture_frame.width) *
            static_cast<std::size_t>(capture_frame.height);
        downsample_buffer_.assign(pixel_count, 0);
        for (int row = 0; row < capture_frame.height; ++row) {
            for (int col = 0; col < capture_frame.width; ++col) {
                std::uint8_t y = 0;
                if (vision::SampleLumaAt(capture_frame,
                                         static_cast<float>(row),
                                         static_cast<float>(col),
                                         y)) {
                    downsample_buffer_[static_cast<std::size_t>(row) *
                                           static_cast<std::size_t>(capture_frame.width) +
                                       static_cast<std::size_t>(col)] = y;
                }
            }
        }
        set_payload(downsample_buffer_.data(), downsample_buffer_.size());
        return;
    }

    const int output_width = (capture_frame.width + downsample_ - 1) / downsample_;
    const int output_height = (capture_frame.height + downsample_ - 1) / downsample_;
    downsample_buffer_.assign(static_cast<std::size_t>(output_width * output_height), 0);
    for (int out_row = 0; out_row < output_height; ++out_row) {
        const int src_row = std::min(capture_frame.height - 1, out_row * downsample_);
        for (int out_col = 0; out_col < output_width; ++out_col) {
            const int src_col = std::min(capture_frame.width - 1, out_col * downsample_);
            std::uint8_t y = 0;
            if (vision::SampleLumaAt(capture_frame,
                                     static_cast<float>(src_row),
                                     static_cast<float>(src_col),
                                     y)) {
                downsample_buffer_[static_cast<std::size_t>(out_row * output_width + out_col)] = y;
            }
        }
    }

    frame.width = output_width;
    frame.height = output_height;
    set_payload(downsample_buffer_.data(), downsample_buffer_.size());
}

/// 媒体服务 Tick：检查连接 → 检查并发布挂起图像 → 发布配置快照 → 查找新帧 → 发布图像帧。
/// 统计每秒窗口摘要并输出诊断。
/// @param state       运行时状态
/// @param diagnostics 诊断输出接口
// NOLINTNEXTLINE(readability-function-size): media tick is a linear publish/connection orchestration surface.
void SteeringMediaService::Tick(RuntimeState& state,
                                CameraFrameStore& frame_store,
                                port::DiagnosticSink& diagnostics) {
    if (!configured_ || !enabled_) {
        return;
    }

    const std::uint64_t now_ms = port::NowMs();
    window_stats_.ticks += 1U;
    const transport::SteeringMediaLinkPollResult poll_result = link_.Poll(diagnostics);
    if (poll_result.became_ready || poll_result.connection_lost) {
        config_sent_ = false;
        last_image_frame_id_ = 0;
    }

    if (link_.FlushPendingImage(diagnostics)) {
        window_stats_.pending_flush_sent += 1U;
    }
    if (!poll_result.ready) {
        window_stats_.not_ready += 1U;
        MaybeEmitWindowSummary(now_ms, diagnostics);
        return;
    }

    if (!config_sent_) {
        window_stats_.config_attempts += 1U;
        config_sent_ = link_.PublishConfigSnapshot(BuildConfigSnapshot(now_ms), diagnostics);
        if (config_sent_) {
            window_stats_.config_sent += 1U;
        }
        if (!config_sent_) {
            window_stats_.config_wait += 1U;
            MaybeEmitWindowSummary(now_ms, diagnostics);
            return;
        }
    }

    ControlDebugSnapshot snapshot{};
    port::CameraFrameStoreHealth camera_store_health{};
    {
        std::lock_guard<std::mutex> lock(state.shared_mutex);
        snapshot = state.control_debug_snapshot;
    }
    std::optional<CameraFrameStore::ReadLease> capture =
        publish_latest_frame_
            ? frame_store.AcquireLatest()
            : frame_store.AcquireExact(snapshot.steering.frame_id,
                                       snapshot.steering.capture_time_ms);
    const bool have_capture = capture.has_value();
    camera_store_health = frame_store.Health();

    if (!have_capture || snapshot.steering.frame_id == 0) {
        if (!have_capture) {
            window_stats_.skip_no_capture += 1U;
        }
        if (snapshot.steering.frame_id == 0) {
            window_stats_.skip_zero_frame += 1U;
        }
        MaybeEmitWindowSummary(now_ms, diagnostics);
        return;
    }
    if (!publish_disarmed_ && snapshot.motion_phase == MotionPhase::kDisarmed) {
        window_stats_.skip_disarmed += 1U;
        MaybeEmitWindowSummary(now_ms, diagnostics);
        return;
    }
    const CameraFrameHandle& capture_handle = capture->Handle();
    if (capture_handle.frame_id == last_image_frame_id_) {
        window_stats_.skip_duplicate += 1U;
        MaybeEmitWindowSummary(now_ms, diagnostics);
        return;
    }
    if (publish_interval_ms_ > 0 && last_image_publish_ms_ != 0 &&
        now_ms >= last_image_publish_ms_ &&
        now_ms - last_image_publish_ms_ < static_cast<std::uint64_t>(publish_interval_ms_)) {
        window_stats_.skip_interval += 1U;
        MaybeEmitWindowSummary(now_ms, diagnostics);
        return;
    }

    transport::SteeringMediaImageFrame frame{};
    frame.frame_id = capture_handle.frame_id;
    frame.capture_time_ms = capture_handle.capture_time_ms;
    frame.publish_time_ms = now_ms;
    frame.camera_metadata = capture_handle.metadata;
    frame.camera_store_health = camera_store_health;
    frame.source_stride = capture_handle.stride;
    frame.motion_phase = ToString(snapshot.motion_phase);
    frame.steering_snapshot = BuildSnapshotView(snapshot.steering);
    frame.frame_source = publish_latest_frame_ ? "latest_camera_frame" : "snapshot_aligned";
    frame.steering_snapshot_frame_id = snapshot.steering.frame_id;
    frame.steering_snapshot_capture_time_ms = snapshot.steering.capture_time_ms;
    frame.steering_snapshot_aligned =
        snapshot.steering.frame_id == capture_handle.frame_id &&
        snapshot.steering.capture_time_ms == capture_handle.capture_time_ms;
    FillImageFrame(capture->PixelView(), frame);
    if (snapshot.steering.ml.roi.valid) {
        frame.auxiliary_data = snapshot.steering.ml.roi.gray.data();
        frame.auxiliary_size = snapshot.steering.ml.roi.gray.size();
        frame.auxiliary_width = port::kMlRoiSide;
        frame.auxiliary_height = port::kMlRoiSide;
        frame.auxiliary_name = "ml_roi";
    }

    const transport::SteeringMediaPublishResult result = link_.PublishImageFrame(frame, diagnostics);
    if (result == transport::SteeringMediaPublishResult::kSent ||
        result == transport::SteeringMediaPublishResult::kQueued) {
        last_image_publish_ms_ = now_ms;
        last_image_frame_id_ = capture_handle.frame_id;
    }
    if (result == transport::SteeringMediaPublishResult::kSent) {
        window_stats_.image_sent += 1U;
    } else if (result == transport::SteeringMediaPublishResult::kQueued) {
        window_stats_.image_queued += 1U;
    } else {
        window_stats_.image_unavailable += 1U;
    }
    MaybeEmitWindowSummary(now_ms, diagnostics);
}

}  // namespace ls2k::runtime
