#include "vision/ml/ml_inertial_path_tracker.hpp"

#include <algorithm>
#include <cmath>

#include "vision/ml/ml_reference_adapter.hpp"

namespace ls2k::vision::ml {
namespace {

void IntegrateBodyStep(port::MlPoseDelta& pose, float ds_m, float dyaw_rad) {
    const float theta_mid = pose.yaw_rad + 0.5F * dyaw_rad;
    pose.forward_m += std::cos(theta_mid) * ds_m;
    pose.lateral_m += std::sin(theta_mid) * ds_m;
    pose.yaw_rad += dyaw_rad;
}

bool IntegrateTo(const port::MotionHistory& history,
                 uint64_t target_time_ms,
                 double encoder_ticks_to_meter,
                 int max_gap_ms,
                 port::MlInertialTrackerState& state,
                 const char*& reason) {
    if (history.count < 2U) { reason = "motion_history_unavailable"; return false; }
    if (!(encoder_ticks_to_meter > 0.0) || !std::isfinite(encoder_ticks_to_meter)) {
        reason = "encoder_scale_unavailable";
        return false;
    }
    if (target_time_ms < state.cursor_time_ms) { reason = "time_regressed"; return false; }
    uint64_t cursor = state.cursor_time_ms;
    const uint64_t max_gap = static_cast<uint64_t>(std::max(1, max_gap_ms));
    for (std::size_t index = 1U; index < history.count && cursor < target_time_ms; ++index) {
        const port::MotionHistorySample& prev = history.OldestOffset(index - 1U);
        const port::MotionHistorySample& curr = history.OldestOffset(index);
        if (curr.time_ms <= cursor) continue;
        if (curr.time_ms <= prev.time_ms || curr.time_ms - prev.time_ms > max_gap) {
            reason = "motion_history_gap";
            return false;
        }
        if (prev.time_ms > cursor) { reason = "motion_history_gap"; return false; }
        const uint64_t segment_end = std::min(curr.time_ms, target_time_ms);
        if (segment_end <= cursor) continue;
        if (!prev.imu_valid || !curr.imu_valid) { reason = "imu_invalid"; return false; }
        if (!curr.encoder_valid) { reason = "encoder_invalid"; return false; }
        const float fraction = static_cast<float>(segment_end - cursor) /
                               static_cast<float>(curr.time_ms - prev.time_ms);
        const float mean_ticks = 0.5F * static_cast<float>(
            curr.left_encoder_delta + curr.right_encoder_delta);
        const float ds_m = mean_ticks * static_cast<float>(encoder_ticks_to_meter) * fraction;
        const float dt_s = static_cast<float>(segment_end - cursor) / 1000.0F;
        const float dyaw_rad = 0.5F * (prev.gyro_z + curr.gyro_z) * dt_s;
        IntegrateBodyStep(state.pose_delta, ds_m, dyaw_rad);
        cursor = segment_end;
    }
    if (cursor < target_time_ms) {
        reason = "motion_history_gap";
        return false;
    }
    state.cursor_time_ms = cursor;
    reason = "ok";
    return true;
}

port::BEVReferencePath CompactRemaining(const port::BEVReferencePath& input,
                                        std::size_t& remaining) {
    port::BEVReferencePath out{};
    out.mode = port::ReferenceMode::kMlObservedBoundary;
    remaining = 0U;
    for (const port::BEVPathSample& sample : input.sampled_path) {
        if (!sample.present) break;
        if (sample.point.forward_m <= 0.0F) continue;
        out.sampled_path[remaining++] = sample;
    }
    if (remaining == 0U) out.mode = port::ReferenceMode::kNone;
    return out;
}

}  // namespace

void MlInertialPathTracker::Start(const port::MlLockedManeuver& locked,
                                  port::MlInertialTrackerState& state) {
    state = {};
    if (!locked.valid || locked.lock_time_ms == 0U) return;
    state.active = true;
    state.cursor_time_ms = locked.lock_time_ms;
    state.pose_delta.valid = true;
}

port::MlInertialTrackerResult MlInertialPathTracker::Step(
    const port::MlLockedManeuver& locked,
    const port::MotionHistory& history,
    uint64_t target_time_ms,
    const port::MotionOdometryParameters& odometry,
    const port::MlManeuverParameters& params,
    port::MlInertialTrackerState& state) {
    port::MlInertialTrackerResult out{};
    if (!state.active || !locked.valid) return out;
    const char* integration_reason = "inactive";
    if (!IntegrateTo(history, target_time_ms, odometry.encoder_ticks_to_meter,
                     params.max_integration_gap_ms, state, integration_reason)) {
        out.reason = integration_reason;
        out.pose_delta = state.pose_delta;
        return out;
    }
    out.pose_delta = state.pose_delta;
    const port::BEVReferencePath transformed =
        TransformMarkerPathToCurrentVehicle(locked, state.pose_delta);
    out.path_sample_count = locked.marker_path.sample_count;
    out.reference_path = CompactRemaining(transformed, out.remaining_sample_count);
    const float axis_f = locked.marker_forward_axis_forward;
    const float axis_l = locked.marker_forward_axis_lateral;
    out.progress_m = state.pose_delta.forward_m * axis_f + state.pose_delta.lateral_m * axis_l;
    out.lateral_error_m = -state.pose_delta.forward_m * axis_l +
                           state.pose_delta.lateral_m * axis_f;
    out.heading_error_rad = state.pose_delta.yaw_rad - std::atan2(axis_l, axis_f);
    if (out.remaining_sample_count == 0U) { out.reason = "path_exhausted"; return out; }
    if (out.remaining_sample_count < static_cast<std::size_t>(params.min_boundary_samples)) {
        out.reason = "insufficient_remaining_samples";
        return out;
    }
    out.valid = true;
    out.reason = "active";
    return out;
}

void MlInertialPathTracker::Reset(port::MlInertialTrackerState& state) { state = {}; }

}  // namespace ls2k::vision::ml
