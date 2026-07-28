#include "vision/elements/circle_v2/detail/circle_v2_internal.hpp"

#include <algorithm>
#include <cmath>

namespace ls2k::vision::detail {
namespace {

const CircleOpeningObservation* OpeningForDir(
    const CircleSideExpansionObservation& expansion,
    CircleDir dir) {
    if (dir == CircleDir::kLeft) {
        return &expansion.openings.left;
    }
    if (dir == CircleDir::kRight) {
        return &expansion.openings.right;
    }
    return nullptr;
}

bool EntryGateReached(const CircleSideExpansionObservation& expansion,
                      CircleDir dir,
                      const CircleV2Params& params) {
    const CircleOpeningObservation* opening = OpeningForDir(expansion, dir);
    return opening != nullptr && opening->available &&
           opening->frontier_forward_m >= params.entry_forward_min_m &&
           opening->frontier_forward_m <= params.entry_forward_max_m;
}

float CircleTurnSign(CircleDir dir) {
    if (dir == CircleDir::kLeft) {
        return -1.0F;
    }
    if (dir == CircleDir::kRight) {
        return 1.0F;
    }
    return 0.0F;
}

uint64_t InnerTraceElapsedMs(const CircleV2Memory& prior, CaptureStamp stamp) {
    if (stamp.capture_time_ms < prior.clock.turn_origin_capture_time_ms) {
        return 0;
    }
    return stamp.capture_time_ms - prior.clock.turn_origin_capture_time_ms;
}

bool StallTimeoutReached(uint64_t elapsed_ms, const CircleV2Params& params) {
    const uint64_t timeout_ms =
        static_cast<uint64_t>(std::max(1, params.inner_trace_stall_timeout_ms));
    return elapsed_ms >= timeout_ms;
}

}  // namespace

CircleV2Events ObserveCircleV2Events(const SceneFrameView& frame,
                                     const CircleSideExpansionObservation& expansion,
                                     const CircleV2GeometryObservation& geometry,
                                     const CircleV2Memory& prior,
                                     const CircleV2Params& params) {
    CircleV2Events events{};
    switch (prior.phase) {
        case CirclePhase::kIdle:
            events.detected_dir = expansion.detected_dir;
            break;
        case CirclePhase::kApproach:
            events.entry_gate_reached = EntryGateReached(expansion, prior.dir, params);
            break;
        case CirclePhase::kInnerTrace:
        case CirclePhase::kNormalTrace:
        case CirclePhase::kExitTrace: {
            events.inner_trace_elapsed_ms = InnerTraceElapsedMs(prior, frame.stamp);
            events.observed_outer_boundary =
                prior.phase == CirclePhase::kExitTrace &&
                geometry.outer.available &&
                geometry.outer.source == CircleV2GeometrySource::kObservedBoundary;
            float yaw_delta = 0.0F;
            if (!frame.motion_arc.TryYawDeltaRad(prior.clock.turn_origin_capture_time_ms,
                                                 frame.stamp.capture_time_ms,
                                                 yaw_delta)) {
                break;
            }
            events.motion_arc_available = true;
            const float directed_turn_angle = CircleTurnSign(prior.dir) * yaw_delta;
            events.directed_turn_angle_rad = directed_turn_angle;
            const float progress_angle =
                std::max(prior.clock.max_directed_turn_angle_rad, directed_turn_angle);
            events.inner_trace_stalled =
                prior.phase == CirclePhase::kInnerTrace &&
                StallTimeoutReached(events.inner_trace_elapsed_ms, params) &&
                progress_angle < params.inner_trace_stall_yaw_min_rad;
            break;
        }
        case CirclePhase::kCalmTrace:
        case CirclePhase::kCooldown:
            break;
    }
    return events;
}

}  // namespace ls2k::vision::detail
