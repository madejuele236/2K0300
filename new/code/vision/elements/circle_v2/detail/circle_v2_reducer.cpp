#include "vision/elements/circle_v2/detail/circle_v2_internal.hpp"

#include <algorithm>

namespace ls2k::vision::detail {
namespace {

void EnterPhase(CircleV2Memory& memory, CirclePhase phase, CaptureStamp stamp) {
    memory.phase = phase;
    memory.clock.phase_enter_capture_time_ms = stamp.capture_time_ms;
}

void StartTurn(CircleV2Memory& memory, CaptureStamp stamp) {
    EnterPhase(memory, CirclePhase::kInnerTrace, stamp);
    memory.clock.turn_origin_capture_time_ms = stamp.capture_time_ms;
    memory.clock.max_directed_turn_angle_rad = 0.0F;
}

void EnterIdle(CircleV2Memory& memory) {
    memory = {};
}

bool IsAngularPhase(CirclePhase phase) {
    return phase == CirclePhase::kInnerTrace ||
           phase == CirclePhase::kNormalTrace ||
           phase == CirclePhase::kExitTrace;
}

CircleV2ReferenceRole RoleForPhase(CirclePhase phase) {
    if (phase == CirclePhase::kInnerTrace) {
        return CircleV2ReferenceRole::kInnerTrace;
    }
    if (phase == CirclePhase::kExitTrace || phase == CirclePhase::kCalmTrace) {
        return CircleV2ReferenceRole::kExitTrace;
    }
    return CircleV2ReferenceRole::kNone;
}

uint64_t PhaseElapsedMs(const CircleV2Memory& memory, CaptureStamp stamp) {
    if (stamp.capture_time_ms < memory.clock.phase_enter_capture_time_ms) {
        return 0U;
    }
    return stamp.capture_time_ms - memory.clock.phase_enter_capture_time_ms;
}

}  // namespace

CircleV2Decision ReduceCircleV2(const CircleV2Memory& prior,
                                const CircleV2Events& events,
                                CaptureStamp stamp,
                                const CircleV2Params& params) {
    CircleV2Memory current = prior;
    CircleV2TelemetryReason reason = CircleV2TelemetryReason::kNone;
    if (IsAngularPhase(prior.phase) && events.motion_arc_available) {
        current.clock.max_directed_turn_angle_rad =
            std::max(current.clock.max_directed_turn_angle_rad,
                     events.directed_turn_angle_rad);
    }
    const float progress_angle = current.clock.max_directed_turn_angle_rad;

    switch (prior.phase) {
        case CirclePhase::kIdle:
            if (events.detected_dir != CircleDir::kNone) {
                current.dir = events.detected_dir;
                EnterPhase(current, CirclePhase::kApproach, stamp);
                reason = events.detected_dir == CircleDir::kLeft
                             ? CircleV2TelemetryReason::kPhase1CueLeft
                             : CircleV2TelemetryReason::kPhase1CueRight;
            }
            break;
        case CirclePhase::kApproach:
            if (events.entry_gate_reached) {
                StartTurn(current, stamp);
                reason = CircleV2TelemetryReason::kEntryGateReached;
            }
            break;
        case CirclePhase::kInnerTrace:
        case CirclePhase::kNormalTrace:
        case CirclePhase::kExitTrace:
            if (events.inner_trace_stalled) {
                EnterIdle(current);
                reason = CircleV2TelemetryReason::kInnerTraceYawStalled;
            } else if (prior.phase == CirclePhase::kExitTrace &&
                       events.observed_outer_boundary) {
                EnterPhase(current, CirclePhase::kCalmTrace, stamp);
                reason = CircleV2TelemetryReason::kObservedOuterBoundary;
            } else if (progress_angle >= params.calm_fallback_yaw_rad) {
                EnterPhase(current, CirclePhase::kCalmTrace, stamp);
                reason = CircleV2TelemetryReason::kFallbackYawReached;
            } else if (progress_angle >= params.exit_trace_start_yaw_rad) {
                if (prior.phase != CirclePhase::kExitTrace) {
                    EnterPhase(current, CirclePhase::kExitTrace, stamp);
                    reason = CircleV2TelemetryReason::kExitTraceStarted;
                }
            } else if (progress_angle >= params.normal_trace_start_yaw_rad) {
                if (prior.phase != CirclePhase::kNormalTrace) {
                    EnterPhase(current, CirclePhase::kNormalTrace, stamp);
                    reason = CircleV2TelemetryReason::kNormalTraceStarted;
                }
            }
            break;
        case CirclePhase::kCalmTrace:
        case CirclePhase::kCooldown:
            break;
    }

    CircleV2Decision decision{};
    decision.frame_phase = current.phase;
    decision.reference.dir = current.dir;
    decision.reference.role = RoleForPhase(current.phase);
    decision.reason = reason;
    decision.progress_angle_rad = current.clock.max_directed_turn_angle_rad;

    CircleV2Memory next = current;
    if (current.phase == CirclePhase::kCalmTrace &&
        PhaseElapsedMs(current, stamp) >=
            static_cast<uint64_t>(std::max(1, params.calm_trace_ms))) {
        EnterPhase(next, CirclePhase::kCooldown, stamp);
        if (decision.reason == CircleV2TelemetryReason::kNone) {
            decision.reason = CircleV2TelemetryReason::kCalmTraceComplete;
        }
    } else if (current.phase == CirclePhase::kCooldown &&
               PhaseElapsedMs(current, stamp) >=
                   static_cast<uint64_t>(std::max(0, params.cooldown_ms))) {
        EnterIdle(next);
        if (decision.reason == CircleV2TelemetryReason::kNone) {
            decision.reason = CircleV2TelemetryReason::kCooldownComplete;
        }
    }
    decision.next_memory = next;
    return decision;
}

CircleV2Telemetry BuildCircleV2Telemetry(const CircleV2Decision& decision,
                                         const CircleV2Events& events,
                                         const CircleV2Geometry& geometry,
                                         const CircleSideExpansionObservation& expansion) {
    CircleV2Telemetry telemetry{};
    telemetry.frame_phase = decision.frame_phase;
    telemetry.next_phase = decision.next_memory.phase;
    telemetry.dir = decision.reference.dir;
    telemetry.reference_role = decision.reference.role;
    telemetry.reason = decision.reason;
    telemetry.motion_arc_available = events.motion_arc_available;
    telemetry.geometry_available = geometry.available;
    telemetry.geometry_source = geometry.source;
    telemetry.inner_trace_elapsed_ms = events.inner_trace_elapsed_ms;
    telemetry.directed_turn_angle_rad = decision.progress_angle_rad;
    telemetry.openings = expansion.openings;
    if ((decision.reference.role == CircleV2ReferenceRole::kInnerTrace ||
         decision.reference.role == CircleV2ReferenceRole::kExitTrace) &&
        !geometry.available && telemetry.reason == CircleV2TelemetryReason::kNone) {
        telemetry.reason = CircleV2TelemetryReason::kGeometryUnavailable;
    }
    return telemetry;
}

}  // namespace ls2k::vision::detail
