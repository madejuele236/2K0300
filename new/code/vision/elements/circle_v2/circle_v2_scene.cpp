#include "vision/elements/circle_v2/circle_v2_scene.hpp"

#include "vision/elements/circle_v2/detail/circle_v2_internal.hpp"

namespace ls2k::vision {

const char* ToString(CircleDir dir) {
    switch (dir) {
        case CircleDir::kNone:
            return "none";
        case CircleDir::kLeft:
            return "left";
        case CircleDir::kRight:
            return "right";
    }
    return "none";
}

const char* ToString(CircleOpeningSource source) {
    return port::CircleOpeningSourceToken(source);
}

const char* ToString(CirclePhase phase) {
    switch (phase) {
        case CirclePhase::kIdle:
            return "idle";
        case CirclePhase::kApproach:
            return "approach";
        case CirclePhase::kInnerTrace:
            return "inner_trace";
        case CirclePhase::kNormalTrace:
            return "normal_trace";
        case CirclePhase::kExitTrace:
            return "exit_trace";
        case CirclePhase::kCalmTrace:
            return "calm_trace";
        case CirclePhase::kCooldown:
            return "cooldown";
    }
    return "idle";
}

const char* ToString(CircleV2ReferenceRole role) {
    switch (role) {
        case CircleV2ReferenceRole::kNone:
            return "none";
        case CircleV2ReferenceRole::kInnerTrace:
            return "inner_trace";
        case CircleV2ReferenceRole::kExitTrace:
            return "exit_trace";
    }
    return "none";
}

const char* ToString(CircleV2TelemetryReason reason) {
    switch (reason) {
        case CircleV2TelemetryReason::kNone:
            return "none";
        case CircleV2TelemetryReason::kPhase1CueLeft:
            return "phase1_cue_left";
        case CircleV2TelemetryReason::kPhase1CueRight:
            return "phase1_cue_right";
        case CircleV2TelemetryReason::kEntryGateReached:
            return "entry_gate_reached";
        case CircleV2TelemetryReason::kNormalTraceStarted:
            return "normal_trace_started";
        case CircleV2TelemetryReason::kExitTraceStarted:
            return "exit_trace_started";
        case CircleV2TelemetryReason::kObservedOuterBoundary:
            return "observed_outer_boundary";
        case CircleV2TelemetryReason::kFallbackYawReached:
            return "fallback_yaw_reached";
        case CircleV2TelemetryReason::kCalmTraceComplete:
            return "calm_trace_complete";
        case CircleV2TelemetryReason::kCooldownComplete:
            return "cooldown_complete";
        case CircleV2TelemetryReason::kInnerTraceYawStalled:
            return "inner_trace_yaw_stalled";
        case CircleV2TelemetryReason::kGeometryUnavailable:
            return "geometry_unavailable";
    }
    return "none";
}

void ResetCircleV2Memory(CircleV2Memory& memory) {
    memory = {};
}

CircleV2StepResult CircleV2Scene::Step(const SceneFrameView& frame,
                                       const CircleV2Memory& prior,
                                       const CircleV2Params& params) const {
    const detail::CircleSideExpansionObservation expansion =
        detail::ObserveCircleSideExpansion(frame, params);
    const detail::CircleV2GeometryObservation geometry_observation =
        detail::ObserveCircleV2Geometry(frame, prior.dir, params);
    const detail::CircleV2Events events =
        detail::ObserveCircleV2Events(frame,
                                      expansion,
                                      geometry_observation,
                                      prior,
                                      params);
    const detail::CircleV2Decision decision =
        detail::ReduceCircleV2(prior, events, frame.stamp, params);
    const detail::CircleV2Geometry& geometry =
        detail::SelectCircleV2Geometry(geometry_observation, decision.reference.role);

    CircleV2StepResult result{};
    result.next_memory = decision.next_memory;
    result.telemetry = detail::BuildCircleV2Telemetry(decision, events, geometry, expansion);
    result.reference_plan =
        detail::ComposeCircleV2Reference(decision.reference, geometry, params);
    return result;
}

}  // namespace ls2k::vision
