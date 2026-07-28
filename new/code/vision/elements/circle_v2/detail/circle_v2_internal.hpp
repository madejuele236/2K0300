#ifndef LS2K_RUNTIME_DETAIL_STEERING_CIRCLE_V2_INTERNAL_HPP
#define LS2K_RUNTIME_DETAIL_STEERING_CIRCLE_V2_INTERNAL_HPP

#include <optional>

#include "vision/elements/circle_v2/circle_v2_scene.hpp"

namespace ls2k::vision::detail {

struct CircleV2Events {
    CircleDir detected_dir = CircleDir::kNone;
    bool entry_gate_reached = false;
    bool inner_trace_stalled = false;
    bool observed_outer_boundary = false;
    bool motion_arc_available = false;
    uint64_t inner_trace_elapsed_ms = 0;
    float directed_turn_angle_rad = 0.0F;
};

struct CircleSideExpansionObservation {
    CircleDir detected_dir = CircleDir::kNone;
    CircleOpeningPairObservation openings{};
};

struct CircleV2ReferenceContext {
    CircleDir dir = CircleDir::kNone;
    CircleV2ReferenceRole role = CircleV2ReferenceRole::kNone;
};

struct CircleV2Decision {
    CirclePhase frame_phase = CirclePhase::kIdle;
    CircleV2Memory next_memory{};
    CircleV2ReferenceContext reference{};
    CircleV2TelemetryReason reason = CircleV2TelemetryReason::kNone;
    float progress_angle_rad = 0.0F;
};

struct CircleV2Geometry {
    bool available = false;
    CircleV2GeometrySource source = CircleV2GeometrySource::kNone;
    port::BEVReferencePath edge_path{};
    float road_half_width_m = 0.0F;
    float reference_offset_m = 0.0F;
};

struct CircleV2GeometryObservation {
    CircleV2Geometry inner{};
    CircleV2Geometry outer{};
};

CircleSideExpansionObservation ObserveCircleSideExpansion(const SceneFrameView& frame,
                                                          const CircleV2Params& params);

CircleV2Events ObserveCircleV2Events(const SceneFrameView& frame,
                                     const CircleSideExpansionObservation& expansion,
                                     const CircleV2GeometryObservation& geometry,
                                     const CircleV2Memory& prior,
                                     const CircleV2Params& params);

CircleV2Decision ReduceCircleV2(const CircleV2Memory& prior,
                                const CircleV2Events& events,
                                CaptureStamp stamp,
                                const CircleV2Params& params);

CircleV2GeometryObservation ObserveCircleV2Geometry(const SceneFrameView& frame,
                                                    CircleDir dir,
                                                    const CircleV2Params& params);

const CircleV2Geometry& SelectCircleV2Geometry(
    const CircleV2GeometryObservation& observation,
    CircleV2ReferenceRole role);

std::optional<CircleV2ReferencePlan> ComposeCircleV2Reference(
    const CircleV2ReferenceContext& reference,
    const CircleV2Geometry& geometry,
    const CircleV2Params& params);

CircleV2Telemetry BuildCircleV2Telemetry(const CircleV2Decision& decision,
                                         const CircleV2Events& events,
                                         const CircleV2Geometry& geometry,
                                         const CircleSideExpansionObservation& expansion);

}  // namespace ls2k::vision::detail

#endif  // LS2K_RUNTIME_DETAIL_STEERING_CIRCLE_V2_INTERNAL_HPP
