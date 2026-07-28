#ifndef LS2K_RUNTIME_DETAIL_STEERING_CIRCLE_V2_INTERNAL_HPP
#define LS2K_RUNTIME_DETAIL_STEERING_CIRCLE_V2_INTERNAL_HPP

#include <cstddef>
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

enum class BoundarySide {
    kLeft,
    kRight,
};

struct ForwardInterval {
    std::size_t begin_row_index = 0U;
    std::size_t end_row_index = 0U;
    float begin_forward_m = 0.0F;
    float end_forward_m = 0.0F;
};

struct SideOpeningObservation {
    CircleOpeningObservation fact{};
    BoundarySide side = BoundarySide::kLeft;
    ForwardInterval opening_range{};
    ForwardInterval baseline_support_range{};
};

struct BoundaryStraightObservation {
    bool observable = false;
    bool straight = false;
    float confidence = 0.0F;
};

struct CircleEntryCueObservation {
    SideOpeningObservation left_opening{};
    SideOpeningObservation right_opening{};
    bool bilateral_overlap = false;
    CircleDir detected_dir = CircleDir::kNone;
    bool selected = false;
    SideOpeningObservation selected_opening{};
    BoundaryStraightObservation selected_opposite_boundary{};
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

CircleEntryCueObservation ObserveCircleEntryCue(const SceneFrameView& frame,
                                                const CircleV2Params& params);

CircleV2Events ObserveCircleV2Events(const SceneFrameView& frame,
                                     const CircleEntryCueObservation& entry_cue,
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
                                         const CircleEntryCueObservation& entry_cue);

}  // namespace ls2k::vision::detail

#endif  // LS2K_RUNTIME_DETAIL_STEERING_CIRCLE_V2_INTERNAL_HPP
