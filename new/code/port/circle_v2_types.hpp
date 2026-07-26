#ifndef LS2K_PORT_CIRCLE_V2_TYPES_HPP
#define LS2K_PORT_CIRCLE_V2_TYPES_HPP

#include <cstdint>

#include "port/bev_reference_types.hpp"

namespace ls2k::port {

enum class CircleDir {
    kNone,
    kLeft,
    kRight,
};

enum class CirclePhase {
    kIdle,
    kApproach,
    kInnerTrace,
    kExitTrace,
};

enum class CircleV2ReferenceRole {
    kNone,
    kInnerTrace,
    kExitTrace,
};

enum class CircleV2TelemetryReason {
    kNone,
    kPhase1CueLeft,
    kPhase1CueRight,
    kEntryGateReached,
    kExitGateReached,
    kInnerTraceYawStalled,
    kExitHoldReleased,
    kGeometryUnavailable,
};

struct CircleV2StageClock {
    uint64_t enter_capture_time_ms = 0;
    int phase_frame_index = 0;
    float max_directed_turn_angle_rad = 0.0F;
};

struct CircleV2Memory {
    CirclePhase phase = CirclePhase::kIdle;
    CircleDir dir = CircleDir::kNone;
    CircleV2StageClock clock{};
};

struct CircleV2Params {
    float exit_yaw_threshold_rad = 5.75958653158F;
    int exit_hold_frames = 60;
    int inner_trace_stall_timeout_ms = 4000;
    float inner_trace_stall_yaw_min_rad = 0.28797932658F;
    float inner_trace_path_offset_m = 0.0F;
    float opposite_straight_confidence_min = 0.70F;
    float max_adjacent_distance_m = 0.195660427F;
    float min_sampleable_width_m = 0.35F;
    float opening_forward_min_m = 0.05F;
    float opening_forward_max_m = 1.50F;
    float opening_distance_min_m = 0.055F;
    float opening_confirm_forward_span_m = 0.10F;
    float entry_forward_min_m = 0.10F;
    float entry_forward_max_m = 0.50F;
    float inner_geometry_forward_min_m = 0.05F;
    float inner_geometry_forward_max_m = 0.50F;
    float exit_geometry_forward_min_m = 0.05F;
    float exit_geometry_forward_max_m = 0.50F;
    float exit_straight_max_lateral_span_m = 0.13F;
};

struct CircleV2ReferencePlan {
    CircleDir dir = CircleDir::kNone;
    CircleV2ReferenceRole role = CircleV2ReferenceRole::kNone;
    BEVReferencePath reference_path{};
};

enum class CircleOpeningSource {
    kNone,
    kObservedBoundary,
    kFovEdgeLowerBound,
};

inline const char* CircleOpeningSourceToken(CircleOpeningSource source) {
    switch (source) {
        case CircleOpeningSource::kNone:
            return "none";
        case CircleOpeningSource::kObservedBoundary:
            return "observed_boundary";
        case CircleOpeningSource::kFovEdgeLowerBound:
            return "fov_edge_lower_bound";
    }
    return "none";
}

struct CircleOpeningObservation {
    bool available = false;
    float frontier_forward_m = 0.0F;
    float effective_lateral_m = 0.0F;
    CircleOpeningSource source = CircleOpeningSource::kNone;
    float outward_distance_m = 0.0F;
    float confirmed_forward_span_m = 0.0F;
    bool origin_connected = false;
    bool opposite_straight = false;
};

struct CircleOpeningPairObservation {
    CircleOpeningObservation left{};
    CircleOpeningObservation right{};
};

struct CircleV2Telemetry {
    CirclePhase frame_phase = CirclePhase::kIdle;
    CirclePhase next_phase = CirclePhase::kIdle;
    CircleDir dir = CircleDir::kNone;
    CircleV2ReferenceRole reference_role = CircleV2ReferenceRole::kNone;
    CircleV2TelemetryReason reason = CircleV2TelemetryReason::kNone;
    bool motion_arc_available = false;
    bool geometry_available = false;
    uint64_t inner_trace_elapsed_ms = 0;
    float directed_turn_angle_rad = 0.0F;
    CircleOpeningPairObservation openings{};
};

}  // namespace ls2k::port

#endif  // LS2K_PORT_CIRCLE_V2_TYPES_HPP
