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

inline const char* CircleDirToken(CircleDir dir) {
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

enum class CirclePhase {
    kIdle,
    kApproach,
    kInnerTrace,
    kNormalTrace,
    kExitTrace,
    kCalmTrace,
    kCooldown,
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
    kNormalTraceStarted,
    kExitTraceStarted,
    kObservedOuterBoundary,
    kFallbackYawReached,
    kCalmTraceComplete,
    kCooldownComplete,
    kInnerTraceYawStalled,
    kGeometryUnavailable,
};

struct CircleV2StageClock {
    uint64_t phase_enter_capture_time_ms = 0;
    uint64_t turn_origin_capture_time_ms = 0;
    float max_directed_turn_angle_rad = 0.0F;
};

struct CircleV2Memory {
    CirclePhase phase = CirclePhase::kIdle;
    CircleDir dir = CircleDir::kNone;
    CircleV2StageClock clock{};
};

struct CircleV2Params {
    float normal_trace_start_yaw_rad = 1.57079632679F;
    float exit_trace_start_yaw_rad = 4.71238898038F;
    float calm_fallback_yaw_rad = 5.93411945678F;
    int calm_trace_ms = 1000;
    int cooldown_ms = 3000;
    int inner_trace_stall_timeout_ms = 4000;
    float inner_trace_stall_yaw_min_rad = 0.28797932658F;
    float inner_trace_path_offset_m = 0.0F;
    float opposite_straight_confidence_min = 0.70F;
    float max_adjacent_distance_m = 0.195660427F;
    float nominal_road_width_m = 0.40F;
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

enum class CircleV2GeometrySource {
    kNone,
    kObservedBoundary,
    kFixedExitRay,
};

inline const char* CircleV2GeometrySourceToken(CircleV2GeometrySource source) {
    switch (source) {
        case CircleV2GeometrySource::kNone:
            return "none";
        case CircleV2GeometrySource::kObservedBoundary:
            return "observed_boundary";
        case CircleV2GeometrySource::kFixedExitRay:
            return "fixed_exit_ray";
    }
    return "none";
}

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
    float begin_forward_m = 0.0F;
    float end_forward_m = 0.0F;
    float effective_lateral_m = 0.0F;
    CircleOpeningSource source = CircleOpeningSource::kNone;
    float outward_distance_m = 0.0F;
    float minimum_white_width_m = 0.0F;
    bool origin_connected = false;
};

struct CircleOpeningPairObservation {
    CircleOpeningObservation left{};
    CircleOpeningObservation right{};
};

struct CircleEntryCueObservation {
    CircleDir detected_dir = CircleDir::kNone;
    bool bilateral_overlap = false;
    bool selected = false;
    float selected_begin_forward_m = 0.0F;
    float selected_end_forward_m = 0.0F;
    bool opposite_observable = false;
    bool opposite_straight = false;
    float opposite_straight_confidence = 0.0F;
};

struct CircleV2Telemetry {
    CirclePhase frame_phase = CirclePhase::kIdle;
    CirclePhase next_phase = CirclePhase::kIdle;
    CircleDir dir = CircleDir::kNone;
    CircleV2ReferenceRole reference_role = CircleV2ReferenceRole::kNone;
    CircleV2TelemetryReason reason = CircleV2TelemetryReason::kNone;
    bool motion_arc_available = false;
    bool geometry_available = false;
    CircleV2GeometrySource geometry_source = CircleV2GeometrySource::kNone;
    uint64_t inner_trace_elapsed_ms = 0;
    float directed_turn_angle_rad = 0.0F;
    CircleEntryCueObservation entry_cue{};
    CircleOpeningPairObservation openings{};
};

}  // namespace ls2k::port

#endif  // LS2K_PORT_CIRCLE_V2_TYPES_HPP
