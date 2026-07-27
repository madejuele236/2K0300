#ifndef LS2K_PORT_ML_TYPES_HPP
#define LS2K_PORT_ML_TYPES_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include "port/bev_geometry_types.hpp"
#include "port/bev_reference_types.hpp"

namespace ls2k::port {

constexpr int kMlRoiSide = 32;
constexpr std::size_t kMlRoiPixelCount = kMlRoiSide * kMlRoiSide;
constexpr std::size_t kV9DescriptorBytes = 16;
constexpr int kV9DescriptorBits = 126;

struct MlOrientedRectangle {
    bool valid = false;
    std::uint64_t frame_id = 0;
    std::uint64_t capture_time_ms = 0;
    std::array<BEVPoint, 4> corners{};
    BEVPoint center{};
    float long_edge_m = 0.0F;
    float short_edge_m = 0.0F;
    float long_axis_forward = 0.0F;
    float long_axis_lateral = 1.0F;
    float long_edge_to_lateral_rad = 0.0F;
    float rectangularity = 0.0F;
    float red_fill_ratio = 0.0F;
    float quality = 0.0F;
    int component_cells = 0;
};

struct MlGrayRoi32 {
    bool valid = false;
    std::uint64_t frame_id = 0;
    float long_axis_forward = 0.0F;
    float long_axis_lateral = 1.0F;
    float forward_normal_forward = 1.0F;
    float forward_normal_lateral = 0.0F;
    const char* reason = "not_sampled";
    std::array<std::uint8_t, kMlRoiPixelCount> gray{};
};

struct V9Descriptor {
    bool valid = false;
    std::array<std::uint8_t, kV9DescriptorBytes> bytes{};
};

enum class MlAction : std::uint8_t { kStraight, kLeft, kRight, kUnmapped };

inline const char* MlActionToken(MlAction action) {
    switch (action) {
        case MlAction::kStraight:
            return "straight";
        case MlAction::kLeft:
            return "left";
        case MlAction::kRight:
            return "right";
        case MlAction::kUnmapped:
            return "unmapped";
    }
    return "unmapped";
}

struct V9ReplayResult {
    bool valid = false;
    int class_id = -1;
    int best_distance = kV9DescriptorBits + 1;
    int margin = 0;
    int prototype_index = -1;
};

enum class MlClassifierBackend {
    kV9Hamming,
    kTfliteInt8,
};

inline const char* MlClassifierBackendToken(MlClassifierBackend backend) {
    switch (backend) {
        case MlClassifierBackend::kV9Hamming: return "v9_hamming";
        case MlClassifierBackend::kTfliteInt8: return "tflite_int8";
    }
    return "unknown";
}

struct MlClassificationResult {
    bool valid = false;
    MlClassifierBackend backend = MlClassifierBackend::kV9Hamming;
    int class_id = -1;
    int margin = 0;
    bool distance_valid = false;
    int best_distance = 0;
    std::array<int, 3> class_scores{};
    const char* reason = "not_run";
};

struct MlTfliteInt8Feature {
    bool valid = false;
    std::array<std::int8_t, 4> values{};
};

struct MlClassifierOutput {
    MlClassificationResult classification{};
    V9Descriptor v9_descriptor{};
    V9ReplayResult v9_replay{};
    MlTfliteInt8Feature tflite_feature{};
};

struct V9AcceptanceState {
    int pending_class_id = -1;
    int consecutive_frames = 0;
};

enum class MlScenePhase : std::uint8_t { kDisabled, kIdle, kCandidate, kActive, kCooldown };

inline const char* MlScenePhaseToken(MlScenePhase phase) {
    switch (phase) {
        case MlScenePhase::kDisabled:
            return "disabled";
        case MlScenePhase::kIdle:
            return "idle";
        case MlScenePhase::kCandidate:
            return "candidate";
        case MlScenePhase::kActive:
            return "active";
        case MlScenePhase::kCooldown:
            return "cooldown";
    }
    return "idle";
}

struct MlConfirmationState {
    MlAction pending_action = MlAction::kUnmapped;
    int consecutive_frames = 0;
};

struct MlSceneMemory {
    MlScenePhase phase = MlScenePhase::kIdle;
    MlConfirmationState confirmation{};
    MlAction locked_action = MlAction::kUnmapped;
    uint64_t maneuver_start_ms = 0;
    uint64_t distance_cursor_ms = 0;
    double traveled_forward_m = 0.0;
    uint64_t cooldown_start_ms = 0;
    const char* cooldown_reason = "none";
};

/// Per-frame transport facts. The ROI payload is meaningful only when
/// roi.valid is true and is retained for later media/debug serialization.
struct MlTelemetrySnapshot {
    bool enabled = false;
    bool maneuver_enabled = false;
    bool takeover_selected = false;
    bool detector_valid = false;
    MlOrientedRectangle detector{};
    MlGrayRoi32 roi{};
    V9Descriptor descriptor{};
    V9ReplayResult replay{};
    MlClassificationResult classification{};
    MlAction mapped_action = MlAction::kUnmapped;
    MlAction locked_action = MlAction::kUnmapped;
    MlScenePhase phase = MlScenePhase::kIdle;
    const char* reason = "disabled";
    int confirm_count = 0;
    bool active = false;
    bool odometry_valid = false;
    const char* odometry_reason = "not_run";
    float traveled_forward_m = 0.0F;
    uint64_t elapsed_ms = 0;
    std::size_t path_sample_count = 0;
    const char* artifact_candidate_id = "unavailable";
    const char* descriptor_config_hash = "unavailable";
    const char* template_table_hash = "unavailable";
    const char* template_codes_sha256 = "unavailable";
    std::size_t artifact_prototype_count = 0;
    std::uint32_t detector_us = 0;
    std::uint32_t roi_us = 0;
    std::uint32_t descriptor_us = 0;
    std::uint32_t replay_us = 0;
    std::uint32_t classifier_us = 0;
    std::uint32_t total_us = 0;
};

struct V9AcceptanceResult {
    bool accepted = false;
    int class_id = -1;
    int consecutive_frames = 0;
};

struct V9ArtifactView {
    const std::uint8_t* template_codes = nullptr;
    const std::uint8_t* template_parent = nullptr;
    std::size_t prototype_count = 0;
    std::size_t byte_count = 0;
    int bit_count = 0;
    const char* candidate_id = nullptr;
    const char* descriptor_config_hash = nullptr;
    const char* template_table_hash = nullptr;
    const char* template_codes_sha256 = nullptr;
};

}  // namespace ls2k::port

#endif
