#ifndef LS2K_VISION_ML_OBSERVER_HPP
#define LS2K_VISION_ML_OBSERVER_HPP

#include <cstdint>

#include "port/camera_frame_types.hpp"
#include "port/ml_types.hpp"
#include "port/runtime_parameter_types.hpp"
#include "vision/bev/bev_projector.hpp"
#include "vision/ml/red_rectangle_detector.hpp"
#include "vision/ml/selected_ml_classifier.hpp"

namespace ls2k::vision::ml {

struct MlObserverInput {
    const port::CameraPixelFrameView* frame = nullptr;
    const BEVProjector* projector = nullptr;
    MlRedRectangleProjectionLut* rectangle_projection_lut = nullptr;
    MlClassifier* classifier = nullptr;
};

/// Per-frame recognition fact. It owns no maneuver state and cannot produce a path.
struct MlObservation {
    bool enabled = false;
    bool accepted = false;
    const char* reason = "disabled";
    bool detector_valid = false;
    port::MlOrientedRectangle detector{};
    port::MlGrayRoi32 roi{};
    port::V9Descriptor descriptor{};
    port::V9ReplayResult replay{};
    port::MlClassificationResult classification{};
    port::MlAction mapped_action = port::MlAction::kUnmapped;
    const char* artifact_candidate_id = "unavailable";
    const char* artifact_sha256 = "unavailable";
    std::size_t artifact_item_count = 0U;
    std::uint32_t detector_us = 0U;
    std::uint32_t roi_us = 0U;
    std::uint32_t classifier_us = 0U;
    std::uint32_t total_us = 0U;
};

MlObservation RunMlObserver(const MlObserverInput& input,
                            const port::MlParameters& params);

}  // namespace ls2k::vision::ml

#endif
