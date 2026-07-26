#ifndef LS2K_VISION_ML_SCENE_HPP
#define LS2K_VISION_ML_SCENE_HPP

#include "port/camera_frame_types.hpp"
#include "port/ml_types.hpp"
#include "port/motion_history_types.hpp"
#include "port/runtime_parameter_types.hpp"
#include "port/visual_reference_orchestration_types.hpp"
#include "vision/bev/bev_reference_path_builder.hpp"
#include "vision/bev/bev_projector.hpp"
#include "vision/ml/red_rectangle_detector.hpp"
#include "vision/ml/selected_ml_classifier.hpp"

namespace ls2k::vision::ml {

struct MlSceneInput {
    const port::CameraPixelFrameView* frame = nullptr;
    const BEVProjector* projector = nullptr;
    const BEVRoadPathFacts* road_path_facts = nullptr;
    const port::MotionHistory* motion_history = nullptr;
    MlRedRectangleProjectionLut* rectangle_projection_lut = nullptr;
    MlClassifier* classifier = nullptr;
    uint64_t capture_time_ms = 0;
};

struct MlSceneResult {
    port::MlSceneMemory next_memory{};
    port::MlTelemetrySnapshot telemetry{};
    port::VisualReferenceCandidate candidate{};
    bool suppress_other_scene_candidates = false;
};

/// Scene-owned consecutive-action confirmation. Acceptance thresholds and
/// raw-class mapping are evaluated before this boundary.
bool StepMlConfirmation(port::MlAction action,
                        bool requested_boundary_available,
                        int confirm_frames,
                        port::MlConfirmationState& state);

MlSceneResult StepMlScene(const MlSceneInput& input,
                          const port::RuntimeParameters& params,
                          const port::MlSceneMemory& prior_memory);
void ResetMlSceneMemory(port::MlSceneMemory& memory);

}  // namespace ls2k::vision::ml

#endif
