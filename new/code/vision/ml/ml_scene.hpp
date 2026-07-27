#ifndef LS2K_VISION_ML_SCENE_HPP
#define LS2K_VISION_ML_SCENE_HPP

#include "port/ml_types.hpp"
#include "port/motion_history_types.hpp"
#include "port/runtime_parameter_types.hpp"
#include "port/visual_reference_orchestration_types.hpp"
#include "vision/bev/bev_reference_path_builder.hpp"
#include "vision/ml/ml_observer.hpp"

namespace ls2k::vision::ml {

struct MlManeuverInput {
    const MlObservation* observation = nullptr;
    const BEVRoadPathFacts* road_path_facts = nullptr;
    const port::MotionHistory* motion_history = nullptr;
    uint64_t capture_time_ms = 0;
};

struct MlManeuverResult {
    port::MlSceneMemory next_memory{};
    port::MlTelemetrySnapshot telemetry{};
    port::VisualReferenceCandidate candidate{};
};

/// Scene-owned consecutive-action confirmation. Acceptance thresholds and
/// raw-class mapping are evaluated before this boundary.
bool StepMlConfirmation(port::MlAction action,
                        bool requested_boundary_available,
                        int confirm_frames,
                        port::MlConfirmationState& state);

MlManeuverResult StepMlManeuver(const MlManeuverInput& input,
                                const port::RuntimeParameters& params,
                                const port::MlSceneMemory& prior_memory);
void ResetMlSceneMemory(port::MlSceneMemory& memory);

}  // namespace ls2k::vision::ml

#endif
