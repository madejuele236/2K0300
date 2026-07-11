#pragma once

#include "port/vision_live_views.hpp"
#include "vision_facts.hpp"

namespace primer::vision {

// Non-owning, process-lifetime views over the vision owner's live state.
// Members are intentionally const references: copying them into snapshots would
// change the original unsynchronised observation timing.  Mutation stays behind
// explicitly named vision commands below.
using VisionControlLiveView = primer::port::vision::ControlLiveView;
using VisionPresentationLiveView = primer::port::vision::PresentationLiveView;

struct RoundaboutYawState {
    const float &initial_yaw;
    float &corrected_yaw;
    float &yaw_error;
};

VisionControlLiveView ObserveVisionControlLiveView();
VisionPresentationLiveView ObserveVisionPresentationLiveView();
RoundaboutYawState AccessRoundaboutYawState();

int VisionDynamicForward();
void SetVisionDynamicForward(int value);

float VisionRoundaboutYaw();
void SetVisionRoundaboutYawCorrection(float corrected_yaw, float yaw_error);

}  // namespace primer::vision

int real_distance_to_row(float distance);
