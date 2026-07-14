#include <cmath>
#include <cstdlib>
#include <iostream>

#include "vision/ml/ml_inertial_path_tracker.hpp"
#include "vision/ml/ml_path_generator.hpp"
#include "vision/ml/ml_reference_adapter.hpp"
#include "vision/ml/ml_scene.hpp"

namespace {

void Expect(bool value, const char* message) {
    if (!value) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

ls2k::port::MlLockedManeuver MakeLocked() {
    ls2k::port::MlOrientedRectangle rectangle{};
    rectangle.valid = true;
    rectangle.center = {0.0F, 0.0F};
    rectangle.long_axis_forward = 0.0F;
    rectangle.long_axis_lateral = -1.0F;  // short normal resolves to +vehicle-forward
    ls2k::port::BEVReferencePath boundary{};
    boundary.mode = ls2k::port::ReferenceMode::kMlObservedBoundary;
    for (std::size_t i = 0; i < 6U; ++i) {
        boundary.sampled_path[i].present = true;
        boundary.sampled_path[i].point = {0.2F + 0.2F * static_cast<float>(i), 0.1F};
        boundary.sampled_path[i].source =
            ls2k::port::BEVPathPointSource::kMlObservedBoundary;
    }
    return ls2k::vision::ml::LockObservedBoundaryToMarker(
        ls2k::port::MlAction::kLeft, 100U, rectangle, boundary);
}

ls2k::port::MotionHistory MakeHistory(bool encoder_valid = true) {
    ls2k::port::MotionHistory history{};
    history.Push({100U, true, 0.0F, encoder_valid, 0, 0});
    history.Push({120U, true, 0.0F, encoder_valid, 10, 10});
    history.Push({140U, true, 0.0F, encoder_valid, 10, 10});
    return history;
}

ls2k::port::RuntimeParameters MakeParams() {
    ls2k::port::RuntimeParameters params{};
    params.ml.enabled = true;
    params.motion_odometry.encoder_ticks_to_meter = 0.01;
    params.ml.maneuver.min_boundary_samples = 3;
    params.ml.maneuver.exit_forward_m = 0.15;
    params.ml.maneuver.exit_max_abs_lateral_error_m = 0.01;
    params.ml.maneuver.exit_max_abs_heading_error_rad = 0.01;
    params.ml.maneuver.max_duration_ms = 1000;
    params.ml.maneuver.max_integration_gap_ms = 30;
    params.ml.maneuver.cooldown_ms = 200;
    return params;
}

void TestMarkerLockAndTracking() {
    const auto locked = MakeLocked();
    Expect(locked.valid, "literal boundary did not lock");
    Expect(std::fabs(locked.marker_forward_axis_forward - 1.0F) < 1.0e-6F,
           "short-axis normal was not made vehicle-forward positive");
    auto state = ls2k::port::MlInertialTrackerState{};
    ls2k::vision::ml::MlInertialPathTracker::Start(locked, state);
    const auto tracked = ls2k::vision::ml::MlInertialPathTracker::Step(
        locked, MakeHistory(), 120U, MakeParams().motion_odometry,
        MakeParams().ml.maneuver, state);
    Expect(tracked.valid, "valid IMU/encoder interval rejected");
    Expect(std::fabs(tracked.pose_delta.forward_m - 0.1F) < 1.0e-5F,
           "encoder mean was not scaled into forward progress");
    Expect(tracked.reference_path.mode == ls2k::port::ReferenceMode::kMlObservedBoundary,
           "tracked path lost ML observed-boundary mode");
}

void TestObservedBoundaryGeneratorIsLiteralAndReplaceable() {
    ls2k::vision::BEVRoadPathFacts facts{};
    for (std::size_t index = 0; index < 3U; ++index) {
        facts.center[index].present = true;
        facts.center[index].point = {0.2F * static_cast<float>(index + 1U), 0.0F};
        facts.actual_left_boundary[index].present = true;
        facts.actual_left_boundary[index].point = {
            facts.center[index].point.forward_m, -0.17F - 0.01F * static_cast<float>(index)};
    }
    const ls2k::vision::ml::ObservedBoundaryMlPathGenerator generator{};
    const auto left = generator.Generate({ls2k::port::MlAction::kLeft, &facts, 3U});
    Expect(left.mode == ls2k::port::ReferenceMode::kMlObservedBoundary &&
               left.sampled_path[0].point.lateral_m == -0.17F,
           "v1 generator must preserve the literal actual-left coordinate");
    const auto right = generator.Generate({ls2k::port::MlAction::kRight, &facts, 3U});
    Expect(right.mode == ls2k::port::ReferenceMode::kNone,
           "v1 generator must not synthesize a missing requested side");
}

void TestInvalidEncoderExitsTracker() {
    const auto locked = MakeLocked();
    auto state = ls2k::port::MlInertialTrackerState{};
    ls2k::vision::ml::MlInertialPathTracker::Start(locked, state);
    const auto tracked = ls2k::vision::ml::MlInertialPathTracker::Step(
        locked, MakeHistory(false), 120U, MakeParams().motion_odometry,
        MakeParams().ml.maneuver, state);
    Expect(!tracked.valid && std::string(tracked.reason) == "encoder_invalid",
           "invalid encoder did not remain visible at tracker owner");
}

void TestActiveCompletionEmitsNoCandidate() {
    auto memory = ls2k::port::MlSceneMemory{};
    memory.phase = ls2k::port::MlScenePhase::kActive;
    memory.locked = MakeLocked();
    ls2k::vision::ml::MlInertialPathTracker::Start(memory.locked, memory.tracker);
    const auto history = MakeHistory();
    ls2k::vision::ml::MlSceneInput input{};
    input.motion_history = &history;
    input.capture_time_ms = 120U;
    auto first = ls2k::vision::ml::StepMlScene(input, MakeParams(), memory);
    Expect(first.candidate.present && first.suppress_other_scene_candidates,
           "active scene did not exclusively emit ML candidate");
    input.capture_time_ms = 140U;
    auto completed = ls2k::vision::ml::StepMlScene(input, MakeParams(), first.next_memory);
    Expect(completed.next_memory.phase == ls2k::port::MlScenePhase::kCooldown,
           "progress/error completion did not enter cooldown");
    Expect(!completed.candidate.present && completed.suppress_other_scene_candidates,
           "exit frame must release ML while limiting takeover to line/hold");
    Expect(std::string(completed.telemetry.reason) == "maneuver_complete",
           "completion reason mismatch");
}

void TestSceneOwnsMappedActionConfirmation() {
    ls2k::port::MlConfirmationState state{};
    Expect(!ls2k::vision::ml::StepMlConfirmation(
               ls2k::port::MlAction::kLeft, true, 2, state) &&
               state.consecutive_frames == 1,
           "first valid left action must enter confirmation");
    Expect(!ls2k::vision::ml::StepMlConfirmation(
               ls2k::port::MlAction::kStraight, true, 2, state) &&
               state.consecutive_frames == 0,
           "mapped straight action must clear non-straight confirmation");
    Expect(!ls2k::vision::ml::StepMlConfirmation(
               ls2k::port::MlAction::kLeft, true, 2, state),
           "left confirmation should restart after straight");
    Expect(!ls2k::vision::ml::StepMlConfirmation(
               ls2k::port::MlAction::kLeft, false, 2, state) &&
               state.consecutive_frames == 0,
           "missing literal requested boundary must clear confirmation");
    Expect(!ls2k::vision::ml::StepMlConfirmation(
               ls2k::port::MlAction::kLeft, true, 2, state),
           "left confirmation should restart after missing boundary");
    Expect(!ls2k::vision::ml::StepMlConfirmation(
               ls2k::port::MlAction::kRight, true, 2, state) &&
               state.pending_action == ls2k::port::MlAction::kRight &&
               state.consecutive_frames == 1,
           "mapped action change must restart the count");
    Expect(ls2k::vision::ml::StepMlConfirmation(
               ls2k::port::MlAction::kRight, true, 2, state),
           "same mapped right action must confirm on the configured frame");
}

void TestDisabledSceneDoesNoWork() {
    auto params = MakeParams();
    params.ml.enabled = false;
    ls2k::vision::ml::MlSceneInput input{};
    ls2k::port::MlSceneMemory prior{};
    prior.phase = ls2k::port::MlScenePhase::kActive;
    const auto result = ls2k::vision::ml::StepMlScene(input, params, prior);
    Expect(result.next_memory.phase == ls2k::port::MlScenePhase::kDisabled &&
               !result.candidate.present && !result.telemetry.detector_valid &&
               result.telemetry.total_us == 0U,
           "disabled ML must reset without scheduling detector or inference work");
}

}  // namespace

int main() {
    TestMarkerLockAndTracking();
    TestObservedBoundaryGeneratorIsLiteralAndReplaceable();
    TestInvalidEncoderExitsTracker();
    TestActiveCompletionEmitsNoCandidate();
    TestSceneOwnsMappedActionConfirmation();
    TestDisabledSceneDoesNoWork();
    std::cout << "ml scene/inertial tests passed\n";
}
