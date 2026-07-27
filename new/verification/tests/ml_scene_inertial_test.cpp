#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "vision/ml/ml_path_generator.hpp"
#include "vision/ml/ml_scene.hpp"

namespace {

void Expect(bool value, const char* message) {
    if (!value) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

void ExpectNear(float actual, float expected, const char* message) {
    if (std::fabs(actual - expected) > 1.0e-5F) {
        std::cerr << "FAIL: " << message << " actual=" << actual
                  << " expected=" << expected << '\n';
        std::exit(1);
    }
}

ls2k::port::RuntimeParameters MakeParams() {
    ls2k::port::RuntimeParameters params{};
    params.ml.enabled = true;
    params.ml.maneuver.enabled = true;
    params.ml.v9.confirm_frames = 1;
    params.motion_odometry.encoder_ticks_to_meter = 0.01;
    params.ml.maneuver.min_boundary_samples = 3;
    params.ml.maneuver.path_outward_offset_m = 0.1;
    params.ml.maneuver.exit_forward_m = 0.5;
    params.ml.maneuver.max_duration_ms = 1000;
    params.ml.maneuver.max_integration_gap_ms = 30;
    params.ml.maneuver.cooldown_ms = 200;
    return params;
}

ls2k::vision::BEVRoadPathFacts MakeFacts(float left_lateral = -0.2F,
                                         float right_lateral = 0.2F,
                                         std::size_t count = 4U) {
    ls2k::vision::BEVRoadPathFacts facts{};
    for (std::size_t index = 0U; index < count; ++index) {
        const float forward = 0.2F * static_cast<float>(index + 1U);
        facts.actual_left_boundary[index] = {true, {forward, left_lateral}, 1.0F};
        facts.actual_right_boundary[index] = {true, {forward, right_lateral}, 1.0F};
    }
    return facts;
}

ls2k::vision::ml::MlObservation MakeObservation(ls2k::port::MlAction action) {
    ls2k::vision::ml::MlObservation observation{};
    observation.accepted = true;
    observation.reason = "accepted";
    observation.mapped_action = action;
    observation.classification.backend = ls2k::port::MlClassifierBackend::kV9Hamming;
    return observation;
}

ls2k::port::MotionHistory MakeHistory(bool encoder_valid = true) {
    ls2k::port::MotionHistory history{};
    history.Push({100U, true, 0.0F, true, 0, 0});
    history.Push({120U, true, 0.0F, encoder_valid, 8, 12});
    history.Push({140U, true, 0.0F, encoder_valid, 8, 12});
    return history;
}

ls2k::port::MlSceneMemory MakeActive(ls2k::port::MlAction action =
                                         ls2k::port::MlAction::kRight) {
    ls2k::port::MlSceneMemory memory{};
    memory.phase = ls2k::port::MlScenePhase::kActive;
    memory.locked_action = action;
    memory.maneuver_start_ms = 100U;
    memory.distance_cursor_ms = 100U;
    return memory;
}

void TestBoundaryOffsetDirectionAndSource() {
    const auto facts = MakeFacts();
    const auto left = ls2k::vision::ml::BuildMlBoundaryOffsetPath(
        ls2k::port::MlAction::kLeft, facts, 3U, 0.1F);
    const auto right = ls2k::vision::ml::BuildMlBoundaryOffsetPath(
        ls2k::port::MlAction::kRight, facts, 3U, 0.1F);
    Expect(left.mode == ls2k::port::ReferenceMode::kMlBoundaryOffset &&
               right.mode == ls2k::port::ReferenceMode::kMlBoundaryOffset,
           "ML boundary offset mode missing");
    ExpectNear(left.sampled_path[0].point.lateral_m, -0.3F,
               "left boundary must offset toward negative lateral");
    ExpectNear(right.sampled_path[0].point.lateral_m, 0.3F,
               "right boundary must offset toward positive lateral");
    Expect(left.sampled_path[0].source ==
               ls2k::port::BEVPathPointSource::kMlBoundaryOffset,
           "offset path must publish its own point source");
}

void TestLockPublishesOffsetPathWithoutConnectivityGate() {
    const auto params = MakeParams();
    const auto facts = MakeFacts();
    const auto observation = MakeObservation(ls2k::port::MlAction::kRight);
    ls2k::vision::ml::MlManeuverInput input{};
    input.observation = &observation;
    input.road_path_facts = &facts;
    input.capture_time_ms = 100U;
    const auto result = ls2k::vision::ml::StepMlManeuver(input, params, {});
    Expect(result.next_memory.phase == ls2k::port::MlScenePhase::kActive &&
               result.next_memory.locked_action == ls2k::port::MlAction::kRight,
           "confirmed right action did not lock");
    Expect(result.candidate.present && result.telemetry.active,
           "lock frame did not publish active ML candidate");
    ExpectNear(result.candidate.reference_path.sampled_path[0].point.lateral_m, 0.3F,
               "lock frame did not publish configured outward offset");
}

void TestActiveRebuildsFromCurrentBoundary() {
    const auto params = MakeParams();
    const auto current_facts = MakeFacts(-0.4F, 0.35F);
    const auto history = MakeHistory();
    auto memory = MakeActive();
    ls2k::vision::ml::MlManeuverInput input{};
    input.road_path_facts = &current_facts;
    input.motion_history = &history;
    input.capture_time_ms = 120U;
    const auto result = ls2k::vision::ml::StepMlManeuver(input, params, memory);
    Expect(result.next_memory.phase == ls2k::port::MlScenePhase::kActive &&
               result.candidate.present,
           "valid current boundary did not keep active candidate");
    ExpectNear(result.candidate.reference_path.sampled_path[0].point.lateral_m, 0.45F,
               "active path was frozen instead of using current boundary");
    ExpectNear(result.next_memory.traveled_forward_m, 0.1F,
               "scalar encoder mean was not integrated");
}

void TestMissingCurrentPathHoldsOwnership() {
    const auto params = MakeParams();
    const auto insufficient = MakeFacts(-0.2F, 0.2F, 2U);
    const auto invalid_history = MakeHistory(false);
    ls2k::vision::ml::MlManeuverInput input{};
    input.road_path_facts = &insufficient;
    input.motion_history = &invalid_history;
    input.capture_time_ms = 120U;
    const auto result = ls2k::vision::ml::StepMlManeuver(input, params, MakeActive());
    Expect(result.next_memory.phase == ls2k::port::MlScenePhase::kActive &&
               result.telemetry.active && !result.candidate.present,
           "single-frame boundary loss must preserve ML ownership without a candidate");
    Expect(std::string(result.telemetry.reason) == "active_path_unavailable" &&
               !result.telemetry.odometry_valid &&
               std::string(result.telemetry.odometry_reason) == "encoder_invalid",
           "boundary and odometry facts were not independently observable");
}

void TestDistanceCompletion() {
    auto params = MakeParams();
    params.ml.maneuver.exit_forward_m = 0.1;
    const auto facts = MakeFacts();
    const auto history = MakeHistory();
    ls2k::vision::ml::MlManeuverInput input{};
    input.road_path_facts = &facts;
    input.motion_history = &history;
    input.capture_time_ms = 120U;
    const auto result = ls2k::vision::ml::StepMlManeuver(input, params, MakeActive());
    Expect(result.next_memory.phase == ls2k::port::MlScenePhase::kCooldown &&
               !result.candidate.present &&
               std::string(result.telemetry.reason) == "distance_complete",
           "distance completion did not release ML ownership");
}

void TestTimeoutCompletesWithoutOdometry() {
    auto params = MakeParams();
    params.ml.maneuver.max_duration_ms = 50;
    ls2k::vision::ml::MlManeuverInput input{};
    input.capture_time_ms = 150U;
    const auto result = ls2k::vision::ml::StepMlManeuver(input, params, MakeActive());
    Expect(result.next_memory.phase == ls2k::port::MlScenePhase::kCooldown &&
               std::string(result.telemetry.reason) == "timeout",
           "timeout must remain an independent completion condition");
}

void TestCooldownRetainsCompletionReason() {
    auto prior = ls2k::port::MlSceneMemory{};
    prior.phase = ls2k::port::MlScenePhase::kCooldown;
    prior.cooldown_start_ms = 100U;
    prior.cooldown_reason = "distance_complete";
    ls2k::vision::ml::MlManeuverInput input{};
    input.capture_time_ms = 150U;
    const auto result = ls2k::vision::ml::StepMlManeuver(input, MakeParams(), prior);
    Expect(result.next_memory.phase == ls2k::port::MlScenePhase::kCooldown &&
               std::string(result.telemetry.reason) == "distance_complete",
           "cooldown did not retain the completion reason");
}

void TestConfirmationAndObserveOnly() {
    ls2k::port::MlConfirmationState state{};
    Expect(!ls2k::vision::ml::StepMlConfirmation(
               ls2k::port::MlAction::kLeft, true, 2, state),
           "first confirmation frame must not lock");
    Expect(ls2k::vision::ml::StepMlConfirmation(
               ls2k::port::MlAction::kLeft, true, 2, state),
           "second identical frame must lock");
    Expect(!ls2k::vision::ml::StepMlConfirmation(
               ls2k::port::MlAction::kLeft, false, 2, state) &&
               state.consecutive_frames == 0,
           "unavailable requested boundary must reset pre-lock confirmation");

    auto params = MakeParams();
    params.ml.maneuver.enabled = false;
    const auto observation = MakeObservation(ls2k::port::MlAction::kLeft);
    ls2k::vision::ml::MlManeuverInput input{};
    input.observation = &observation;
    const auto result = ls2k::vision::ml::StepMlManeuver(input, params, MakeActive());
    Expect(result.next_memory.phase == ls2k::port::MlScenePhase::kDisabled &&
               !result.candidate.present &&
               std::string(result.telemetry.reason) == "observe_only",
           "observe-only must not retain maneuver ownership");
}

}  // namespace

int main() {
    TestBoundaryOffsetDirectionAndSource();
    TestLockPublishesOffsetPathWithoutConnectivityGate();
    TestActiveRebuildsFromCurrentBoundary();
    TestMissingCurrentPathHoldsOwnership();
    TestDistanceCompletion();
    TestTimeoutCompletesWithoutOdometry();
    TestCooldownRetainsCompletionReason();
    TestConfirmationAndObserveOnly();
    std::cout << "ml scene distance/timeout tests passed\n";
}
