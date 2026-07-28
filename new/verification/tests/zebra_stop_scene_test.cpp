#include <cstdlib>
#include <iostream>
#include <string>

#include "vision/elements/zebra_stop_scene.hpp"

namespace {

void Expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

ls2k::vision::ZebraStopSceneResult Step(
    bool active,
    bool detected,
    uint64_t now_ms,
    const ls2k::port::BEVElementParameters& params,
    const ls2k::port::ZebraStopMemory& memory) {
    return ls2k::vision::StepZebraStopScene(
        {active, detected, now_ms}, params, memory);
}

}  // namespace

int main() {
    using ls2k::port::ZebraStopPhase;

    ls2k::port::BEVElementParameters params{};
    params.zebra_reentry_arm_absence_ms = 500;
    params.zebra_controlled_stop_delay_ms = 300;

    auto result = Step(false, true, 1000, params, {});
    Expect(result.next_memory.phase == ZebraStopPhase::kWaitingForFirstDetection,
           "inactive session must not record a detection");

    result = Step(true, true, 1000, params, result.next_memory);
    Expect(result.next_memory.phase == ZebraStopPhase::kWaitingForClearance,
           "first detection must start clearance tracking");
    Expect(!result.telemetry.controlled_stop_requested,
           "first detection must not request a stop");

    result = Step(true, true, 1200, params, result.next_memory);
    Expect(result.next_memory.last_detection_time_ms == 1200,
           "continued detection must move the start of the absence interval");
    result = Step(true, false, 1699, params, result.next_memory);
    Expect(result.next_memory.phase == ZebraStopPhase::kWaitingForClearance,
           "absence shorter than the configured interval must not arm reentry");
    result = Step(true, false, 1700, params, result.next_memory);
    Expect(result.next_memory.phase == ZebraStopPhase::kArmedForReentry,
           "continuous absence at the configured interval must arm reentry");

    result = Step(true, true, 1800, params, result.next_memory);
    Expect(result.next_memory.phase == ZebraStopPhase::kStopDelay,
           "a detection after clearance must start the stop delay");
    result = Step(true, false, 2099, params, result.next_memory);
    Expect(result.next_memory.phase == ZebraStopPhase::kStopDelay &&
               !result.telemetry.controlled_stop_requested,
           "loss of Zebra during the committed delay must not cancel or finish it");
    result = Step(true, false, 2100, params, result.next_memory);
    Expect(result.next_memory.phase == ZebraStopPhase::kStopRequested &&
               result.telemetry.controlled_stop_requested,
           "the configured stop delay must end in a controlled stop request");
    result = Step(true, false, 2200, params, result.next_memory);
    Expect(result.telemetry.controlled_stop_requested,
           "the request must remain asserted for the active drive session");

    result = Step(false, false, 2300, params, result.next_memory);
    Expect(result.next_memory.phase == ZebraStopPhase::kWaitingForFirstDetection &&
               !result.telemetry.controlled_stop_requested,
           "leaving the drive session must reset the Zebra lifecycle");

    params.zebra_reentry_arm_absence_ms = 0;
    params.zebra_controlled_stop_delay_ms = 0;
    result = Step(true, true, 3000, params, {});
    result = Step(true, false, 3000, params, result.next_memory);
    Expect(result.next_memory.phase == ZebraStopPhase::kArmedForReentry,
           "zero absence interval must arm on the first absent frame");
    result = Step(true, true, 3001, params, result.next_memory);
    Expect(result.telemetry.controlled_stop_requested,
           "zero stop delay must request stop on the reentry frame");

    params.zebra_reentry_arm_absence_ms = 500;
    params.zebra_controlled_stop_delay_ms = 300;
    result = Step(true, true, 5000, params, {});
    result = Step(true, false, 4900, params, result.next_memory);
    Expect(result.next_memory.phase == ZebraStopPhase::kWaitingForClearance &&
               result.telemetry.absence_elapsed_ms == 0,
           "capture timestamp regression must not falsely advance the state");

    std::cout << "zebra stop scene tests passed\n";
    return 0;
}
