#include <cstdlib>
#include <iostream>
#include <string>

#include "control/zebra_controlled_stop.hpp"
#include "control/motion_supervisor.hpp"

namespace {

void Expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

}  // namespace

int main() {
    ls2k::port::ZebraStopTelemetry zebra{};
    zebra.controlled_stop_requested = true;

    ls2k::control::MotionIntent intent{};
    intent.start_requested = true;
    ls2k::control::ApplyZebraControlledStopRequest(
        zebra, ls2k::control::MotionPhase::kRunning, intent);
    Expect(!intent.start_requested && intent.stop_requested,
           "running Zebra request must become the ordinary stop intent");
    ls2k::control::MotionSupervisorInputs supervisor_input{};
    supervisor_input.state.phase = ls2k::control::MotionPhase::kRunning;
    supervisor_input.state.last_effective_speed_target = 100.0;
    supervisor_input.intent = intent;
    supervisor_input.startup_complete = true;
    supervisor_input.gate_clear = true;
    supervisor_input.motion_stop_ms = 500;
    const ls2k::control::MotionDecision decision =
        ls2k::control::MotionSupervisor{}.Evaluate(supervisor_input);
    Expect(decision.state.phase == ls2k::control::MotionPhase::kStopping &&
               !decision.require_emergency_stop,
           "the ordinary stop intent must enter controlled Stopping, not fail-safe");

    intent = {};
    intent.start_requested = true;
    ls2k::control::ApplyZebraControlledStopRequest(
        zebra, ls2k::control::MotionPhase::kDisarmed, intent);
    Expect(intent.start_requested && !intent.stop_requested,
           "a stale request outside a drive session must not affect the next start");

    zebra.controlled_stop_requested = false;
    intent = {};
    intent.start_requested = true;
    ls2k::control::ApplyZebraControlledStopRequest(
        zebra, ls2k::control::MotionPhase::kSpinup, intent);
    Expect(intent.start_requested && !intent.stop_requested,
           "an inactive Zebra request must not change motion intent");

    std::cout << "zebra controlled stop tests passed\n";
    return 0;
}
