#include <cstdlib>
#include <iostream>

#include "control/ml_speed_policy.hpp"
#include "control/tuning_state.hpp"

namespace {

void Expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

}  // namespace

int main() {
    ls2k::port::RuntimeParameters params{};
    params.running_speed_target = 400.0;
    params.ml.maneuver.speed_target = 180.0;
    ls2k::port::PerceptionResult perception{};
    Expect(ls2k::control::SelectPerceptionSpeedTarget(perception, params) == 400.0,
           "ordinary running target must be the inactive default");
    perception.ml.active = true;
    Expect(ls2k::control::SelectPerceptionSpeedTarget(perception, params) == 400.0,
           "unselected ML state must not change speed");
    perception.ml.takeover_selected = true;
    const double ml_default = ls2k::control::SelectPerceptionSpeedTarget(perception, params);
    Expect(ml_default == 180.0, "ML active target must override ordinary running speed");

    ls2k::control::RuntimeTuningSnapshot tuning{};
    Expect(ls2k::control::ResolveRuntimeSpeedTarget(tuning, ml_default, 100U) == 180.0,
           "ML target must remain when no online override exists");
    tuning.target_speed_override_enabled = true;
    tuning.target_speed_override_value = 75.0;
    tuning.target_speed_override_expire_at_ms = 200U;
    Expect(ls2k::control::ResolveRuntimeSpeedTarget(tuning, ml_default, 100U) == 75.0,
           "valid online override must outrank ML target");
    Expect(ls2k::control::ResolveRuntimeSpeedTarget(tuning, ml_default, 200U) == 180.0,
           "expired online override must release back to ML target");
    std::cout << "ml_speed_policy_test passed\n";
    return 0;
}
