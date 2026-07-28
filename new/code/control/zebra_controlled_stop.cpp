#include "control/zebra_controlled_stop.hpp"

namespace ls2k::control {

void ApplyZebraControlledStopRequest(
    const port::ZebraStopTelemetry& zebra,
    MotionPhase phase,
    MotionIntent& intent) {
    if (!zebra.controlled_stop_requested ||
        (phase != MotionPhase::kSpinup && phase != MotionPhase::kRunning)) {
        return;
    }
    intent.start_requested = false;
    intent.stop_requested = true;
}

}  // namespace ls2k::control
