#ifndef LS2K_CONTROL_ZEBRA_CONTROLLED_STOP_HPP
#define LS2K_CONTROL_ZEBRA_CONTROLLED_STOP_HPP

#include "control/motion_types.hpp"
#include "port/zebra_types.hpp"

namespace ls2k::control {

void ApplyZebraControlledStopRequest(
    const port::ZebraStopTelemetry& zebra,
    MotionPhase phase,
    MotionIntent& intent);

}  // namespace ls2k::control

#endif  // LS2K_CONTROL_ZEBRA_CONTROLLED_STOP_HPP
