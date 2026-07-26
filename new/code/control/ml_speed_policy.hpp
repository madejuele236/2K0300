#ifndef LS2K_CONTROL_ML_SPEED_POLICY_HPP
#define LS2K_CONTROL_ML_SPEED_POLICY_HPP

#include "port/perception_result.hpp"
#include "port/runtime_parameter_types.hpp"

namespace ls2k::control {

inline double SelectPerceptionSpeedTarget(const port::PerceptionResult& perception,
                                          const port::RuntimeParameters& params) {
    return perception.ml.active ? params.ml.maneuver.speed_target
                                : params.running_speed_target;
}

}  // namespace ls2k::control

#endif  // LS2K_CONTROL_ML_SPEED_POLICY_HPP
