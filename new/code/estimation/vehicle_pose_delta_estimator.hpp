#ifndef LS2K_ESTIMATION_VEHICLE_POSE_DELTA_ESTIMATOR_HPP
#define LS2K_ESTIMATION_VEHICLE_POSE_DELTA_ESTIMATOR_HPP

#include <cstdint>

#include "port/control_command_history_types.hpp"
#include "port/motion_history_types.hpp"
#include "port/runtime_parameter_types.hpp"
#include "port/vehicle_pose_delta_types.hpp"

namespace ls2k::estimation {

port::VehiclePoseDelta EstimateVehiclePoseDelta(
    uint64_t start_time_ms,
    uint64_t now_time_ms,
    uint64_t end_time_ms,
    const port::MotionHistory& motion_history,
    const port::ControlCommandHistory& command_history,
    const port::ReferenceTimeAlignmentParameters& params);

}  // namespace ls2k::estimation

#endif  // LS2K_ESTIMATION_VEHICLE_POSE_DELTA_ESTIMATOR_HPP
