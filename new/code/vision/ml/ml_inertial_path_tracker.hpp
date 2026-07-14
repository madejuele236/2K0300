#ifndef LS2K_VISION_ML_INERTIAL_PATH_TRACKER_HPP
#define LS2K_VISION_ML_INERTIAL_PATH_TRACKER_HPP

#include "port/ml_types.hpp"
#include "port/motion_history_types.hpp"
#include "port/runtime_parameter_types.hpp"

namespace ls2k::vision::ml {

class MlInertialPathTracker {
public:
    static void Start(const port::MlLockedManeuver& locked,
                      port::MlInertialTrackerState& state);
    static port::MlInertialTrackerResult Step(
        const port::MlLockedManeuver& locked,
        const port::MotionHistory& history,
        uint64_t target_time_ms,
        const port::MotionOdometryParameters& odometry,
        const port::MlManeuverParameters& params,
        port::MlInertialTrackerState& state);
    static void Reset(port::MlInertialTrackerState& state);
};

}  // namespace ls2k::vision::ml

#endif
