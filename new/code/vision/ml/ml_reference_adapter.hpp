#ifndef LS2K_VISION_ML_REFERENCE_ADAPTER_HPP
#define LS2K_VISION_ML_REFERENCE_ADAPTER_HPP

#include "port/ml_types.hpp"

namespace ls2k::vision::ml {

port::MlLockedManeuver LockObservedBoundaryToMarker(
    port::MlAction action,
    uint64_t lock_time_ms,
    const port::MlOrientedRectangle& rectangle,
    const port::BEVReferencePath& observed_boundary);

port::BEVReferencePath TransformMarkerPathToCurrentVehicle(
    const port::MlLockedManeuver& locked,
    const port::MlPoseDelta& pose_delta);

}  // namespace ls2k::vision::ml

#endif
