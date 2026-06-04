#ifndef LS2K_REFERENCE_REFERENCE_TIME_ALIGNMENT_HPP
#define LS2K_REFERENCE_REFERENCE_TIME_ALIGNMENT_HPP

#include <cstdint>

#include "port/bev_reference_types.hpp"
#include "port/runtime_parameter_types.hpp"
#include "port/vehicle_pose_delta_types.hpp"

namespace ls2k::reference {

struct ReferenceTimeAlignmentResult {
    port::BEVReferencePath reference_path{};
    port::ReferenceTimeAlignmentFacts facts{};
};

/// Align a capture-time reference path into the control-effective-time vehicle frame.
ReferenceTimeAlignmentResult AlignReferencePathToVehiclePoseDelta(
    const port::BEVReferencePath& reference_path,
    uint64_t reference_capture_time_ms,
    uint64_t control_time_ms,
    uint64_t control_effective_time_ms,
    const port::VehiclePoseDelta& pose_delta,
    const port::RuntimeParameters& params);

}  // namespace ls2k::reference

#endif  // LS2K_REFERENCE_REFERENCE_TIME_ALIGNMENT_HPP
