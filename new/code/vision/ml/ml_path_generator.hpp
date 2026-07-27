#ifndef LS2K_VISION_ML_PATH_GENERATOR_HPP
#define LS2K_VISION_ML_PATH_GENERATOR_HPP

#include <cstddef>

#include "port/ml_types.hpp"
#include "vision/bev/bev_reference_path_builder.hpp"

namespace ls2k::vision::ml {

/// Builds the current-frame ML path from the selected observed boundary. The
/// offset is directed away from the road: negative for left, positive for right.
port::BEVReferencePath BuildMlBoundaryOffsetPath(
    port::MlAction action,
    const BEVRoadPathFacts& road_path_facts,
    std::size_t min_boundary_samples,
    float outward_offset_m);

}  // namespace ls2k::vision::ml

#endif  // LS2K_VISION_ML_PATH_GENERATOR_HPP
