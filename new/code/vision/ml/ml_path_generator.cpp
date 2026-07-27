#include "vision/ml/ml_path_generator.hpp"

#include <cmath>

#include "port/bev_reference_path_utils.hpp"

namespace ls2k::vision::ml {

port::BEVReferencePath BuildMlBoundaryOffsetPath(
    port::MlAction action,
    const BEVRoadPathFacts& road_path_facts,
    std::size_t min_boundary_samples,
    float outward_offset_m) {
    if ((action != port::MlAction::kLeft && action != port::MlAction::kRight) ||
        !std::isfinite(outward_offset_m) || outward_offset_m < 0.0F) {
        return {};
    }
    const ObservedBoundarySide side = action == port::MlAction::kLeft
        ? ObservedBoundarySide::kLeft : ObservedBoundarySide::kRight;
    port::BEVReferencePath path = BuildObservedBoundaryReferencePath(
        road_path_facts, side, min_boundary_samples);
    if (path.mode != port::ReferenceMode::kMlObservedBoundary) {
        return {};
    }
    const float signed_offset = action == port::MlAction::kLeft
        ? -outward_offset_m : outward_offset_m;
    path.mode = port::ReferenceMode::kMlBoundaryOffset;
    for (port::BEVPathSample& sample : path.sampled_path) {
        if (!port::IsFiniteReferenceSample(sample)) {
            continue;
        }
        sample.point.lateral_m += signed_offset;
        sample.source = port::BEVPathPointSource::kMlBoundaryOffset;
    }
    return path;
}

}  // namespace ls2k::vision::ml
