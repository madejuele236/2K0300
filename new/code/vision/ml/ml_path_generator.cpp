#include "vision/ml/ml_path_generator.hpp"

#include "vision/bev/bev_reference_path_builder.hpp"

namespace ls2k::vision::ml {

port::BEVReferencePath ObservedBoundaryMlPathGenerator::Generate(
    const MlPathGenerationInput& input) const {
    if (input.road_path_facts == nullptr ||
        (input.action != port::MlAction::kLeft && input.action != port::MlAction::kRight)) {
        return {};
    }
    const ObservedBoundarySide side = input.action == port::MlAction::kLeft
        ? ObservedBoundarySide::kLeft : ObservedBoundarySide::kRight;
    return BuildObservedBoundaryReferencePath(*input.road_path_facts,
                                              side,
                                              input.min_boundary_samples);
}

}  // namespace ls2k::vision::ml
