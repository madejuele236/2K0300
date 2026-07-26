#ifndef LS2K_VISION_ML_PATH_GENERATOR_HPP
#define LS2K_VISION_ML_PATH_GENERATOR_HPP

#include <cstddef>

#include "port/ml_types.hpp"
#include "vision/bev/bev_reference_path_builder.hpp"

namespace ls2k::vision::ml {

struct MlPathGenerationInput {
    port::MlAction action = port::MlAction::kUnmapped;
    const BEVRoadPathFacts* road_path_facts = nullptr;
    std::size_t min_boundary_samples = 0;
};

/// Replaceable path-generation contract. Detector, predictor, class mapping,
/// scene confirmation, and inertial tracking depend only on this interface.
class MlPathGenerator {
public:
    virtual ~MlPathGenerator() = default;
    virtual port::BEVReferencePath Generate(const MlPathGenerationInput& input) const = 0;
};

/// v1 generator: left consumes literal actual-left; right consumes literal
/// actual-right. It never synthesizes a side, applies an offset, or substitutes
/// the ordinary center path.
class ObservedBoundaryMlPathGenerator final : public MlPathGenerator {
public:
    port::BEVReferencePath Generate(const MlPathGenerationInput& input) const override;
};

}  // namespace ls2k::vision::ml

#endif  // LS2K_VISION_ML_PATH_GENERATOR_HPP
