#include "vision/ml/ml_class_mapping.hpp"

namespace ls2k::vision::ml {

port::MlAction ParseMlAction(const std::string& token) {
    if (token == "straight") return port::MlAction::kStraight;
    if (token == "left") return port::MlAction::kLeft;
    if (token == "right") return port::MlAction::kRight;
    return port::MlAction::kUnmapped;
}

port::MlAction MapMlClass(int class_id,
                          const port::MlClassMappingParameters& mapping) {
    if (class_id == 0) return ParseMlAction(mapping.class_0_action);
    if (class_id == 1) return ParseMlAction(mapping.class_1_action);
    if (class_id == 2) return ParseMlAction(mapping.class_2_action);
    return port::MlAction::kUnmapped;
}

}  // namespace ls2k::vision::ml
