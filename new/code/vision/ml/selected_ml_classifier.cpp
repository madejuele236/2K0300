#include "vision/ml/selected_ml_classifier.hpp"

namespace ls2k::vision::ml {

bool AcceptMlClassification(const port::MlClassificationResult& result,
                            const port::MlV9Parameters& acceptance) {
    return result.valid && result.margin >= acceptance.min_margin &&
           (!result.distance_valid || result.best_distance <= acceptance.max_best_distance);
}

}  // namespace ls2k::vision::ml
