#include "vision/ml/selected_ml_classifier.hpp"

namespace ls2k::vision::ml {

MlClassificationPolicy SelectMlClassificationPolicy(
    port::MlClassifierBackend backend,
    const port::MlParameters& params) {
    switch (backend) {
        case port::MlClassifierBackend::kV9Hamming:
            return {params.v9.min_margin, params.v9.max_best_distance,
                    params.v9.confirm_frames, false};
        case port::MlClassifierBackend::kTfliteInt8:
            return {params.tflite_identity.min_margin,
                    params.tflite_identity.max_best_distance,
                    params.tflite_identity.confirm_frames, true};
    }
    return {};
}

bool AcceptMlClassification(const port::MlClassificationResult& result,
                            const MlClassificationPolicy& policy) {
    return result.valid && result.margin >= policy.min_margin &&
           (!policy.require_distance || result.distance_valid) &&
           (!result.distance_valid || result.best_distance <= policy.max_best_distance);
}

}  // namespace ls2k::vision::ml
