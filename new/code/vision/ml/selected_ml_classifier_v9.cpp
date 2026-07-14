#include "vision/ml/selected_ml_classifier.hpp"

#include "generated_v9_artifact.hpp"
#include "vision/ml/v9_descriptor.hpp"
#include "vision/ml/v9_replay.hpp"

namespace ls2k::vision::ml {

struct SelectedMlClassifier::Impl {
    port::V9ArtifactView artifact = generated::Artifact();
    bool initialized = false;
};

SelectedMlClassifier::SelectedMlClassifier() = default;
SelectedMlClassifier::~SelectedMlClassifier() = default;

bool SelectedMlClassifier::Initialize() {
    impl_ = std::make_unique<Impl>();
    impl_->initialized = ValidateV9Artifact(impl_->artifact);
    return impl_->initialized;
}

port::MlClassifierOutput SelectedMlClassifier::Predict(const port::MlGrayRoi32& roi) {
    port::MlClassifierOutput out{};
    out.classification.backend = port::MlClassifierBackend::kV9Hamming;
    if (impl_ == nullptr || !impl_->initialized || !roi.valid) {
        out.classification.reason = "not_initialized_or_invalid_roi";
        return out;
    }
    out.v9_descriptor = BuildV9Descriptor(roi);
    out.v9_replay = ReplayV9Descriptor(out.v9_descriptor, impl_->artifact);
    if (!out.v9_replay.valid) {
        out.classification.reason = "v9_replay_invalid";
        return out;
    }
    out.classification.valid = true;
    out.classification.class_id = out.v9_replay.class_id;
    out.classification.margin = out.v9_replay.margin;
    out.classification.distance_valid = true;
    out.classification.best_distance = out.v9_replay.best_distance;
    out.classification.reason = "ok";
    return out;
}

const char* SelectedMlClassifier::BackendName() const { return "v9_hamming"; }
const char* SelectedMlClassifier::ArtifactId() const {
    const port::V9ArtifactView artifact = generated::Artifact();
    return artifact.candidate_id == nullptr ? "unavailable" : artifact.candidate_id;
}
const char* SelectedMlClassifier::ArtifactSha256() const {
    const port::V9ArtifactView artifact = generated::Artifact();
    return artifact.template_codes_sha256 == nullptr
        ? "unavailable" : artifact.template_codes_sha256;
}
std::size_t SelectedMlClassifier::ArtifactItemCount() const {
    return generated::Artifact().prototype_count;
}
std::size_t SelectedMlClassifier::WorkingMemoryBytes() const { return 0U; }

}  // namespace ls2k::vision::ml
