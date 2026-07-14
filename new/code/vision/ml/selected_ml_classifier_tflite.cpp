#include "vision/ml/selected_ml_classifier.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>

#include "generated_tflite_classifier_artifact.hpp"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

namespace ls2k::vision::ml {
namespace {

// Keep a bounded arena so model drift fails at AllocateTensors instead of
// silently consuming unrelated runtime memory. Board evidence records the
// actual used size for the selected frozen graph.
constexpr std::size_t kTensorArenaBytes = 16U * 1024U;

bool ShapeEquals(const TfLiteTensor& tensor,
                 const std::initializer_list<int>& expected) {
    if (tensor.dims == nullptr || tensor.dims->size != static_cast<int>(expected.size())) {
        return false;
    }
    int index = 0;
    for (const int dimension : expected) {
        if (tensor.dims->data[index++] != dimension) return false;
    }
    return true;
}

}  // namespace

struct SelectedMlClassifier::Impl {
    tflite::MicroMutableOpResolver<5> resolver{};
    alignas(16) std::array<std::uint8_t, kTensorArenaBytes> arena{};
    const tflite::Model* model = nullptr;
    std::unique_ptr<tflite::MicroInterpreter> interpreter{};
    bool initialized = false;
};

SelectedMlClassifier::SelectedMlClassifier() = default;
SelectedMlClassifier::~SelectedMlClassifier() = default;

bool SelectedMlClassifier::Initialize() {
    impl_ = std::make_unique<Impl>();
    impl_->initialized = false;
    impl_->model = tflite::GetModel(generated::kTfliteClassifierModel);
    if (impl_->model == nullptr || impl_->model->version() != TFLITE_SCHEMA_VERSION ||
        impl_->resolver.AddSpaceToDepth() != kTfLiteOk ||
        impl_->resolver.AddConv2D() != kTfLiteOk ||
        impl_->resolver.AddMaxPool2D() != kTfLiteOk ||
        impl_->resolver.AddMean() != kTfLiteOk ||
        impl_->resolver.AddFullyConnected() != kTfLiteOk) {
        return false;
    }
    impl_->interpreter = std::make_unique<tflite::MicroInterpreter>(
        impl_->model, impl_->resolver, impl_->arena.data(), impl_->arena.size());
    if (impl_->interpreter->AllocateTensors() != kTfLiteOk ||
        impl_->interpreter->inputs_size() != 1 ||
        impl_->interpreter->outputs_size() != 1) {
        return false;
    }
    const TfLiteTensor* input = impl_->interpreter->input(0);
    const TfLiteTensor* output = impl_->interpreter->output(0);
    if (input == nullptr || output == nullptr || input->type != kTfLiteInt8 ||
        output->type != kTfLiteInt8 || !ShapeEquals(*input, {1, 32, 32, 1}) ||
        !ShapeEquals(*output, {1, 4}) || input->params.zero_point != -128 ||
        output->params.zero_point != 19 ||
        std::fabs(input->params.scale - 1.0F / 255.0F) > 1.0e-7F ||
        std::fabs(output->params.scale - 0.148821F) > 1.0e-5F) {
        return false;
    }
    impl_->initialized = true;
    return true;
}

port::MlClassifierOutput SelectedMlClassifier::Predict(const port::MlGrayRoi32& roi) {
    port::MlClassifierOutput out{};
    out.classification.backend = port::MlClassifierBackend::kTfliteInt8;
    if (impl_ == nullptr || !impl_->initialized || !roi.valid) {
        out.classification.reason = "not_initialized_or_invalid_roi";
        return out;
    }
    TfLiteTensor* input = impl_->interpreter->input(0);
    for (std::size_t index = 0; index < roi.gray.size(); ++index) {
        input->data.int8[index] = static_cast<std::int8_t>(
            static_cast<int>(roi.gray[index]) - 128);
    }
    if (impl_->interpreter->Invoke() != kTfLiteOk) {
        out.classification.reason = "invoke_failed";
        return out;
    }
    const TfLiteTensor* output = impl_->interpreter->output(0);
    out.tflite_feature.valid = true;
    std::copy_n(output->data.int8, out.tflite_feature.values.size(),
                out.tflite_feature.values.begin());
    const generated::IdentityPrototypeScore score =
        generated::ScoreIdentityFeature(out.tflite_feature.values.data());
    for (int cls = 0; cls < 3; ++cls) {
        out.classification.class_scores[static_cast<std::size_t>(cls)] =
            -score.class_distances[static_cast<std::size_t>(cls)];
    }
    out.classification.valid = true;
    out.classification.class_id = score.class_id;
    out.classification.margin = score.margin;
    out.classification.distance_valid = true;
    out.classification.best_distance =
        score.class_distances[static_cast<std::size_t>(score.class_id)];
    out.classification.reason = "ok";
    return out;
}

const char* SelectedMlClassifier::BackendName() const { return "tflite_int8"; }
const char* SelectedMlClassifier::ArtifactId() const {
    return generated::kTfliteClassifierArtifactId;
}
const char* SelectedMlClassifier::ArtifactSha256() const {
    return generated::kTfliteClassifierArtifactSha256;
}
std::size_t SelectedMlClassifier::ArtifactItemCount() const {
    return generated::kIdentityPrototypeCount;
}
std::size_t SelectedMlClassifier::WorkingMemoryBytes() const {
    return impl_ == nullptr || impl_->interpreter == nullptr
        ? 0U : impl_->interpreter->arena_used_bytes();
}

}  // namespace ls2k::vision::ml
