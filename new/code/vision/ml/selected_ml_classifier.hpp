#ifndef LS2K_VISION_ML_SELECTED_ML_CLASSIFIER_HPP
#define LS2K_VISION_ML_SELECTED_ML_CLASSIFIER_HPP

#include <memory>

#include "port/ml_types.hpp"
#include "port/runtime_parameter_types.hpp"

#define LS2K_ML_CLASSIFIER_BACKEND_V9 1
#define LS2K_ML_CLASSIFIER_BACKEND_TFLITE_INT8 2

#ifndef LS2K_ML_CLASSIFIER_BACKEND
#define LS2K_ML_CLASSIFIER_BACKEND LS2K_ML_CLASSIFIER_BACKEND_V9
#endif

#if LS2K_ML_CLASSIFIER_BACKEND != LS2K_ML_CLASSIFIER_BACKEND_V9 && \
    LS2K_ML_CLASSIFIER_BACKEND != LS2K_ML_CLASSIFIER_BACKEND_TFLITE_INT8
#error "LS2K_ML_CLASSIFIER_BACKEND must select exactly one supported classifier"
#endif

namespace ls2k::vision::ml {

class MlClassifier {
public:
    virtual ~MlClassifier() = default;
    virtual bool Initialize() = 0;
    virtual port::MlClassifierOutput Predict(const port::MlGrayRoi32& roi) = 0;
    virtual const char* BackendName() const = 0;
    virtual const char* ArtifactId() const = 0;
    virtual const char* ArtifactSha256() const = 0;
    virtual std::size_t ArtifactItemCount() const = 0;
    virtual std::size_t WorkingMemoryBytes() const = 0;
};

class SelectedMlClassifier final : public MlClassifier {
public:
    SelectedMlClassifier();
    ~SelectedMlClassifier();
    SelectedMlClassifier(const SelectedMlClassifier&) = delete;
    SelectedMlClassifier& operator=(const SelectedMlClassifier&) = delete;

    bool Initialize() override;
    port::MlClassifierOutput Predict(const port::MlGrayRoi32& roi) override;
    const char* BackendName() const override;
    const char* ArtifactId() const override;
    const char* ArtifactSha256() const override;
    std::size_t ArtifactItemCount() const override;
    std::size_t WorkingMemoryBytes() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

bool AcceptMlClassification(const port::MlClassificationResult& result,
                            const port::MlV9Parameters& acceptance);

}  // namespace ls2k::vision::ml

#endif
