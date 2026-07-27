#include "vision/ml/ml_observer.hpp"

#include <algorithm>
#include <chrono>
#include <limits>

#include "vision/ml/ml_class_mapping.hpp"
#include "vision/ml/roi_sampler.hpp"

namespace ls2k::vision::ml {
namespace {

using MlClock = std::chrono::steady_clock;

std::uint32_t ElapsedUs(MlClock::time_point start, MlClock::time_point end) {
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    return static_cast<std::uint32_t>(std::min<std::int64_t>(
        std::max<std::int64_t>(elapsed, 0),
        std::numeric_limits<std::uint32_t>::max()));
}

}  // namespace

MlObservation RunMlObserver(const MlObserverInput& input,
                            const port::MlParameters& params) {
    MlObservation out{};
    out.enabled = params.enabled;
    if (!params.enabled) {
        return out;
    }
    if (input.classifier != nullptr) {
        out.artifact_candidate_id = input.classifier->ArtifactId();
        out.artifact_sha256 = input.classifier->ArtifactSha256();
        out.artifact_item_count = input.classifier->ArtifactItemCount();
    }
    if (input.frame == nullptr || input.projector == nullptr ||
        input.classifier == nullptr) {
        out.reason = "input_unavailable";
        return out;
    }

    const MlClock::time_point total_start = MlClock::now();
    const MlClock::time_point detector_start = total_start;
    out.detector = DetectRedRectangle(*input.frame,
                                      *input.projector,
                                      params.roi,
                                      input.rectangle_projection_lut);
    const MlClock::time_point detector_end = MlClock::now();
    out.detector_us = ElapsedUs(detector_start, detector_end);
    out.detector_valid = out.detector.valid;

    const MlClock::time_point roi_start = detector_end;
    if (out.detector.valid) {
        out.roi = SampleSquareRoi32(*input.frame,
                                   *input.projector,
                                   out.detector,
                                   params.roi);
    }
    const MlClock::time_point roi_end = MlClock::now();
    out.roi_us = ElapsedUs(roi_start, roi_end);

    const MlClock::time_point classifier_start = roi_end;
    if (out.roi.valid) {
        const port::MlClassifierOutput classifier_output =
            input.classifier->Predict(out.roi);
        out.classification = classifier_output.classification;
        out.descriptor = classifier_output.v9_descriptor;
        out.replay = classifier_output.v9_replay;
    }
    const MlClock::time_point classifier_end = MlClock::now();
    out.classifier_us = ElapsedUs(classifier_start, classifier_end);
    out.total_us = ElapsedUs(total_start, classifier_end);

    const MlClassificationPolicy policy =
        SelectMlClassificationPolicy(out.classification.backend, params);
    if (!AcceptMlClassification(out.classification, policy)) {
        out.reason = "not_accepted";
        return out;
    }
    out.accepted = true;
    out.mapped_action = MapMlClass(out.classification.class_id,
                                  params.class_mapping);
    out.reason = "accepted";
    return out;
}

}  // namespace ls2k::vision::ml
