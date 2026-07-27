#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "vision/bev/bev_projector.hpp"
#include "vision/ml/red_rectangle_detector.hpp"
#include "vision/ml/roi_sampler.hpp"
#include "vision/ml/ml_observer.hpp"

namespace {

class FakeClassifier final : public ls2k::vision::ml::MlClassifier {
public:
    bool Initialize() override { return true; }
    ls2k::port::MlClassifierOutput Predict(
        const ls2k::port::MlGrayRoi32& roi) override {
        ls2k::port::MlClassifierOutput out{};
        if (!roi.valid) return out;
        out.classification.valid = true;
        out.classification.backend = ls2k::port::MlClassifierBackend::kV9Hamming;
        out.classification.class_id = 1;
        out.classification.margin = 10;
        out.classification.distance_valid = true;
        out.classification.best_distance = 5;
        return out;
    }
    const char* BackendName() const override { return "fake_v9"; }
    const char* ArtifactId() const override { return "fake-artifact"; }
    const char* ArtifactSha256() const override { return "fake-sha"; }
    std::size_t ArtifactItemCount() const override { return 3U; }
    std::size_t WorkingMemoryBytes() const override { return 0U; }
};

void Expect(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

ls2k::vision::BEVProjector MakeProjector() {
    ls2k::port::BEVProjectorCalibration calibration{};
    calibration.source_points = {{{0.0F, 0.0F}, {0.0F, 79.0F},
                                  {79.0F, 0.0F}, {79.0F, 79.0F}}};
    calibration.target_points = {{{0.0F, -0.4F}, {0.0F, 0.39F},
                                  {0.79F, -0.4F}, {0.79F, 0.39F}}};
    ls2k::vision::BEVProjector projector;
    Expect(projector.Configure(calibration), "identity-scale projector configuration failed");
    return projector;
}

void SetYuyvPair(std::vector<std::uint8_t>& frame, int row, int pair,
                 std::uint8_t y0, std::uint8_t u, std::uint8_t y1, std::uint8_t v) {
    const std::size_t offset = static_cast<std::size_t>(row * 160 + pair * 4);
    frame[offset] = y0;
    frame[offset + 1U] = u;
    frame[offset + 2U] = y1;
    frame[offset + 3U] = v;
}

ls2k::port::CameraPixelFrameView View(const std::vector<std::uint8_t>& bytes) {
    ls2k::port::CameraPixelFrameView view{};
    view.valid = true;
    view.format = ls2k::port::CameraFrameFormat::kYuyv;
    view.data = bytes.data();
    view.width = 80;
    view.height = 80;
    view.stride = 160;
    view.frame_id = 77;
    view.capture_time_ms = 1234;
    return view;
}

void PaintMarker(std::vector<std::uint8_t>& frame, float center_forward,
                 float center_lateral, float angle,
                 float half_long = 0.15F, float half_short = 0.05F) {
    const float long_f = std::sin(angle);
    const float long_l = std::cos(angle);
    const float short_f = std::cos(angle);
    const float short_l = -std::sin(angle);
    for (int row = 0; row < 80; ++row) {
        for (int pair = 0; pair < 40; ++pair) {
            bool red = false;
            for (int within = 0; within < 2; ++within) {
                const int col = pair * 2 + within;
                const float forward = row * 0.01F;
                const float lateral = col * 0.01F - 0.4F;
                const float df = forward - center_forward;
                const float dl = lateral - center_lateral;
                const float along_long = df * long_f + dl * long_l;
                const float along_short = df * short_f + dl * short_l;
                red = red || (std::fabs(along_long) <= half_long &&
                              std::fabs(along_short) <= half_short);
            }
            if (red) SetYuyvPair(frame, row, pair, 160, 90, 160, 205);
        }
    }
}

ls2k::port::MlRoiParameters DetectorParameters() {
    ls2k::port::MlRoiParameters params{};
    params.search_forward_min_m = 0.05;
    params.search_forward_max_m = 0.75;
    params.search_lateral_limit_m = 0.35;
    params.grid_forward_step_m = 0.01;
    params.grid_lateral_step_m = 0.01;
    params.red_y_min = 45; params.red_y_max = 230;
    params.red_u_min = 70; params.red_u_max = 135;
    params.red_v_min = 140; params.red_v_max = 220;
    params.expected_long_edge_m = 0.30;
    params.expected_short_edge_m = 0.10;
    params.long_edge_tolerance_m = 0.04;
    params.short_edge_tolerance_m = 0.04;
    params.max_long_edge_to_lateral_rad = 0.7853982;
    params.min_component_cells = 100;
    params.min_rectangularity = 0.75;
    params.min_red_fill_ratio = 0.70;
    params.score_size_weight = 1.0;
    params.score_rectangularity_weight = 1.0;
    params.score_red_fill_weight = 1.0;
    params.score_orientation_weight = 1.0;
    return params;
}

void TestTiltedDetectionAndTieBreak() {
    std::vector<std::uint8_t> frame(80U * 160U, 0U);
    for (int row = 0; row < 80; ++row)
        for (int pair = 0; pair < 40; ++pair)
            SetYuyvPair(frame, row, pair, 20, 128, 20, 128);
    PaintMarker(frame, 0.25F, 0.0F, 0.25F);
    PaintMarker(frame, 0.58F, 0.0F, 0.25F);
    const auto rectangle = ls2k::vision::ml::DetectRedRectangle(
        View(frame), MakeProjector(), DetectorParameters());
    Expect(rectangle.valid, "tilted red rectangle should be detected");
    Expect(rectangle.frame_id == 77 && rectangle.capture_time_ms == 1234,
           "rectangle observation must retain source-frame identity");
    Expect(std::fabs(rectangle.long_edge_m - 0.30F) < 0.04F,
           "detected long edge is outside configured tolerance");
    Expect(std::fabs(rectangle.short_edge_m - 0.10F) < 0.04F,
           "detected short edge is outside configured tolerance");
    Expect(std::atan2(std::fabs(rectangle.long_axis_forward),
                      std::fabs(rectangle.long_axis_lateral)) < 0.35F,
           "tilted long-axis orientation was not preserved");
    Expect(std::fabs(rectangle.long_edge_to_lateral_rad -
                     std::atan2(std::fabs(rectangle.long_axis_forward),
                                std::fabs(rectangle.long_axis_lateral))) < 1.0e-5F,
           "rectangle orientation telemetry mismatch");
    ls2k::vision::ml::MlRedRectangleProjectionLut projection_lut{};
    const auto cached_first = ls2k::vision::ml::DetectRedRectangle(
        View(frame), MakeProjector(), DetectorParameters(), &projection_lut);
    const auto cached_second = ls2k::vision::ml::DetectRedRectangle(
        View(frame), MakeProjector(), DetectorParameters(), &projection_lut);
    Expect(projection_lut.valid && cached_first.valid && cached_second.valid &&
               cached_first.center.forward_m == rectangle.center.forward_m &&
               cached_second.center.lateral_m == rectangle.center.lateral_m,
           "cached ML projection grid must preserve detector output exactly");
    Expect(rectangle.center.forward_m < 0.4F,
           "equal-quality markers must select the nearer forward target");
}

void TestPositiveAndNegativeFortyFiveDegreeDetection() {
    constexpr float kFortyFiveDegrees = 0.7853981633974483F;
    for (const float angle : {kFortyFiveDegrees, -kFortyFiveDegrees}) {
        std::vector<std::uint8_t> frame(80U * 160U, 0U);
        for (int row = 0; row < 80; ++row) {
            for (int pair = 0; pair < 40; ++pair) {
                SetYuyvPair(frame, row, pair, 220, 128, 220, 128);
            }
        }
        PaintMarker(frame, 0.38F, 0.0F, angle);
        const auto rectangle = ls2k::vision::ml::DetectRedRectangle(
            View(frame), MakeProjector(), DetectorParameters());
        Expect(rectangle.valid, "plus/minus 45 degree red rectangle should be detected");
        Expect(std::fabs(rectangle.long_edge_to_lateral_rad - kFortyFiveDegrees) < 0.03F,
               "detected 45 degree orientation is outside grid tolerance");
    }
}

void TestExpandedRedRangeRejectsNonRedAndWrongSizeDistractors() {
    for (const std::array<std::uint8_t, 3> yuv :
         {std::array<std::uint8_t, 3>{220, 128, 128},
          std::array<std::uint8_t, 3>{180, 90, 135},
          std::array<std::uint8_t, 3>{160, 150, 205}}) {
        std::vector<std::uint8_t> frame(80U * 160U, 0U);
        for (int row = 0; row < 80; ++row) {
            for (int pair = 0; pair < 40; ++pair) {
                SetYuyvPair(frame, row, pair, yuv[0], yuv[1], yuv[0], yuv[2]);
            }
        }
        const auto rectangle = ls2k::vision::ml::DetectRedRectangle(
            View(frame), MakeProjector(), DetectorParameters());
        Expect(!rectangle.valid, "non-red frame must not produce a rectangle");
    }

    std::vector<std::uint8_t> small_red(80U * 160U, 0U);
    for (int row = 0; row < 80; ++row) {
        for (int pair = 0; pair < 40; ++pair) {
            SetYuyvPair(small_red, row, pair, 220, 128, 220, 128);
        }
    }
    PaintMarker(small_red, 0.38F, 0.0F, 0.0F, 0.03F, 0.01F);
    const auto rectangle = ls2k::vision::ml::DetectRedRectangle(
        View(small_red), MakeProjector(), DetectorParameters());
    Expect(!rectangle.valid, "wrong-size red distractor must not produce a rectangle");
}

void TestRoiTopRowIsFarthestForwardAndAxisCanonicalization() {
    std::vector<std::uint8_t> frame(80U * 160U, 0U);
    for (int row = 0; row < 80; ++row) {
        for (int pair = 0; pair < 40; ++pair) {
            const int col0 = pair * 2;
            SetYuyvPair(frame, row, pair,
                        static_cast<std::uint8_t>(row + col0), 128,
                        static_cast<std::uint8_t>(row + col0 + 1), 128);
        }
    }
    ls2k::port::MlOrientedRectangle marker{};
    marker.valid = true;
    marker.center = {0.20F, 0.0F};
    marker.long_edge_m = 0.30F;
    marker.short_edge_m = 0.10F;
    marker.long_axis_forward = 0.0F;
    marker.long_axis_lateral = 1.0F;
    const auto params = DetectorParameters();
    const auto first = ls2k::vision::ml::SampleSquareRoi32(
        View(frame), MakeProjector(), marker, params);
    Expect(first.valid, "forward square ROI should be sampleable");
    Expect(first.frame_id == 77 && std::string(first.reason) == "ok",
           "ROI observation must retain frame identity and sampling validity");
    const int first_row = first.gray[16];
    const int last_row = first.gray[31 * 32 + 16];
    Expect(first_row > last_row + 20,
           "ROI row 0 must sample farther forward than row 31");
    marker.long_axis_lateral = -1.0F;
    const auto reversed = ls2k::vision::ml::SampleSquareRoi32(
        View(frame), MakeProjector(), marker, params);
    Expect(reversed.valid && reversed.gray == first.gray,
           "long-axis sign must not mirror classifier input");
    marker.long_edge_m = 0.34F;
    marker.short_edge_m = 0.13F;
    const auto quantized_observation = ls2k::vision::ml::SampleSquareRoi32(
        View(frame), MakeProjector(), marker, params);
    Expect(quantized_observation.valid && quantized_observation.gray == first.gray,
           "ROI crop size must use calibrated marker truth, not grid-quantized box edges");
}

void TestObserverPublishesRecognitionWithoutPathInputs() {
    std::vector<std::uint8_t> frame(80U * 160U, 0U);
    for (int row = 0; row < 80; ++row) {
        for (int pair = 0; pair < 40; ++pair) {
            SetYuyvPair(frame, row, pair, 20, 128, 20, 128);
        }
    }
    PaintMarker(frame, 0.25F, 0.0F, 0.25F);
    const auto view = View(frame);
    const auto projector = MakeProjector();
    FakeClassifier classifier{};
    ls2k::port::MlParameters params{};
    params.enabled = true;
    params.roi = DetectorParameters();
    ls2k::vision::ml::MlRedRectangleProjectionLut lut{};
    const ls2k::vision::ml::MlObservation observation =
        ls2k::vision::ml::RunMlObserver(
            {&view, &projector, &lut, &classifier}, params);
    Expect(observation.accepted && observation.detector_valid &&
               observation.roi.valid && observation.classification.valid,
           "observer must publish the complete recognition chain");
    Expect(observation.classification.class_id == 1 &&
               observation.mapped_action == ls2k::port::MlAction::kLeft,
           "observer must publish raw class and mapped action facts");
    Expect(std::string(observation.artifact_candidate_id) == "fake-artifact" &&
               std::string(observation.artifact_sha256) == "fake-sha",
           "observer must preserve classifier identity facts");
}

}  // namespace

int main() {
    try {
        TestTiltedDetectionAndTieBreak();
        TestPositiveAndNegativeFortyFiveDegreeDetection();
        TestExpandedRedRangeRejectsNonRedAndWrongSizeDistractors();
        TestRoiTopRowIsFarthestForwardAndAxisCanonicalization();
        TestObserverPublishesRecognitionWithoutPathInputs();
    } catch (const std::exception& error) {
        std::cerr << "ml_rectangle_roi_test failed: " << error.what() << '\n';
        return 1;
    }
    std::cout << "ml_rectangle_roi_test passed\n";
    return 0;
}
