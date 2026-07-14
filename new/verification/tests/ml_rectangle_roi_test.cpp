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

namespace {

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
                 float center_lateral, float angle) {
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
                red = red || (std::fabs(along_long) <= 0.15F &&
                              std::fabs(along_short) <= 0.05F);
            }
            if (red) SetYuyvPair(frame, row, pair, 100, 50, 100, 200);
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
    params.red_y_min = 90; params.red_y_max = 110;
    params.red_u_min = 40; params.red_u_max = 60;
    params.red_v_min = 190; params.red_v_max = 210;
    params.expected_long_edge_m = 0.30;
    params.expected_short_edge_m = 0.10;
    params.long_edge_tolerance_m = 0.04;
    params.short_edge_tolerance_m = 0.04;
    params.max_long_edge_to_lateral_rad = 0.35;
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
    Expect(rectangle.center.forward_m < 0.4F,
           "equal-quality markers must select the nearer forward target");
}

void TestRoiForwardDirectionAndAxisCanonicalization() {
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
    const auto first = ls2k::vision::ml::SampleSquareRoi32(
        View(frame), MakeProjector(), marker);
    Expect(first.valid, "forward square ROI should be sampleable");
    Expect(first.frame_id == 77 && std::string(first.reason) == "ok",
           "ROI observation must retain frame identity and sampling validity");
    const int first_row = first.gray[16];
    const int last_row = first.gray[31 * 32 + 16];
    Expect(last_row > first_row + 20, "ROI rows must extend along vehicle forward");
    marker.long_axis_lateral = -1.0F;
    const auto reversed = ls2k::vision::ml::SampleSquareRoi32(
        View(frame), MakeProjector(), marker);
    Expect(reversed.valid && reversed.gray == first.gray,
           "long-axis sign must not mirror classifier input");
}

}  // namespace

int main() {
    try {
        TestTiltedDetectionAndTieBreak();
        TestRoiForwardDirectionAndAxisCanonicalization();
    } catch (const std::exception& error) {
        std::cerr << "ml_rectangle_roi_test failed: " << error.what() << '\n';
        return 1;
    }
    std::cout << "ml_rectangle_roi_test passed\n";
    return 0;
}
