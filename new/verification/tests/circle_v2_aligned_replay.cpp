#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "platform/bootstrap.hpp"
#include "vision/bev/bev_simple_perception.hpp"
#include "vision/elements/circle_v2/circle_v2_scene.hpp"
#include "vision/image/otsu_threshold.hpp"

namespace {

struct Diagnostics final : ls2k::port::DiagnosticSink {
    void Emit(const ls2k::port::DiagnosticEvent&) override {}
};

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::vector<std::uint8_t> ReadGray8(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    Require(input.is_open(), "cannot open frame: " + path.string());
    const std::streamsize size = input.tellg();
    Require(size == 320 * 240, "frame is not aligned 320x240 gray8: " + path.string());
    input.seekg(0);
    std::vector<std::uint8_t> gray(static_cast<std::size_t>(size));
    input.read(reinterpret_cast<char*>(gray.data()), size);
    Require(input.gcount() == size, "truncated frame: " + path.string());
    return gray;
}

std::vector<std::uint8_t> GrayToYuyv(const std::vector<std::uint8_t>& gray) {
    std::vector<std::uint8_t> yuyv(gray.size() * 2U, 128U);
    for (std::size_t index = 0; index < gray.size(); ++index) {
        yuyv[index * 2U] = gray[index];
    }
    return yuyv;
}

ls2k::vision::CircleV2Params BuildCircleParams(
    const ls2k::port::RuntimeParameters& params) {
    constexpr float kPi = 3.14159265358979323846F;
    ls2k::vision::CircleV2Params circle{};
    circle.exit_yaw_threshold_rad =
        params.bev_element.circle_v2_exit_yaw_threshold_deg * kPi / 180.0F;
    circle.exit_hold_frames = params.bev_element.circle_v2_exit_hold_frames;
    circle.inner_trace_stall_timeout_ms =
        params.bev_element.circle_v2_inner_trace_stall_timeout_ms;
    circle.inner_trace_stall_yaw_min_rad =
        params.bev_element.circle_v2_inner_trace_stall_yaw_min_deg * kPi / 180.0F;
    circle.inner_trace_path_offset_m =
        params.bev_element.circle_v2_inner_trace_path_offset_m;
    circle.opposite_straight_confidence_min =
        params.bev_element.circle_v2_opposite_straight_confidence_min;
    circle.max_adjacent_distance_m =
        params.bev_geometry.boundary_trace_max_adjacent_distance_m;
    circle.min_sampleable_width_m =
        params.bev_element.circle_v2_min_sampleable_width_m;
    circle.opening_forward_min_m =
        params.bev_element.circle_v2_opening_forward_min_m;
    circle.opening_forward_max_m =
        params.bev_element.circle_v2_opening_forward_max_m;
    circle.opening_distance_min_m =
        params.bev_element.circle_v2_opening_distance_min_m;
    circle.opening_confirm_forward_span_m =
        params.bev_element.circle_v2_opening_confirm_forward_span_m;
    circle.entry_forward_min_m = params.bev_element.circle_v2_entry_forward_min_m;
    circle.entry_forward_max_m = params.bev_element.circle_v2_entry_forward_max_m;
    circle.inner_geometry_forward_min_m =
        params.bev_element.circle_v2_inner_geometry_forward_min_m;
    circle.inner_geometry_forward_max_m =
        params.bev_element.circle_v2_inner_geometry_forward_max_m;
    circle.exit_geometry_forward_min_m =
        params.bev_element.circle_v2_exit_geometry_forward_min_m;
    circle.exit_geometry_forward_max_m =
        params.bev_element.circle_v2_exit_geometry_forward_max_m;
    circle.exit_straight_max_lateral_span_m =
        params.bev_element.circle_v2_exit_straight_max_lateral_span_m;
    return circle;
}

void WriteOpening(std::ostream& out,
                  const ls2k::vision::CircleOpeningObservation& opening) {
    out << "{\"available\":" << (opening.available ? "true" : "false")
        << ",\"frontier_forward_m\":" << opening.frontier_forward_m
        << ",\"effective_lateral_m\":" << opening.effective_lateral_m
        << ",\"source\":\"" << ls2k::vision::ToString(opening.source) << "\""
        << ",\"outward_distance_m\":" << opening.outward_distance_m
        << ",\"confirmed_forward_span_m\":" << opening.confirmed_forward_span_m
        << ",\"origin_connected\":" << (opening.origin_connected ? "true" : "false")
        << ",\"opposite_straight\":" << (opening.opposite_straight ? "true" : "false")
        << '}';
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Require(argc == 4, "usage: replay FRAMES_DIR PARAMS EVIDENCE_DIR");
        std::vector<std::filesystem::path> frames;
        for (const auto& entry : std::filesystem::directory_iterator(argv[1])) {
            if (entry.is_regular_file() && entry.path().extension() == ".raw") {
                frames.push_back(entry.path());
            }
        }
        std::sort(frames.begin(), frames.end());
        Require(frames.size() == 191U,
                "expected 191 aligned gray8 frames, got " + std::to_string(frames.size()));

        Diagnostics diagnostics{};
        std::unique_ptr<ls2k::port::IParamStore> store = ls2k::platform::MakeParamStore();
        ls2k::port::RuntimeParameters params{};
        Require(store && store->LoadRuntimeParameters(argv[2], params, diagnostics),
                "failed to load production runtime parameters");
        ls2k::vision::BEVProjector projector{};
        Require(projector.Configure(params.bev_projector), "projector configuration failed");
        const ls2k::vision::CircleV2Params circle_params = BuildCircleParams(params);

        std::filesystem::create_directories(argv[3]);
        std::ofstream evidence(std::filesystem::path(argv[3]) / "opening_evidence.jsonl");
        Require(evidence.is_open(), "cannot create opening evidence");
        evidence << std::setprecision(9);

        ls2k::vision::CircleV2Memory memory{};
        ls2k::vision::OtsuThresholdTracker otsu_tracker{};
        ls2k::vision::BEVSampleProjectionLut lut{};
        bool saw_approach = false;
        bool saw_inner_trace = false;
        std::size_t current_threshold_frames = 0U;

        for (std::size_t index = 0; index < frames.size(); ++index) {
            const std::vector<std::uint8_t> gray = ReadGray8(frames[index]);
            std::vector<std::uint8_t> yuyv = GrayToYuyv(gray);
            ls2k::port::CameraPixelFrameView frame{};
            frame.valid = true;
            frame.format = ls2k::port::CameraFrameFormat::kYuyv;
            frame.data = yuyv.data();
            frame.width = 320;
            frame.height = 240;
            frame.stride = 640;
            const ls2k::port::OtsuThresholdState otsu =
                otsu_tracker.Update(ls2k::vision::ComputeSparseOtsuThreshold(frame));
            current_threshold_frames +=
                otsu.source == ls2k::port::OtsuThresholdSource::kCurrent ? 1U : 0U;
            const ls2k::vision::BEVSimplePerceptionResult perception =
                ls2k::vision::RunBEVSimplePerception(frame, otsu, params, projector, &lut);

            ls2k::vision::OrdinaryRoadModel road{};
            road.center_path = perception.reference_path;
            road.half_width.value_m = params.bev_geometry.nominal_road_half_width_m;
            ls2k::vision::SceneFrameView scene{};
            scene.rows.rows = {perception.rows.data(), perception.rows.size()};
            scene.ordinary_road = road;
            scene.stamp.capture_time_ms = static_cast<std::uint64_t>(index * 17U);

            const ls2k::vision::CircleV2StepResult result =
                ls2k::vision::CircleV2Scene{}.Step(scene, memory, circle_params);
            saw_approach = saw_approach ||
                           result.next_memory.phase == ls2k::vision::CirclePhase::kApproach;
            saw_inner_trace = saw_inner_trace ||
                              result.next_memory.phase == ls2k::vision::CirclePhase::kInnerTrace;

            evidence << "{\"index\":" << index << ",\"frame\":\""
                     << frames[index].filename().string() << "\",\"otsu\":{\"valid\":"
                     << (otsu.valid ? "true" : "false") << ",\"threshold\":"
                     << otsu.threshold << "},\"frame_phase\":\""
                     << ls2k::vision::ToString(result.telemetry.frame_phase)
                     << "\",\"next_phase\":\""
                     << ls2k::vision::ToString(result.next_memory.phase)
                     << "\",\"dir\":\"" << ls2k::vision::ToString(result.next_memory.dir)
                     << "\",\"openings\":{\"left\":";
            WriteOpening(evidence, result.telemetry.openings.left);
            evidence << ",\"right\":";
            WriteOpening(evidence, result.telemetry.openings.right);
            evidence << "}}\n";
            memory = result.next_memory;
        }

        std::ofstream summary(std::filesystem::path(argv[3]) / "summary.json");
        Require(summary.is_open(), "cannot create replay summary");
        const bool passed = saw_approach && saw_inner_trace;
        summary << "{\n  \"result\": \"" << (passed ? "PASS" : "FAIL")
                << "\",\n  \"frame_count\": " << frames.size()
                << ",\n  \"current_threshold_frames\": " << current_threshold_frames
                << ",\n  \"saw_approach\": " << (saw_approach ? "true" : "false")
                << ",\n  \"saw_inner_trace\": "
                << (saw_inner_trace ? "true" : "false") << "\n}\n";
        Require(passed, "aligned replay did not reach Idle -> Approach -> InnerTrace");
        std::cout << "circle_v2_aligned_replay passed frames=" << frames.size() << '\n';
    } catch (const std::exception& error) {
        std::cerr << "circle_v2_aligned_replay failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
