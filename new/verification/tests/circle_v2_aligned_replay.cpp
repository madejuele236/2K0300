#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "platform/bootstrap.hpp"
#include "vision/bev/bev_simple_perception.hpp"
#include "vision/elements/circle_v2/circle_v2_scene.hpp"
#include "vision/image/illumination_binary_model.hpp"

namespace {

struct Diagnostics final : ls2k::port::DiagnosticSink {
    void Emit(const ls2k::port::DiagnosticEvent&) override {}
};

enum class ReplayMode {
    kNormal,
    kForceExitLeft,
    kForceExitRight,
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
    circle.normal_trace_start_yaw_rad =
        params.bev_element.circle_v2_normal_trace_start_yaw_deg * kPi / 180.0F;
    circle.exit_trace_start_yaw_rad =
        params.bev_element.circle_v2_exit_trace_start_yaw_deg * kPi / 180.0F;
    circle.calm_fallback_yaw_rad =
        params.bev_element.circle_v2_calm_fallback_yaw_deg * kPi / 180.0F;
    circle.calm_trace_ms = params.bev_element.circle_v2_calm_trace_ms;
    circle.cooldown_ms = params.bev_element.circle_v2_cooldown_ms;
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
    circle.nominal_road_width_m =
        2.0F * params.bev_geometry.nominal_road_half_width_m;
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
        << ",\"begin_forward_m\":" << opening.begin_forward_m
        << ",\"end_forward_m\":" << opening.end_forward_m
        << ",\"effective_lateral_m\":" << opening.effective_lateral_m
        << ",\"source\":\"" << ls2k::vision::ToString(opening.source) << "\""
        << ",\"outward_distance_m\":" << opening.outward_distance_m
        << ",\"minimum_white_width_m\":" << opening.minimum_white_width_m
        << ",\"origin_connected\":" << (opening.origin_connected ? "true" : "false")
        << '}';
}

const char* EndpointToken(ls2k::vision::BEVWhiteRunEndpointState state) {
    switch (state) {
        case ls2k::vision::BEVWhiteRunEndpointState::kBoundary:
            return "boundary";
        case ls2k::vision::BEVWhiteRunEndpointState::kFovEdge:
            return "fov_edge";
        case ls2k::vision::BEVWhiteRunEndpointState::kUnavailableGap:
            return "unobservable_gap";
    }
    return "unobservable_gap";
}

void WriteRows(std::ostream& out,
               const std::vector<ls2k::vision::BEVSimpleRowScan>& rows) {
    out << '[';
    for (std::size_t row_index = 0; row_index < rows.size(); ++row_index) {
        if (row_index > 0U) {
            out << ',';
        }
        const auto& row = rows[row_index];
        out << "{\"forward_m\":" << row.forward_m << ",\"runs\":[";
        for (std::size_t run_index = 0; run_index < row.white_runs.size(); ++run_index) {
            if (run_index > 0U) {
                out << ',';
            }
            const auto& run = row.white_runs[run_index];
            out << "{\"left_m\":" << run.left_m
                << ",\"right_m\":" << run.right_m
                << ",\"left_endpoint\":\"" << EndpointToken(run.left_endpoint)
                << "\",\"right_endpoint\":\"" << EndpointToken(run.right_endpoint)
                << "\",\"origin_connected\":"
                << (run.origin_connectivity ==
                            ls2k::vision::BEVWhiteRunOriginConnectivity::kConnected
                        ? "true"
                        : "false") << '}';
        }
        out << "]}";
    }
    out << ']';
}

void WriteReferencePath(
    std::ostream& out,
    const std::optional<ls2k::vision::CircleV2ReferencePlan>& plan) {
    out << '[';
    if (plan.has_value()) {
        bool first = true;
        for (const auto& sample : plan->reference_path.sampled_path) {
            if (!sample.present) {
                continue;
            }
            if (!first) {
                out << ',';
            }
            first = false;
            out << "{\"forward_m\":" << sample.point.forward_m
                << ",\"lateral_m\":" << sample.point.lateral_m << '}';
        }
    }
    out << ']';
}

std::optional<cv::Point> ProjectPathPoint(
    const ls2k::vision::BEVProjector& projector,
    const ls2k::port::BEVPoint& point) {
    ls2k::port::ImagePoint image_point{};
    if (!projector.ProjectVehicleToImage(point, image_point)) {
        return std::nullopt;
    }
    const int row = static_cast<int>(std::lround(image_point.row_px));
    const int col = static_cast<int>(std::lround(image_point.col_px));
    if (row < 0 || row >= 240 || col < 0 || col >= 320) {
        return std::nullopt;
    }
    return cv::Point(col, row);
}

std::size_t RenderReferencePath(
    const std::filesystem::path& output_path,
    const std::vector<std::uint8_t>& gray,
    const ls2k::vision::BEVProjector& projector,
    const ls2k::vision::CircleV2StepResult& result) {
    cv::Mat gray_image(240,
                       320,
                       CV_8UC1,
                       const_cast<std::uint8_t*>(gray.data()));
    cv::Mat canvas{};
    cv::cvtColor(gray_image, canvas, cv::COLOR_GRAY2BGR);

    std::size_t sample_count = 0U;
    std::optional<cv::Point> previous{};
    if (result.reference_plan.has_value()) {
        for (const auto& sample :
             result.reference_plan->reference_path.sampled_path) {
            if (!sample.present) {
                previous.reset();
                continue;
            }
            ++sample_count;
            const std::optional<cv::Point> current =
                ProjectPathPoint(projector, sample.point);
            if (!current.has_value()) {
                previous.reset();
                continue;
            }
            if (previous.has_value()) {
                cv::line(canvas,
                         *previous,
                         *current,
                         cv::Scalar(0, 255, 0),
                         3,
                         cv::LINE_AA);
            }
            cv::circle(canvas,
                       *current,
                       3,
                       cv::Scalar(0, 0, 255),
                       cv::FILLED,
                       cv::LINE_AA);
            previous = current;
        }
    }

    const std::string status =
        "phase=" + std::string(ls2k::vision::ToString(result.telemetry.frame_phase)) +
        " dir=" + ls2k::vision::ToString(result.telemetry.dir) +
        " source=" +
        ls2k::port::CircleV2GeometrySourceToken(
            result.telemetry.geometry_source) +
        " samples=" + std::to_string(sample_count);
    cv::rectangle(canvas,
                  cv::Point(0, 0),
                  cv::Point(319, 20),
                  cv::Scalar(0, 0, 0),
                  cv::FILLED);
    cv::putText(canvas,
                status,
                cv::Point(4, 14),
                cv::FONT_HERSHEY_SIMPLEX,
                0.34,
                cv::Scalar(255, 255, 255),
                1,
                cv::LINE_AA);
    Require(cv::imwrite(output_path.string(), canvas),
            "cannot write replay image: " + output_path.string());
    return sample_count;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Require(argc == 4 || argc == 5,
                "usage: replay FRAME_OR_DIR PARAMS EVIDENCE_DIR "
                "[--force-exit-left|--force-exit-right]");
        ReplayMode mode = ReplayMode::kNormal;
        if (argc == 5) {
            const std::string mode_token = argv[4];
            if (mode_token == "--force-exit-left") {
                mode = ReplayMode::kForceExitLeft;
            } else if (mode_token == "--force-exit-right") {
                mode = ReplayMode::kForceExitRight;
            } else {
                Require(false, "unknown replay mode");
            }
        }
        std::vector<std::filesystem::path> frames;
        const std::filesystem::path input_path = argv[1];
        if (std::filesystem::is_regular_file(input_path)) {
            Require(input_path.extension() == ".raw", "input frame must be .raw");
            frames.push_back(input_path);
        } else {
            for (const auto& entry :
                 std::filesystem::directory_iterator(input_path)) {
                if (entry.is_regular_file() &&
                    entry.path().extension() == ".raw") {
                    frames.push_back(entry.path());
                }
            }
        }
        std::sort(frames.begin(), frames.end());
        Require(!frames.empty(), "no aligned gray8 frames found");

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
        if (mode == ReplayMode::kForceExitLeft) {
            memory.phase = ls2k::vision::CirclePhase::kExitTrace;
            memory.dir = ls2k::vision::CircleDir::kLeft;
        } else if (mode == ReplayMode::kForceExitRight) {
            memory.phase = ls2k::vision::CirclePhase::kExitTrace;
            memory.dir = ls2k::vision::CircleDir::kRight;
        }
        ls2k::vision::BinaryModelTracker binary_model_tracker{};
        ls2k::vision::BEVSampleProjectionLut lut{};
        bool saw_approach = false;
        bool saw_inner_trace = false;
        std::size_t current_threshold_frames = 0U;
        std::size_t fixed_exit_ray_frames = 0U;
        std::size_t reference_frames = 0U;

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
            const ls2k::port::BinaryModelState binary_model =
                binary_model_tracker.Update(
                    ls2k::vision::ComputeIlluminationBinaryModel(frame));
            current_threshold_frames +=
                binary_model.source == ls2k::port::BinaryModelSource::kCurrent
                    ? 1U
                    : 0U;
            const ls2k::vision::BEVSimplePerceptionResult perception =
                ls2k::vision::RunBEVSimplePerception(
                    frame, binary_model, params, projector, &lut);

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
            fixed_exit_ray_frames +=
                result.telemetry.geometry_source ==
                        ls2k::vision::CircleV2GeometrySource::kFixedExitRay
                    ? 1U
                    : 0U;
            reference_frames += result.reference_plan.has_value() ? 1U : 0U;
            const std::filesystem::path image_path =
                std::filesystem::path(argv[3]) /
                (frames[index].stem().string() + ".png");
            const std::size_t rendered_sample_count =
                RenderReferencePath(image_path, gray, projector, result);

            evidence << "{\"index\":" << index << ",\"frame\":\""
                     << frames[index].filename().string()
                     << "\",\"binary_model\":{\"valid\":"
                     << (binary_model.valid ? "true" : "false")
                     << ",\"residual_threshold\":"
                     << binary_model.residual_threshold
                     << "},\"frame_phase\":\""
                     << ls2k::vision::ToString(result.telemetry.frame_phase)
                     << "\",\"next_phase\":\""
                     << ls2k::vision::ToString(result.next_memory.phase)
                     << "\",\"geometry_source\":\""
                     << ls2k::port::CircleV2GeometrySourceToken(
                            result.telemetry.geometry_source)
                     << "\",\"dir\":\"" << ls2k::vision::ToString(result.next_memory.dir)
                     << "\",\"geometry_available\":"
                     << (result.telemetry.geometry_available ? "true" : "false")
                     << ",\"rendered_sample_count\":"
                     << rendered_sample_count
                     << ",\"reference_path\":";
            WriteReferencePath(evidence, result.reference_plan);
            evidence << ",\"openings\":{\"left\":";
            WriteOpening(evidence, result.telemetry.openings.left);
            evidence << ",\"right\":";
            WriteOpening(evidence, result.telemetry.openings.right);
            evidence << "},\"entry_cue\":{\"detected_dir\":\""
                     << ls2k::vision::ToString(
                            result.telemetry.entry_cue.detected_dir)
                     << "\",\"bilateral_overlap\":"
                     << (result.telemetry.entry_cue.bilateral_overlap
                             ? "true"
                             : "false")
                     << ",\"selected\":"
                     << (result.telemetry.entry_cue.selected ? "true" : "false")
                     << ",\"selected_begin_forward_m\":"
                     << result.telemetry.entry_cue.selected_begin_forward_m
                     << ",\"selected_end_forward_m\":"
                     << result.telemetry.entry_cue.selected_end_forward_m
                     << ",\"opposite_observable\":"
                     << (result.telemetry.entry_cue.opposite_observable
                             ? "true"
                             : "false")
                     << ",\"opposite_straight\":"
                     << (result.telemetry.entry_cue.opposite_straight
                             ? "true"
                             : "false")
                     << ",\"opposite_straight_confidence\":"
                     << result.telemetry.entry_cue
                            .opposite_straight_confidence
                     << "},\"rows\":";
            WriteRows(evidence, perception.rows);
            evidence << "}\n";
            memory = result.next_memory;
        }

        std::ofstream summary(std::filesystem::path(argv[3]) / "summary.json");
        Require(summary.is_open(), "cannot create replay summary");
        const bool forced_exit =
            mode == ReplayMode::kForceExitLeft ||
            mode == ReplayMode::kForceExitRight;
        const bool passed = forced_exit ? true : saw_approach && saw_inner_trace;
        summary << "{\n  \"result\": \"" << (passed ? "PASS" : "FAIL")
                << "\",\n  \"frame_count\": " << frames.size()
                << ",\n  \"current_threshold_frames\": " << current_threshold_frames
                << ",\n  \"fixed_exit_ray_frames\": " << fixed_exit_ray_frames
                << ",\n  \"reference_frames\": " << reference_frames
                << ",\n  \"saw_approach\": " << (saw_approach ? "true" : "false")
                << ",\n  \"saw_inner_trace\": "
                << (saw_inner_trace ? "true" : "false") << "\n}\n";
        Require(passed,
                "aligned replay did not reach Idle -> Approach -> InnerTrace");
        std::cout << "circle_v2_aligned_replay passed frames=" << frames.size() << '\n';
    } catch (const std::exception& error) {
        std::cerr << "circle_v2_aligned_replay failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
