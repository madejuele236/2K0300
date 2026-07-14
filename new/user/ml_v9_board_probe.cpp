#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include <opencv2/core/persistence.hpp>

#include "generated_v9_artifact.hpp"
#include "platform/true_ls2k0300/camera_device.hpp"
#include "port/camera_frame_types.hpp"
#include "port/runtime_parameter_types.hpp"
#include "vision/bev/bev_projector.hpp"
#include "vision/ml/red_rectangle_detector.hpp"
#include "vision/ml/roi_sampler.hpp"
#include "vision/ml/v9_descriptor.hpp"
#include "vision/ml/v9_replay.hpp"

namespace {

using Clock = std::chrono::steady_clock;

struct ProbeConfig {
    ls2k::port::BEVProjectorCalibration projector{};
    ls2k::port::MlRoiParameters roi{};
};

template <typename Duration = std::chrono::microseconds>
std::uint64_t Elapsed(Clock::time_point begin, Clock::time_point end) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<Duration>(end - begin).count());
}

bool ParseNonNegativeInt(std::string_view text, int& value) {
    if (text.empty()) return false;
    int parsed = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size() || parsed < 0) {
        return false;
    }
    value = parsed;
    return true;
}

bool ReadDouble(const cv::FileNode& parent, const char* key, double& value) {
    const cv::FileNode node = parent[key];
    if (node.empty() || (!node.isInt() && !node.isReal())) return false;
    value = node.real();
    return std::isfinite(value);
}

bool ReadFloat(const cv::FileNode& parent, const char* key, float& value) {
    double parsed = 0.0;
    if (!ReadDouble(parent, key, parsed) ||
        parsed < -static_cast<double>(std::numeric_limits<float>::max()) ||
        parsed > static_cast<double>(std::numeric_limits<float>::max())) {
        return false;
    }
    value = static_cast<float>(parsed);
    return true;
}

bool ReadInt(const cv::FileNode& parent, const char* key, int& value) {
    const cv::FileNode node = parent[key];
    if (node.empty() || !node.isInt()) return false;
    const double parsed = node.real();
    if (parsed < static_cast<double>(std::numeric_limits<int>::min()) ||
        parsed > static_cast<double>(std::numeric_limits<int>::max())) {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

bool ReadBool(const cv::FileNode& parent, const char* key, bool& value) {
    int parsed = 0;
    if (!ReadInt(parent, key, parsed) || (parsed != 0 && parsed != 1)) return false;
    value = parsed != 0;
    return true;
}

bool ReadString(const cv::FileNode& parent, const char* key, std::string& value) {
    const cv::FileNode node = parent[key];
    if (node.empty() || !node.isString()) return false;
    value = node.string();
    return !value.empty();
}

bool LoadProbeConfig(const std::string& path, ProbeConfig& out, std::string& reason) {
    cv::FileStorage storage;
    try {
        if (!storage.open(path, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON)) {
            reason = "open_failed";
            return false;
        }
    } catch (...) {
        reason = "parse_failed";
        return false;
    }

    const cv::FileNode root = storage.root();
    const cv::FileNode projector = root["BEV_PROJECTOR"];
    const cv::FileNode ml = root["ML"];
    const cv::FileNode roi = ml["ROI"];
    if (root.empty() || !root.isMap() || projector.empty() || !projector.isMap() ||
        ml.empty() || !ml.isMap() || roi.empty() || !roi.isMap()) {
        reason = "missing_BEV_PROJECTOR_or_ML_ROI";
        return false;
    }

    bool ok = ReadBool(projector, "VALID", out.projector.valid) &&
              ReadString(projector, "PROJECTOR_ID", out.projector.projector_id) &&
              ReadString(projector, "PROJECTOR_HASH", out.projector.projector_hash);
    for (std::size_t index = 0; index < ls2k::port::kBevCalibrationPointCount; ++index) {
        const std::string suffix = std::to_string(index);
        ok = ReadFloat(projector, ("SOURCE_ROW_" + suffix).c_str(),
                       out.projector.source_points[index].row_px) && ok;
        ok = ReadFloat(projector, ("SOURCE_COL_" + suffix).c_str(),
                       out.projector.source_points[index].col_px) && ok;
        ok = ReadFloat(projector, ("TARGET_FORWARD_" + suffix).c_str(),
                       out.projector.target_points[index].forward_m) && ok;
        ok = ReadFloat(projector, ("TARGET_LATERAL_" + suffix).c_str(),
                       out.projector.target_points[index].lateral_m) && ok;
    }

    ok = ReadDouble(roi, "SEARCH_FORWARD_MIN_M", out.roi.search_forward_min_m) && ok;
    ok = ReadDouble(roi, "SEARCH_FORWARD_MAX_M", out.roi.search_forward_max_m) && ok;
    ok = ReadDouble(roi, "SEARCH_LATERAL_LIMIT_M", out.roi.search_lateral_limit_m) && ok;
    ok = ReadDouble(roi, "GRID_FORWARD_STEP_M", out.roi.grid_forward_step_m) && ok;
    ok = ReadDouble(roi, "GRID_LATERAL_STEP_M", out.roi.grid_lateral_step_m) && ok;
    ok = ReadInt(roi, "RED_Y_MIN", out.roi.red_y_min) && ok;
    ok = ReadInt(roi, "RED_Y_MAX", out.roi.red_y_max) && ok;
    ok = ReadInt(roi, "RED_U_MIN", out.roi.red_u_min) && ok;
    ok = ReadInt(roi, "RED_U_MAX", out.roi.red_u_max) && ok;
    ok = ReadInt(roi, "RED_V_MIN", out.roi.red_v_min) && ok;
    ok = ReadInt(roi, "RED_V_MAX", out.roi.red_v_max) && ok;
    ok = ReadDouble(roi, "EXPECTED_LONG_EDGE_M", out.roi.expected_long_edge_m) && ok;
    ok = ReadDouble(roi, "EXPECTED_SHORT_EDGE_M", out.roi.expected_short_edge_m) && ok;
    ok = ReadDouble(roi, "LONG_EDGE_TOLERANCE_M", out.roi.long_edge_tolerance_m) && ok;
    ok = ReadDouble(roi, "SHORT_EDGE_TOLERANCE_M", out.roi.short_edge_tolerance_m) && ok;
    ok = ReadDouble(roi, "MAX_LONG_EDGE_TO_LATERAL_RAD",
                    out.roi.max_long_edge_to_lateral_rad) && ok;
    ok = ReadInt(roi, "MIN_COMPONENT_CELLS", out.roi.min_component_cells) && ok;
    ok = ReadDouble(roi, "MIN_RECTANGULARITY", out.roi.min_rectangularity) && ok;
    ok = ReadDouble(roi, "MIN_RED_FILL_RATIO", out.roi.min_red_fill_ratio) && ok;
    ok = ReadDouble(roi, "SCORE_SIZE_WEIGHT", out.roi.score_size_weight) && ok;
    ok = ReadDouble(roi, "SCORE_RECTANGULARITY_WEIGHT",
                    out.roi.score_rectangularity_weight) && ok;
    ok = ReadDouble(roi, "SCORE_RED_FILL_WEIGHT", out.roi.score_red_fill_weight) && ok;
    ok = ReadDouble(roi, "SCORE_ORIENTATION_WEIGHT",
                    out.roi.score_orientation_weight) && ok;
    if (!ok) {
        reason = "missing_or_malformed_probe_field";
        return false;
    }
    if (!out.projector.valid) {
        reason = "projector_marked_invalid";
        return false;
    }
    return true;
}

bool SaveRoi(const std::string& path, const ls2k::port::MlGrayRoi32& roi) {
    if (!roi.valid) return false;
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output) return false;
    output.write(reinterpret_cast<const char*>(roi.gray.data()),
                 static_cast<std::streamsize>(roi.gray.size()));
    output.close();
    return static_cast<bool>(output);
}

void PrintDescriptor(const ls2k::port::V9Descriptor& descriptor) {
    std::cout << "descriptor valid=" << (descriptor.valid ? "true" : "false")
              << " bytes=";
    const char previous_fill = std::cout.fill('0');
    for (std::uint8_t byte : descriptor.bytes) {
        std::cout << std::hex << std::setw(2) << static_cast<unsigned>(byte);
    }
    std::cout << std::dec;
    std::cout.fill(previous_fill);
    std::cout << '\n';
}

bool LoadSavedRoi(const std::string& path, ls2k::port::MlGrayRoi32& roi) {
    std::ifstream input(path, std::ios::binary);
    if (!input) return false;
    input.read(reinterpret_cast<char*>(roi.gray.data()),
               static_cast<std::streamsize>(roi.gray.size()));
    if (input.gcount() != static_cast<std::streamsize>(roi.gray.size())) return false;
    char extra = 0;
    if (input.get(extra)) return false;
    roi.valid = true;
    roi.reason = "saved_roi";
    return true;
}

bool SameReplay(const ls2k::port::V9ReplayResult& first,
                const ls2k::port::V9ReplayResult& second) {
    return first.valid == second.valid && first.class_id == second.class_id &&
           first.best_distance == second.best_distance && first.margin == second.margin &&
           first.prototype_index == second.prototype_index;
}

int RunSavedRoiBenchmark(int argc,
                         char** argv,
                         const ls2k::port::V9ArtifactView& artifact) {
    constexpr int kWarmupIterations = 16;
    int measured_iterations = 0;
    if (argc < 4 || !ParseNonNegativeInt(argv[2], measured_iterations) ||
        measured_iterations == 0) {
        std::cerr << "usage: " << argv[0]
                  << " --saved-roi MEASURED_ITERATIONS ROI_PATH [ROI_PATH ...]\n";
        return 2;
    }

    std::uint64_t descriptor_total_ns = 0;
    std::uint64_t replay_total_ns = 0;
    std::uint64_t combined_total_ns = 0;
    std::uint64_t measured_samples = 0;
    for (int argument = 3; argument < argc; ++argument) {
        ls2k::port::MlGrayRoi32 roi{};
        if (!LoadSavedRoi(argv[argument], roi)) {
            std::cerr << "saved ROI must contain exactly " << roi.gray.size()
                      << " bytes path=" << argv[argument] << '\n';
            return 10;
        }

        const ls2k::port::V9Descriptor expected_descriptor =
            ls2k::vision::ml::BuildV9Descriptor(roi);
        const ls2k::port::V9ReplayResult expected_replay =
            ls2k::vision::ml::ReplayV9Descriptor(expected_descriptor, artifact);
        if (!expected_descriptor.valid || !expected_replay.valid) {
            std::cerr << "saved ROI production pipeline invalid path=" << argv[argument] << '\n';
            return 11;
        }
        for (int iteration = 0; iteration < kWarmupIterations; ++iteration) {
            const auto descriptor = ls2k::vision::ml::BuildV9Descriptor(roi);
            const auto replay = ls2k::vision::ml::ReplayV9Descriptor(descriptor, artifact);
            if (descriptor.bytes != expected_descriptor.bytes ||
                !SameReplay(replay, expected_replay)) {
                std::cerr << "warmup parity failure path=" << argv[argument]
                          << " iteration=" << iteration << '\n';
                return 12;
            }
        }

        std::uint64_t roi_descriptor_ns = 0;
        std::uint64_t roi_replay_ns = 0;
        std::uint64_t roi_combined_ns = 0;
        for (int iteration = 0; iteration < measured_iterations; ++iteration) {
            const Clock::time_point combined_begin = Clock::now();
            const Clock::time_point descriptor_begin = combined_begin;
            const auto descriptor = ls2k::vision::ml::BuildV9Descriptor(roi);
            const Clock::time_point descriptor_end = Clock::now();
            const auto replay = ls2k::vision::ml::ReplayV9Descriptor(descriptor, artifact);
            const Clock::time_point replay_end = Clock::now();
            if (descriptor.bytes != expected_descriptor.bytes ||
                !SameReplay(replay, expected_replay)) {
                std::cerr << "measured parity failure path=" << argv[argument]
                          << " iteration=" << iteration << '\n';
                return 12;
            }
            roi_descriptor_ns += Elapsed<std::chrono::nanoseconds>(descriptor_begin, descriptor_end);
            roi_replay_ns += Elapsed<std::chrono::nanoseconds>(descriptor_end, replay_end);
            roi_combined_ns += Elapsed<std::chrono::nanoseconds>(combined_begin, replay_end);
        }
        descriptor_total_ns += roi_descriptor_ns;
        replay_total_ns += roi_replay_ns;
        combined_total_ns += roi_combined_ns;
        measured_samples += static_cast<std::uint64_t>(measured_iterations);

        std::cout << "saved_roi path=" << argv[argument] << '\n';
        PrintDescriptor(expected_descriptor);
        std::cout << "replay valid=true raw_class=" << expected_replay.class_id
                  << " best_distance=" << expected_replay.best_distance
                  << " margin=" << expected_replay.margin
                  << " prototype_index=" << expected_replay.prototype_index << '\n';
        std::cout << std::fixed << std::setprecision(3)
                  << "saved_roi_timing descriptor_avg_us="
                  << static_cast<double>(roi_descriptor_ns) /
                         static_cast<double>(measured_iterations) / 1000.0
                  << " replay_avg_us="
                  << static_cast<double>(roi_replay_ns) /
                         static_cast<double>(measured_iterations) / 1000.0
                  << " combined_avg_us="
                  << static_cast<double>(roi_combined_ns) /
                         static_cast<double>(measured_iterations) / 1000.0 << '\n';
    }
    std::cout << std::fixed << std::setprecision(3)
              << "benchmark_summary roi_count=" << (argc - 3)
              << " warmup_iterations=" << kWarmupIterations
              << " measured_iterations=" << measured_iterations
              << " measured_samples=" << measured_samples
              << " descriptor_avg_us="
              << static_cast<double>(descriptor_total_ns) /
                     static_cast<double>(measured_samples) / 1000.0
              << " replay_avg_us="
              << static_cast<double>(replay_total_ns) /
                     static_cast<double>(measured_samples) / 1000.0
              << " combined_avg_us="
              << static_cast<double>(combined_total_ns) /
                     static_cast<double>(measured_samples) / 1000.0
              << " parity=true\n";
    return 0;
}

void PrintUsage(const char* program) {
    std::cerr << "usage: " << program
              << " CONFIG_JSON ROI_OUTPUT_PATH [WARMUP_FRAMES] [DEVICE]\n"
              << "CONFIG_JSON must contain BEV_PROJECTOR and ML.ROI using production field names\n";
    std::cerr << "       " << program
              << " --saved-roi MEASURED_ITERATIONS ROI_PATH [ROI_PATH ...]\n";
}

}  // namespace

int main(int argc, char** argv) {
    namespace platform = ls2k::platform::true_ls2k0300;
    namespace ml = ls2k::vision::ml;

    if (argc >= 2 && std::string_view(argv[1]) == "--saved-roi") {
        const ls2k::port::V9ArtifactView artifact = ml::generated::Artifact();
        if (!ml::ValidateV9Artifact(artifact)) {
            std::cerr << "generated V9 artifact invalid\n";
            return 5;
        }
        return RunSavedRoiBenchmark(argc, argv, artifact);
    }

    if (argc < 3 || argc > 5) {
        PrintUsage(argv[0]);
        return 2;
    }
    int warmup_frames = 3;
    if (argc >= 4 && !ParseNonNegativeInt(argv[3], warmup_frames)) {
        std::cerr << "invalid WARMUP_FRAMES: expected a non-negative integer\n";
        return 2;
    }
    const char* device = argc >= 5 ? argv[4] : "/dev/video0";

    ProbeConfig probe_config{};
    std::string config_reason;
    if (!LoadProbeConfig(argv[1], probe_config, config_reason)) {
        std::cerr << "config invalid path=" << argv[1] << " reason=" << config_reason << '\n';
        return 3;
    }

    ls2k::vision::BEVProjector projector;
    if (!projector.Configure(probe_config.projector)) {
        std::cerr << "projector configure failed id=" << probe_config.projector.projector_id
                  << " hash=" << probe_config.projector.projector_hash << '\n';
        return 4;
    }

    const ls2k::port::V9ArtifactView artifact = ml::generated::Artifact();
    const bool artifact_valid = ml::ValidateV9Artifact(artifact);
    std::cout << "artifact valid=" << (artifact_valid ? "true" : "false")
              << " candidate_id=" << (artifact.candidate_id ? artifact.candidate_id : "null")
              << " descriptor_config_hash="
              << (artifact.descriptor_config_hash ? artifact.descriptor_config_hash : "null")
              << " template_table_hash="
              << (artifact.template_table_hash ? artifact.template_table_hash : "null")
              << " template_codes_sha256="
              << (artifact.template_codes_sha256 ? artifact.template_codes_sha256 : "null")
              << " prototypes=" << artifact.prototype_count
              << " descriptor_bytes=" << artifact.byte_count
              << " descriptor_bits=" << artifact.bit_count << '\n';
    std::cout << "config path=" << argv[1]
              << " projector_id=" << probe_config.projector.projector_id
              << " projector_hash=" << probe_config.projector.projector_hash
              << " motion_parameters_loaded=false\n";
    if (!artifact_valid) return 5;

    platform::CameraConfig camera_config{};
    camera_config.device = device;
    platform::CameraDevice camera;
    if (!camera.Start(camera_config)) {
        std::cerr << "camera start failed device=" << device
                  << " width=" << camera_config.width
                  << " height=" << camera_config.height
                  << " fps=" << camera_config.fps << '\n';
        return 6;
    }
    for (int index = 0; index < warmup_frames; ++index) {
        if (!camera.Capture().valid()) {
            std::cerr << "camera warmup capture failed frame=" << (index + 1) << '\n';
            camera.Stop();
            return 7;
        }
    }

    const Clock::time_point capture_begin = Clock::now();
    const platform::CameraFrameView captured = camera.Capture();
    const Clock::time_point capture_end = Clock::now();
    if (!captured.valid()) {
        std::cerr << "camera probe capture failed after_warmup=" << warmup_frames << '\n';
        camera.Stop();
        return 7;
    }
    const ls2k::port::CameraPixelFrameView frame{
        true,
        ls2k::port::CameraFrameFormat::kYuyv,
        captured.data,
        captured.width,
        captured.height,
        captured.stride,
        1U,
        static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                capture_end.time_since_epoch()).count())};

    const Clock::time_point detector_begin = Clock::now();
    const ls2k::port::MlOrientedRectangle rectangle =
        ml::DetectRedRectangle(frame, projector, probe_config.roi);
    const Clock::time_point detector_end = Clock::now();
    const ls2k::port::MlGrayRoi32 roi = ml::SampleSquareRoi32(frame, projector, rectangle);
    const Clock::time_point roi_end = Clock::now();
    const ls2k::port::V9Descriptor descriptor = ml::BuildV9Descriptor(roi);
    const Clock::time_point descriptor_end = Clock::now();
    const ls2k::port::V9ReplayResult replay = ml::ReplayV9Descriptor(descriptor, artifact);
    const Clock::time_point replay_end = Clock::now();
    camera.Stop();

    const bool roi_saved = SaveRoi(argv[2], roi);
    std::cout << std::fixed << std::setprecision(6)
              << "detector valid=" << (rectangle.valid ? "true" : "false")
              << " frame_id=" << rectangle.frame_id
              << " center_forward_m=" << rectangle.center.forward_m
              << " center_lateral_m=" << rectangle.center.lateral_m
              << " long_edge_m=" << rectangle.long_edge_m
              << " short_edge_m=" << rectangle.short_edge_m
              << " long_axis_forward=" << rectangle.long_axis_forward
              << " long_axis_lateral=" << rectangle.long_axis_lateral
              << " orientation_rad=" << rectangle.long_edge_to_lateral_rad
              << " rectangularity=" << rectangle.rectangularity
              << " red_fill_ratio=" << rectangle.red_fill_ratio
              << " quality=" << rectangle.quality
              << " component_cells=" << rectangle.component_cells << '\n';
    for (std::size_t index = 0; index < rectangle.corners.size(); ++index) {
        std::cout << "detector_corner index=" << index
                  << " forward_m=" << rectangle.corners[index].forward_m
                  << " lateral_m=" << rectangle.corners[index].lateral_m << '\n';
    }
    std::cout << "roi valid=" << (roi.valid ? "true" : "false")
              << " reason=" << (roi.reason ? roi.reason : "null")
              << " frame_id=" << roi.frame_id
              << " bytes=" << roi.gray.size()
              << " saved=" << (roi_saved ? "true" : "false")
              << " path=" << argv[2]
              << " long_axis_forward=" << roi.long_axis_forward
              << " long_axis_lateral=" << roi.long_axis_lateral
              << " forward_normal_forward=" << roi.forward_normal_forward
              << " forward_normal_lateral=" << roi.forward_normal_lateral << '\n';
    PrintDescriptor(descriptor);
    std::cout << "replay valid=" << (replay.valid ? "true" : "false")
              << " raw_class=" << replay.class_id
              << " best_distance=" << replay.best_distance
              << " margin=" << replay.margin
              << " prototype_index=" << replay.prototype_index << '\n';
    std::cout << "timing capture_us=" << Elapsed(capture_begin, capture_end)
              << " detector_us=" << Elapsed(detector_begin, detector_end)
              << " roi_us=" << Elapsed(detector_end, roi_end)
              << " descriptor_us=" << Elapsed(roi_end, descriptor_end)
              << " replay_us=" << Elapsed(descriptor_end, replay_end)
              << " algorithm_total_us=" << Elapsed(detector_begin, replay_end) << '\n';

    if (!rectangle.valid || !roi.valid || !descriptor.valid || !replay.valid) return 8;
    if (!roi_saved) {
        std::cerr << "ROI output write failed path=" << argv[2]
                  << " bytes=" << roi.gray.size() << '\n';
        return 9;
    }
    return 0;
}
