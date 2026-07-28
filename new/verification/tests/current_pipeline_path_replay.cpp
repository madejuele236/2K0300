#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "platform/bootstrap.hpp"
#include "runtime/pipelines/steering_frame_pipeline.hpp"
#include "vision/bev/bev_projector.hpp"

namespace {

constexpr int kWidth = 320;
constexpr int kHeight = 240;

struct Diagnostics final : ls2k::port::DiagnosticSink {
    void Emit(const ls2k::port::DiagnosticEvent&) override {}
};

struct FrameStamp {
    std::uint64_t frame_id = 0U;
    std::uint64_t capture_time_ms = 0U;
};

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::uint64_t ParseUnsignedField(const std::string& line,
                                 const std::string& field) {
    const std::size_t key = line.find(field);
    Require(key != std::string::npos, "metadata field absent: " + field);
    std::size_t first = key + field.size();
    while (first < line.size() && line[first] == ' ') {
        ++first;
    }
    std::size_t last = first;
    while (last < line.size() && line[last] >= '0' && line[last] <= '9') {
        ++last;
    }
    Require(last > first, "metadata field is not unsigned: " + field);
    return std::stoull(line.substr(first, last - first));
}

std::string ParseStringField(const std::string& line,
                             const std::string& field) {
    const std::size_t key = line.find(field);
    Require(key != std::string::npos, "metadata field absent: " + field);
    const std::size_t first = key + field.size();
    const std::size_t last = line.find('"', first);
    Require(last != std::string::npos, "unterminated metadata field: " + field);
    return line.substr(first, last - first);
}

std::map<std::string, FrameStamp> LoadFrameStamps(
    const std::filesystem::path& metadata_path) {
    std::ifstream input(metadata_path);
    Require(input.is_open(), "cannot open metadata: " + metadata_path.string());
    std::map<std::string, FrameStamp> stamps{};
    std::string line{};
    while (std::getline(input, line)) {
        if (line.find("\"type\": \"image_frame\"") == std::string::npos) {
            continue;
        }
        const std::string frame_path =
            ParseStringField(line, "\"frame_path\": \"");
        const std::size_t basename_separator =
            frame_path.find_last_of("\\/");
        const std::string frame_name =
            basename_separator == std::string::npos
                ? frame_path
                : frame_path.substr(basename_separator + 1U);
        const std::uint64_t frame_id =
            ParseUnsignedField(line, "\"frame_id\":");
        const std::uint64_t capture_time_ms =
            ParseUnsignedField(line, "\"capture_time_ms\":");
        // A restarted capture session can reuse a host filename. The file on
        // disk is overwritten, so its matching metadata is the last record.
        stamps[frame_name] = FrameStamp{frame_id, capture_time_ms};
    }
    Require(!stamps.empty(), "metadata contains no frames");
    return stamps;
}

std::vector<std::uint8_t> ReadGray8(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    Require(input.is_open(), "cannot open frame: " + path.string());
    const std::streamsize size = input.tellg();
    Require(size == kWidth * kHeight,
            "frame is not aligned 320x240 gray8: " + path.string());
    input.seekg(0);
    std::vector<std::uint8_t> gray(static_cast<std::size_t>(size));
    input.read(reinterpret_cast<char*>(gray.data()), size);
    Require(input.gcount() == size, "truncated frame: " + path.string());
    return gray;
}

std::vector<std::uint8_t> GrayToNeutralYuyv(
    const std::vector<std::uint8_t>& gray) {
    std::vector<std::uint8_t> yuyv(gray.size() * 2U, 128U);
    for (std::size_t index = 0U; index < gray.size(); ++index) {
        yuyv[index * 2U] = gray[index];
    }
    return yuyv;
}

std::set<std::string> LoadExcludedFrames(
    const std::filesystem::path& path) {
    std::set<std::string> excluded{};
    if (path.empty()) {
        return excluded;
    }
    std::ifstream input(path);
    Require(input.is_open(), "cannot open excluded-frame list: " + path.string());
    std::string frame_name{};
    while (std::getline(input, frame_name)) {
        if (!frame_name.empty()) {
            excluded.insert(frame_name);
        }
    }
    return excluded;
}

ls2k::port::CameraCapture MakeCapture(
    const FrameStamp& stamp,
    const std::vector<std::uint8_t>& gray,
    const std::vector<std::uint8_t>& yuyv) {
    ls2k::port::CameraCapture capture{};
    capture.has_frame = true;
    capture.marker = ls2k::port::CameraGeometryMarker::kPhase1Adapted;
    capture.frame_id = stamp.frame_id;
    capture.capture_time_ms = stamp.capture_time_ms;
    capture.source_width = kWidth;
    capture.source_height = kHeight;
    capture.view.gray = gray.data();
    capture.view.width = kWidth;
    capture.view.height = kHeight;
    capture.view.stride = kWidth;
    capture.view.frame_id = stamp.frame_id;
    capture.view.capture_time_ms = stamp.capture_time_ms;
    capture.pixel_view.valid = true;
    capture.pixel_view.format = ls2k::port::CameraFrameFormat::kYuyv;
    capture.pixel_view.data = yuyv.data();
    capture.pixel_view.width = kWidth;
    capture.pixel_view.height = kHeight;
    capture.pixel_view.stride = kWidth * 2;
    capture.pixel_view.frame_id = stamp.frame_id;
    capture.pixel_view.capture_time_ms = stamp.capture_time_ms;
    return capture;
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
    if (row < 0 || row >= kHeight || col < 0 || col >= kWidth) {
        return std::nullopt;
    }
    return cv::Point(col, row);
}

std::size_t DrawReferencePath(
    cv::Mat& canvas,
    const ls2k::port::BEVReferencePath& path,
    const ls2k::vision::BEVProjector& projector) {
    std::optional<cv::Point> previous{};
    std::size_t sample_count = 0U;
    for (const ls2k::port::BEVPathSample& sample : path.sampled_path) {
        if (!sample.present) {
            previous.reset();
            continue;
        }
        ++sample_count;
        const std::optional<cv::Point> image_point =
            ProjectPathPoint(projector, sample.point);
        if (!image_point.has_value()) {
            previous.reset();
            continue;
        }
        if (previous.has_value()) {
            cv::line(canvas,
                     *previous,
                     *image_point,
                     cv::Scalar(0, 255, 0),
                     3,
                     cv::LINE_AA);
        }
        cv::circle(canvas,
                   *image_point,
                   3,
                   cv::Scalar(0, 0, 255),
                   cv::FILLED,
                   cv::LINE_AA);
        previous = image_point;
    }
    return sample_count;
}

void DrawStatus(cv::Mat& canvas,
                const ls2k::port::PerceptionResult& perception,
                std::size_t sample_count) {
    const std::string first =
        "frame=" + std::to_string(perception.frame_id) +
        " source=" + perception.reference_source +
        " mode=" + perception.reference_mode;
    const std::string second =
        "samples=" + std::to_string(sample_count) +
        " cross=" +
        (perception.element_evidence.cross_exit.present ? "1" : "0") +
        " circle=" + perception.circle_v2.frame_phase +
        " ml=" + (perception.ml.active ? "1" : "0");
    cv::rectangle(canvas,
                  cv::Point(0, 0),
                  cv::Point(kWidth - 1, 35),
                  cv::Scalar(0, 0, 0),
                  cv::FILLED);
    cv::putText(canvas,
                first,
                cv::Point(4, 14),
                cv::FONT_HERSHEY_SIMPLEX,
                0.34,
                cv::Scalar(255, 255, 255),
                1,
                cv::LINE_AA);
    cv::putText(canvas,
                second,
                cv::Point(4, 30),
                cv::FONT_HERSHEY_SIMPLEX,
                0.34,
                cv::Scalar(255, 255, 255),
                1,
                cv::LINE_AA);
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Require(argc == 5 || argc == 6,
                "usage: replay FRAME_DIR FRAME_METADATA.jsonl "
                "PARAMS.json OUTPUT_DIR [EXCLUDED_FRAMES.txt]");
        const std::filesystem::path frame_directory = argv[1];
        const std::filesystem::path metadata_path = argv[2];
        const std::filesystem::path params_path = argv[3];
        const std::filesystem::path output_directory = argv[4];
        const std::filesystem::path excluded_path =
            argc == 6 ? std::filesystem::path(argv[5])
                      : std::filesystem::path{};
        const std::set<std::string> excluded =
            LoadExcludedFrames(excluded_path);

        Diagnostics diagnostics{};
        std::unique_ptr<ls2k::port::IParamStore> store =
            ls2k::platform::MakeParamStore();
        ls2k::port::RuntimeParameters params{};
        Require(store &&
                    store->LoadRuntimeParameters(
                        params_path.string(), params, diagnostics),
                "failed to load current runtime parameters");

        ls2k::runtime::SteeringFramePipeline pipeline{};
        Require(pipeline.Configure(params, diagnostics),
                "current steering frame pipeline configure failed");
        ls2k::vision::BEVProjector projector{};
        Require(projector.Configure(params.bev_projector),
                "render projector configure failed");

        const std::map<std::string, FrameStamp> stamps =
            LoadFrameStamps(metadata_path);
        std::vector<std::filesystem::path> frames{};
        for (const auto& entry :
             std::filesystem::directory_iterator(frame_directory)) {
            if (entry.is_regular_file() &&
                entry.path().extension() == ".raw" &&
                excluded.find(entry.path().filename().string()) ==
                    excluded.end()) {
                frames.push_back(entry.path());
            }
        }
        Require(!frames.empty(), "input directory contains no raw frames");
        for (const std::filesystem::path& frame_path : frames) {
            Require(stamps.find(frame_path.filename().string()) != stamps.end(),
                    "metadata missing " + frame_path.filename().string());
        }
        std::sort(
            frames.begin(),
            frames.end(),
            [&stamps](const std::filesystem::path& left,
                      const std::filesystem::path& right) {
                const FrameStamp& left_stamp =
                    stamps.at(left.filename().string());
                const FrameStamp& right_stamp =
                    stamps.at(right.filename().string());
                if (left_stamp.capture_time_ms != right_stamp.capture_time_ms) {
                    return left_stamp.capture_time_ms <
                           right_stamp.capture_time_ms;
                }
                return left.filename().string() < right.filename().string();
            });
        std::filesystem::create_directories(output_directory);

        std::ofstream manifest(output_directory / "paths.tsv",
                               std::ios::trunc);
        Require(manifest.is_open(), "cannot create paths.tsv");
        manifest << "frame\tframe_id\tcapture_time_ms\treference_source"
                 << "\treference_mode\tsample_count\tcross\tcircle_phase"
                 << "\tcircle_next_phase\tml_active\toutput\n";

        const ls2k::port::MotionHistory empty_motion_history{};
        std::size_t frames_with_path = 0U;
        std::size_t cross_frames = 0U;
        std::size_t circle_frames = 0U;
        std::size_t ml_frames = 0U;
        for (const std::filesystem::path& frame_path : frames) {
            const auto stamp = stamps.find(frame_path.filename().string());
            Require(stamp != stamps.end(),
                    "metadata missing " + frame_path.filename().string());
            const std::vector<std::uint8_t> gray = ReadGray8(frame_path);
            const std::vector<std::uint8_t> yuyv = GrayToNeutralYuyv(gray);
            const ls2k::port::CameraCapture capture =
                MakeCapture(stamp->second, gray, yuyv);
            const ls2k::port::PerceptionResult perception =
                pipeline.ProcessFrame(capture, params, empty_motion_history);

            cv::Mat gray_image(kHeight,
                               kWidth,
                               CV_8UC1,
                               const_cast<std::uint8_t*>(gray.data()));
            cv::Mat canvas{};
            cv::cvtColor(gray_image, canvas, cv::COLOR_GRAY2BGR);
            const std::size_t sample_count =
                DrawReferencePath(canvas, perception.reference_path, projector);
            DrawStatus(canvas, perception, sample_count);
            if (sample_count > 0U) {
                ++frames_with_path;
            }
            cross_frames +=
                perception.element_evidence.cross_exit.present ? 1U : 0U;
            circle_frames += perception.circle_v2.frame_phase != "idle" ? 1U : 0U;
            ml_frames += perception.ml.active ? 1U : 0U;

            const std::string output_name = frame_path.stem().string() + ".png";
            Require(cv::imwrite((output_directory / output_name).string(),
                                canvas),
                    "failed to write " + output_name);
            manifest << frame_path.filename().string() << '\t'
                     << stamp->second.frame_id << '\t'
                     << stamp->second.capture_time_ms << '\t'
                     << perception.reference_source << '\t'
                     << perception.reference_mode << '\t'
                     << sample_count << '\t'
                     << (perception.element_evidence.cross_exit.present ? 1 : 0)
                     << '\t' << perception.circle_v2.frame_phase
                     << '\t' << perception.circle_v2.next_phase << '\t'
                     << (perception.ml.active ? 1 : 0) << '\t'
                     << output_name << '\n';
        }
        Require(manifest.good(), "failed while writing paths.tsv");
        std::cout << "PASS: current production pipeline image-only replay"
                  << " frames=" << frames.size()
                  << " excluded=" << excluded.size()
                  << " with_path=" << frames_with_path
                  << " cross=" << cross_frames
                  << " circle_non_idle=" << circle_frames
                  << " ml_active=" << ml_frames
                  << " output=" << output_directory << '\n';
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
