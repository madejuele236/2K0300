#include <cmath>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include "platform/bootstrap.hpp"
#include "vision/bev/bev_image_segment_connectivity.hpp"
#include "vision/image/otsu_threshold.hpp"

namespace {

struct Diagnostics final : ls2k::port::DiagnosticSink {
    void Emit(const ls2k::port::DiagnosticEvent&) override {}
};

struct PathPoint {
    float forward_m = 0.0F;
    float lateral_m = 0.0F;
};

void Require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

std::string ReadFrameMetadata(const std::string& path, int frame_id) {
    std::ifstream input(path);
    Require(input.is_open(), "cannot open metadata: " + path);
    std::string line;
    const std::string needle = "\"frame_id\": " + std::to_string(frame_id) + ',';
    while (std::getline(input, line)) {
        if (line.find(needle) != std::string::npos) {
            return line;
        }
    }
    throw std::runtime_error("metadata lacks requested frame_id=" +
                             std::to_string(frame_id));
}

std::string CompactJson(const std::string& text) {
    std::string compact;
    compact.reserve(text.size());
    bool in_string = false;
    bool escaped = false;
    for (const char value : text) {
        if (in_string) {
            compact.push_back(value);
            if (escaped) {
                escaped = false;
            } else if (value == '\\') {
                escaped = true;
            } else if (value == '"') {
                in_string = false;
            }
        } else if (value == '"') {
            in_string = true;
            compact.push_back(value);
        } else if (!std::isspace(static_cast<unsigned char>(value))) {
            compact.push_back(value);
        }
    }
    return compact;
}

std::vector<std::uint8_t> ReadRawGray8(const std::string& path,
                                       int width,
                                       int height) {
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    Require(input.is_open(), "cannot open raw gray8 frame: " + path);
    const std::streamsize size = input.tellg();
    const std::size_t expected =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    Require(size == static_cast<std::streamsize>(expected),
            "raw gray8 payload size mismatch");
    input.seekg(0);
    std::vector<std::uint8_t> bytes(expected);
    input.read(reinterpret_cast<char*>(bytes.data()), size);
    Require(input.gcount() == size, "truncated raw gray8 frame");
    return bytes;
}

std::string JsonArrayAfter(const std::string& text,
                           std::size_t begin,
                           const std::string& key) {
    const std::size_t key_position = text.find(key, begin);
    Require(key_position != std::string::npos, "metadata lacks " + key);
    const std::size_t array_begin = text.find('[', key_position + key.size());
    Require(array_begin != std::string::npos, "metadata has malformed " + key);
    int depth = 0;
    bool in_string = false;
    bool escaped = false;
    for (std::size_t index = array_begin; index < text.size(); ++index) {
        const char value = text[index];
        if (in_string) {
            if (escaped) {
                escaped = false;
            } else if (value == '\\') {
                escaped = true;
            } else if (value == '"') {
                in_string = false;
            }
            continue;
        }
        if (value == '"') {
            in_string = true;
        } else if (value == '[') {
            ++depth;
        } else if (value == ']') {
            --depth;
            if (depth == 0) {
                return text.substr(array_begin, index - array_begin + 1U);
            }
        }
    }
    throw std::runtime_error("metadata has unterminated " + key);
}

std::vector<PathPoint> ParseAcceptedLinePath(const std::string& metadata) {
    const std::size_t candidates = metadata.find("\"path_candidates\":");
    Require(candidates != std::string::npos, "metadata lacks path_candidates");
    const std::size_t line_candidate = metadata.find("\"kind\":\"line\"", candidates);
    Require(line_candidate != std::string::npos, "metadata lacks accepted line candidate");
    const std::string samples = JsonArrayAfter(metadata, line_candidate, "\"samples\":");
    const std::regex point_re(
        R"re("forward_m":(-?[0-9]+(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?),"lateral_m":(-?[0-9]+(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?))re");
    std::vector<PathPoint> points;
    for (std::sregex_iterator it(samples.begin(), samples.end(), point_re), last;
         it != last;
         ++it) {
        points.push_back({std::stof((*it)[1].str()), std::stof((*it)[2].str())});
    }
    Require(!points.empty(), "accepted line candidate has no present samples");
    Require(points.size() <= ls2k::port::kBevReferenceSampleCount,
            "accepted line candidate exceeds reference sample contract");
    return points;
}

int ParsePublishedCurrentOtsu(const std::string& metadata) {
    const std::regex otsu_re(
        R"re("otsu":\{"valid":true,"threshold":([0-9]+),"source":"current","stale_frames":0\})re");
    std::smatch match;
    Require(std::regex_search(metadata, match, otsu_re),
            "aligned frame does not publish current valid Otsu state");
    return std::stoi(match[1].str());
}

std::vector<std::uint8_t> ExpandGray8ToYuyv(
    const std::vector<std::uint8_t>& gray) {
    std::vector<std::uint8_t> yuyv(gray.size() * 2U, 128U);
    for (std::size_t index = 0; index < gray.size(); ++index) {
        yuyv[index * 2U] = gray[index];
    }
    return yuyv;
}

std::string StatusName(ls2k::vision::BEVSegmentConnectivityStatus status) {
    using Status = ls2k::vision::BEVSegmentConnectivityStatus;
    if (status == Status::kConnected) return "connected";
    if (status == Status::kBlocked) return "blocked";
    return "unobservable";
}

void WriteReport(
    const std::string& path,
    const std::string& metadata_path,
    const std::string& raw_path,
    const std::string& metadata_sha256,
    const std::string& raw_sha256,
    const std::string& params_sha256,
    int otsu_threshold,
    const std::vector<PathPoint>& points,
    const std::vector<ls2k::vision::BEVSegmentConnectivityResult>& edges) {
    std::ofstream out(path);
    Require(out.is_open(), "cannot write replay report: " + path);
    out << "{\n  \"result\": \"PASS\",\n"
        << "  \"metadata\": {\"path\": \"" << metadata_path
        << "\", \"sha256\": \"" << metadata_sha256 << "\"},\n"
        << "  \"raw_gray8\": {\"path\": \"" << raw_path
        << "\", \"sha256\": \"" << raw_sha256 << "\"},\n"
        << "  \"params\": {\"path\": \"new/config/default_params.json\", \"sha256\": \""
        << params_sha256 << "\"},\n"
        << "  \"alignment\": {\"frame_source\": \"snapshot_aligned\", \"width\": 320, \"height\": 240, \"pixel_format\": \"gray8\", \"payload_encoding\": \"raw\"},\n"
        << "  \"otsu\": {\"valid\": true, \"threshold\": " << otsu_threshold
        << ", \"source\": \"current\", \"stale_frames\": 0},\n"
        << "  \"accepted_point_count\": " << points.size() << ",\n"
        << "  \"verified_edges\": [";
    for (std::size_t index = 0; index < edges.size(); ++index) {
        if (index) out << ',';
        out << "{\"from\":\"" << (index == 0U ? "origin" : "accepted_point")
            << "\",\"to_index\":" << index
            << ",\"status\":\"" << StatusName(edges[index].status)
            << "\",\"sample_count\":" << edges[index].sampled_point_count
            << ",\"visible_segment_clipped\":"
            << (edges[index].visible_segment_clipped ? "true" : "false") << '}';
    }
    out << "]\n}\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Require(argc == 11,
                "usage: replay METADATA RAW PARAMS REPORT METADATA_SHA RAW_SHA PARAMS_SHA WIDTH HEIGHT FRAME_ID");
        const int width = std::stoi(argv[8]);
        const int height = std::stoi(argv[9]);
        const int frame_id = std::stoi(argv[10]);
        Require(width == 320 && height == 240, "replay expects captured 320x240 geometry");
        const std::string metadata = CompactJson(ReadFrameMetadata(argv[1], frame_id));
        Require(metadata.find("\"frame_source\":\"snapshot_aligned\"") != std::string::npos,
                "frame source is not snapshot_aligned");
        Require(metadata.find("\"aligned\":true") != std::string::npos,
                "steering snapshot is not frame-aligned");
        Require(metadata.find("\"pixel_format\":\"gray8\"") != std::string::npos &&
                    metadata.find("\"payload_encoding\":\"raw\"") != std::string::npos,
                "payload is not exact gray8/raw");
        const int published_threshold = ParsePublishedCurrentOtsu(metadata);
        const std::vector<PathPoint> points = ParseAcceptedLinePath(metadata);

        Diagnostics diagnostics{};
        const std::unique_ptr<ls2k::port::IParamStore> store =
            ls2k::platform::MakeParamStore();
        ls2k::port::RuntimeParameters params{};
        Require(store && store->LoadRuntimeParameters(argv[3], params, diagnostics),
                "production ParamStore failed to load current defaults");
        Require(!params.loaded_from_defaults && !params.parse_failure,
                "current defaults unexpectedly fell back or failed validation");
        ls2k::vision::BEVProjector projector{};
        Require(projector.Configure(params.bev_projector),
                "production projector rejected current calibration");

        const std::vector<std::uint8_t> gray = ReadRawGray8(argv[2], width, height);
        std::vector<std::uint8_t> yuyv = ExpandGray8ToYuyv(gray);
        ls2k::port::CameraPixelFrameView frame{};
        frame.valid = true;
        frame.format = ls2k::port::CameraFrameFormat::kYuyv;
        frame.data = yuyv.data();
        frame.width = width;
        frame.height = height;
        frame.stride = width * 2;
        const ls2k::vision::OtsuThresholdResult computed =
            ls2k::vision::ComputeSparseOtsuThreshold(frame);
        Require(computed.valid, "aligned gray8 frame has no valid two-class Otsu threshold");
        Require(computed.threshold == published_threshold,
                "host replay Otsu differs from aligned board snapshot");
        const ls2k::port::OtsuThresholdState otsu{
            true, computed.threshold, ls2k::port::OtsuThresholdSource::kCurrent, 0U};
        const ls2k::vision::BEVImageSegmentConnectivity query(frame, projector, otsu);

        std::vector<ls2k::vision::BEVSegmentConnectivityResult> edges;
        edges.reserve(points.size());
        for (std::size_t index = 0; index < points.size(); ++index) {
            const ls2k::port::BEVPoint from =
                index == 0U
                    ? ls2k::port::BEVPoint{0.0F, 0.0F}
                    : ls2k::port::BEVPoint{points[index - 1U].forward_m,
                                           points[index - 1U].lateral_m};
            const ls2k::port::BEVPoint to{points[index].forward_m,
                                          points[index].lateral_m};
            const auto policy =
                index == 0U
                    ? ls2k::vision::BEVSegmentVisibilityPolicy::kAllowFromEndpointClip
                    : ls2k::vision::BEVSegmentVisibilityPolicy::kRequireFullSegment;
            const auto edge = query.Evaluate(from, to, policy);
            Require(edge.status == ls2k::vision::BEVSegmentConnectivityStatus::kConnected,
                    "accepted path contains blocked or unobservable edge at index " +
                        std::to_string(index));
            edges.push_back(edge);
        }

        WriteReport(argv[4], argv[1], argv[2], argv[5], argv[6], argv[7],
                    computed.threshold, points, edges);
        std::cout << "PASS: aligned gray8 Otsu=" << computed.threshold
                  << " accepted_points=" << points.size()
                  << " verified_edges=" << edges.size() << '\n';
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
