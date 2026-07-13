#include <arpa/inet.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "platform/bootstrap.hpp"
#include "vision/bev/bev_image_segment_connectivity.hpp"
#include "vision/bev/bev_reference_path_builder.hpp"

namespace {

struct Diagnostics final : ls2k::port::DiagnosticSink {
    void Emit(const ls2k::port::DiagnosticEvent&) override {}
};

struct PathPoint {
    float forward_m = 0.0F;
    float lateral_m = 0.0F;
};

struct Envelope {
    std::string header;
    std::vector<std::uint8_t> payload;
};

void Require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

Envelope ReadEnvelope(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    Require(input.is_open(), "cannot open aligned envelope: " + path);
    std::uint32_t lengths[2]{};
    input.read(reinterpret_cast<char*>(lengths), sizeof(lengths));
    Require(input.gcount() == static_cast<std::streamsize>(sizeof(lengths)), "truncated envelope prefix");
    const std::uint32_t header_size = ntohl(lengths[0]);
    const std::uint32_t payload_size = ntohl(lengths[1]);
    Envelope envelope{};
    envelope.header.resize(header_size);
    envelope.payload.resize(payload_size);
    input.read(envelope.header.data(), static_cast<std::streamsize>(header_size));
    Require(input.gcount() == static_cast<std::streamsize>(header_size), "truncated envelope header");
    input.read(reinterpret_cast<char*>(envelope.payload.data()), static_cast<std::streamsize>(payload_size));
    Require(input.gcount() == static_cast<std::streamsize>(payload_size), "truncated envelope payload");
    Require(input.peek() == std::char_traits<char>::eof(), "envelope has unaccounted trailing bytes");
    return envelope;
}

std::vector<PathPoint> ParseAlignedCenters(const std::string& header) {
    const std::size_t begin = header.find("\"samples\":[");
    Require(begin != std::string::npos, "aligned header lacks visual-reference samples");
    const std::size_t end = header.find(",\"reference\":", begin);
    Require(end != std::string::npos, "cannot bound visual-reference samples");
    const std::string samples = header.substr(begin, end - begin);
    const std::regex point_re(
        R"re("forward_m":(-?[0-9]+(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?),"lateral_m":(-?[0-9]+(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?))re");
    std::vector<PathPoint> points;
    for (std::sregex_iterator it(samples.begin(), samples.end(), point_re), last; it != last; ++it) {
        points.push_back({std::stof((*it)[1].str()), std::stof((*it)[2].str())});
    }
    Require(points.size() == ls2k::port::kBevReferenceSampleCount,
            "aligned header path sample count differs from runtime contract");
    return points;
}

std::vector<std::uint8_t> ExpandGray2ToYuyv(const std::vector<std::uint8_t>& packed,
                                            int width,
                                            int height) {
    const std::size_t pixels = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    Require(packed.size() == (pixels * 2U + 7U) / 8U, "gray2 payload size mismatch");
    std::vector<std::uint8_t> yuyv(pixels * 2U, 128U);
    for (std::size_t index = 0; index < pixels; ++index) {
        const std::size_t bit_index = index * 2U;
        const unsigned shift = 6U - static_cast<unsigned>(bit_index % 8U);
        const std::uint8_t level = (packed[bit_index / 8U] >> shift) & 3U;
        yuyv[index * 2U] = static_cast<std::uint8_t>(level * 85U);
    }
    return yuyv;
}

std::string StatusName(ls2k::vision::BEVSegmentConnectivityStatus status) {
    using Status = ls2k::vision::BEVSegmentConnectivityStatus;
    if (status == Status::kConnected) return "connected";
    if (status == Status::kBlocked) return "blocked";
    return "unobservable";
}

ls2k::vision::BEVSimpleRowScan MakeRow(const PathPoint& point,
                                       const ls2k::port::RuntimeParameters& params) {
    ls2k::vision::BEVSimpleRowScan row{};
    row.valid = true;
    row.forward_m = point.forward_m;
    row.sampleable_count = 161U;
    row.sampleable_left_m = -0.8F;
    row.sampleable_right_m = 0.8F;
    row.sampleable_width_m = 1.6F;
    const float half_width = params.bev_geometry.nominal_road_half_width_m;
    ls2k::vision::BEVBoundarySpan span{};
    span.forward_m = point.forward_m;
    span.left_m = point.lateral_m - half_width;
    span.right_m = point.lateral_m + half_width;
    span.center_m = point.lateral_m;
    span.width_m = 2.0F * half_width;
    span.left_lateral_index = static_cast<int>(std::lround(span.left_m * 100.0F));
    span.right_lateral_index = static_cast<int>(std::lround(span.right_m * 100.0F));
    row.spans.push_back(span);
    return row;
}

void WriteReport(const std::string& path,
                 const std::string& source,
                 const std::string& source_sha256,
                 const std::string& params_sha256,
                 const ls2k::port::RuntimeParameters& params,
                 std::size_t omitted_row,
                 const std::vector<PathPoint>& output,
                 const std::vector<ls2k::vision::BEVSegmentConnectivityResult>& edges,
                 bool blocked_found,
                 const PathPoint& blocked_from,
                 const PathPoint& blocked_to,
                 const ls2k::vision::BEVSegmentConnectivityResult& blocked) {
    std::ofstream out(path);
    Require(out.is_open(), "cannot write replay report: " + path);
    out << "{\n  \"result\": \"PASS\",\n"
        << "  \"source\": \"" << source << "\",\n"
        << "  \"source_sha256\": \"" << source_sha256 << "\",\n"
        << "  \"alignment\": {\"frame_source\": \"snapshot_aligned\", \"width\": 320, \"height\": 240, \"payload_encoding\": \"gray2_packed\"},\n"
        << "  \"decode\": \"gray2 levels expanded to gray8 luma in YUYV storage; this preserves the captured quantized levels but not raw-sensor precision\",\n"
        << "  \"parameters\": {\"source\": \"new/config/default_params.json via production ParamStore\", \"sha256\": \""
        << params_sha256 << "\", \"local_jump_min_y\": " << params.bev_boundary.local_jump_min_y
        << ", \"nominal_road_half_width_m\": " << params.bev_geometry.nominal_road_half_width_m
        << ", \"boundary_trace_max_adjacent_distance_m\": " << params.bev_geometry.boundary_trace_max_adjacent_distance_m << "},\n"
        << "  \"omitted_row\": " << omitted_row << ",\n  \"output_points\": [";
    for (std::size_t i = 0; i < output.size(); ++i) {
        if (i) out << ',';
        out << "{\"forward_m\":" << std::setprecision(10) << output[i].forward_m
            << ",\"lateral_m\":" << output[i].lateral_m << '}';
    }
    out << "],\n  \"accepted_edges\": [";
    for (std::size_t i = 0; i < edges.size(); ++i) {
        if (i) out << ',';
        out << "{\"from_output_index\":" << i << ",\"to_output_index\":" << (i + 1U)
            << ",\"status\":\"" << StatusName(edges[i].status) << "\",\"sample_count\":"
            << edges[i].sampled_point_count << '}';
    }
    out << "],\n  \"blocked_pair\": ";
    if (blocked_found) {
        out << "{\"from\":{\"forward_m\":" << blocked_from.forward_m << ",\"lateral_m\":" << blocked_from.lateral_m
            << "},\"to\":{\"forward_m\":" << blocked_to.forward_m << ",\"lateral_m\":" << blocked_to.lateral_m
            << "},\"status\":\"blocked\",\"sample_count\":" << blocked.sampled_point_count << '}';
    } else {
        out << "null";
    }
    out << ",\n  \"commands\": [\"new/verification/tests/run_bev_connectivity_aligned_replay.sh\", \"git diff --check\"]\n}\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Require(argc == 6, "usage: replay ENVELOPE PARAMS REPORT SOURCE_SHA256 PARAMS_SHA256");
        const Envelope envelope = ReadEnvelope(argv[1]);
        Require(envelope.header.find("\"frame_source\":\"snapshot_aligned\"") != std::string::npos,
                "frame_source is not snapshot_aligned");
        Require(envelope.header.find("\"width\":320") != std::string::npos &&
                    envelope.header.find("\"height\":240") != std::string::npos,
                "aligned dimensions are not 320x240");
        Require(envelope.header.find("\"payload_encoding\":\"gray2_packed\"") != std::string::npos,
                "payload is not gray2_packed");
        const std::vector<PathPoint> centers = ParseAlignedCenters(envelope.header);

        Diagnostics diagnostics{};
        const std::unique_ptr<ls2k::port::IParamStore> store = ls2k::platform::MakeParamStore();
        ls2k::port::RuntimeParameters params{};
        Require(store && store->LoadRuntimeParameters(argv[2], params, diagnostics),
                "production ParamStore failed to load current defaults");
        Require(!params.loaded_from_defaults && !params.parse_failure,
                "current defaults unexpectedly fell back or failed validation");
        ls2k::vision::BEVProjector projector{};
        Require(projector.Configure(params.bev_projector), "production projector rejected current calibration");

        std::vector<std::uint8_t> yuyv = ExpandGray2ToYuyv(envelope.payload, 320, 240);
        ls2k::port::CameraPixelFrameView frame{};
        frame.valid = true;
        frame.format = ls2k::port::CameraFrameFormat::kYuyv;
        frame.data = yuyv.data();
        frame.width = 320;
        frame.height = 240;
        frame.stride = 640;
        const ls2k::vision::BEVImageSegmentConnectivity query(frame, projector, params.bev_boundary);

        const std::size_t omitted_row = 10U;
        std::vector<ls2k::vision::BEVSimpleRowScan> rows;
        rows.reserve(centers.size());
        for (std::size_t i = 0; i < centers.size(); ++i) {
            auto row = MakeRow(centers[i], params);
            if (i == omitted_row) row.valid = false;
            rows.push_back(row);
        }
        const ls2k::port::BEVReferencePath reference =
            ls2k::vision::BuildReferencePath(rows, params, query);
        std::vector<PathPoint> output;
        for (const auto& sample : reference.sampled_path) {
            if (sample.present) output.push_back({sample.point.forward_m, sample.point.lateral_m});
        }
        Require(output.size() > omitted_row,
                "connected path ended before it could reconnect across omitted row");
        for (const PathPoint& point : output) {
            Require(std::fabs(point.forward_m - centers[omitted_row].forward_m) > 1.0e-5F,
                    "omitted row unexpectedly appeared in connected output");
        }
        Require(output[omitted_row].forward_m > centers[omitted_row].forward_m,
                "later point did not reconnect across omitted interior row");

        std::vector<ls2k::vision::BEVSegmentConnectivityResult> edges;
        for (std::size_t i = 1; i < output.size(); ++i) {
            const auto result = query.Evaluate({output[i - 1].forward_m, output[i - 1].lateral_m},
                                               {output[i].forward_m, output[i].lateral_m},
                                               ls2k::vision::BEVSegmentVisibilityPolicy::kRequireFullSegment);
            Require(result.status == ls2k::vision::BEVSegmentConnectivityStatus::kConnected,
                    "accepted edge is blocked or unobservable on production re-evaluation");
            edges.push_back(result);
        }

        bool blocked_found = false;
        PathPoint blocked_from{}, blocked_to{};
        ls2k::vision::BEVSegmentConnectivityResult blocked{};
        for (float forward : params.bev_geometry.forward_samples_m) {
            for (float a = -0.8F; a <= 0.8F && !blocked_found; a += 0.1F) {
                for (float b = a + 0.1F; b <= 0.8F; b += 0.1F) {
                    const auto result = query.Evaluate({forward, a}, {forward, b},
                        ls2k::vision::BEVSegmentVisibilityPolicy::kRequireFullSegment);
                    if (result.status == ls2k::vision::BEVSegmentConnectivityStatus::kBlocked) {
                        blocked_found = true;
                        blocked_from = {forward, a};
                        blocked_to = {forward, b};
                        blocked = result;
                        break;
                    }
                }
            }
            if (blocked_found) break;
        }
        WriteReport(argv[3], argv[1], argv[4], argv[5], params, omitted_row,
                    output, edges, blocked_found, blocked_from, blocked_to, blocked);
        std::cout << "PASS: aligned connectivity replay reconnected row " << omitted_row
                  << "; accepted_edges=" << edges.size()
                  << "; blocked_pair=" << (blocked_found ? "found" : "not_found") << '\n';
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
