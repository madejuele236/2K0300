#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "platform/bootstrap.hpp"
#include "vision/bev/bev_image_segment_connectivity.hpp"
#include "vision/bev/bev_simple_perception.hpp"
#include "vision/elements/cross_exit_element_evidence.hpp"
#include "vision/elements/cross_straight_path_planner.hpp"
#include "vision/image/illumination_binary_model.hpp"

namespace {

struct Diagnostics final : ls2k::port::DiagnosticSink {
    void Emit(const ls2k::port::DiagnosticEvent&) override {}
};

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::vector<std::uint8_t> ReadGray8(const std::string& path) {
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    Require(input.is_open(), "cannot open frame: " + path);
    const std::streamsize size = input.tellg();
    Require(size == 320 * 240, "expected aligned 320x240 gray8 frame");
    input.seekg(0);
    std::vector<std::uint8_t> gray(static_cast<std::size_t>(size));
    input.read(reinterpret_cast<char*>(gray.data()), size);
    Require(input.gcount() == size, "truncated frame");
    return gray;
}

std::vector<std::uint8_t> GrayToYuyv(const std::vector<std::uint8_t>& gray) {
    std::vector<std::uint8_t> yuyv(gray.size() * 2U, 128U);
    for (std::size_t index = 0; index < gray.size(); ++index) {
        yuyv[index * 2U] = gray[index];
    }
    return yuyv;
}

const char* Polarity(ls2k::vision::BEVBoundaryJumpPolarity polarity) {
    return polarity == ls2k::vision::BEVBoundaryJumpPolarity::kRisingY
               ? "rising"
               : "falling";
}

const char* Endpoint(ls2k::vision::BEVWhiteRunEndpointState state) {
    using State = ls2k::vision::BEVWhiteRunEndpointState;
    if (state == State::kBoundary) return "boundary";
    if (state == State::kFovEdge) return "fov";
    return "gap";
}

const char* Origin(ls2k::vision::BEVWhiteRunOriginConnectivity state) {
    using State = ls2k::vision::BEVWhiteRunOriginConnectivity;
    if (state == State::kConnected) return "connected";
    if (state == State::kBlocked) return "blocked";
    if (state == State::kUnobservable) return "unobservable";
    return "unknown";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Require(argc == 3, "usage: cross_path_row_probe FRAME.raw PARAMS.json");
        Diagnostics diagnostics{};
        std::unique_ptr<ls2k::port::IParamStore> store =
            ls2k::platform::MakeParamStore();
        ls2k::port::RuntimeParameters params{};
        Require(store && store->LoadRuntimeParameters(argv[2], params, diagnostics),
                "failed to load runtime parameters");

        ls2k::vision::BEVProjector projector{};
        Require(projector.Configure(params.bev_projector), "projector configure failed");
        const std::vector<std::uint8_t> gray = ReadGray8(argv[1]);
        std::vector<std::uint8_t> yuyv = GrayToYuyv(gray);
        ls2k::port::CameraPixelFrameView frame{};
        frame.valid = true;
        frame.format = ls2k::port::CameraFrameFormat::kYuyv;
        frame.data = yuyv.data();
        frame.width = 320;
        frame.height = 240;
        frame.stride = 640;

        ls2k::vision::BinaryModelTracker model_tracker{};
        const ls2k::port::BinaryModelState model = model_tracker.Update(
            ls2k::vision::ComputeIlluminationBinaryModel(frame));
        Require(model.valid, "binary model invalid");
        ls2k::vision::BEVSampleProjectionLut lut{};
        const ls2k::vision::BEVSimplePerceptionResult result =
            ls2k::vision::RunBEVSimplePerception(frame, model, params, projector, &lut);
        const auto cross = ls2k::vision::DetectCrossExitEvidence(
            result.rows,
            result.origin_to_cross_sample_midpoint_connectivity,
            params);
        const ls2k::vision::BEVImageSegmentConnectivity connectivity(frame,
                                                                      projector,
                                                                      model);
        const auto cross_candidate = ls2k::vision::BuildCrossStraightPathCandidate(
            result.rows, cross, params, connectivity);

        std::cout << std::fixed << std::setprecision(6)
                  << "residual_threshold=" << model.residual_threshold
                  << " cross=" << cross.present
                  << " cross_path=" << cross_candidate.present
                  << " cross_reason=" << cross_candidate.reason
                  << " selected_points=";
        std::size_t selected_count = 0U;
        for (const auto& selected : result.road_path_facts.center) {
            selected_count += selected.present ? 1U : 0U;
        }
        std::cout << selected_count << '\n';
        std::cout << "cross_samples=";
        ls2k::port::BEVPoint previous{};
        bool first_cross_sample = true;
        for (const auto& sample : cross_candidate.reference_path.sampled_path) {
            if (!sample.present) {
                break;
            }
            const auto segment = connectivity.Evaluate(
                previous,
                sample.point,
                first_cross_sample
                    ? ls2k::vision::BEVSegmentVisibilityPolicy::kAllowFromEndpointClip
                    : ls2k::vision::BEVSegmentVisibilityPolicy::kRequireFullSegment);
            std::cout << '[' << sample.point.forward_m << ',' << sample.point.lateral_m
                      << ",status=" << static_cast<int>(segment.status) << ']';
            previous = sample.point;
            first_cross_sample = false;
        }
        std::cout << '\n';

        std::cout << "zero_axis_segments=";
        ls2k::port::BEVPoint previous_zero{};
        bool have_previous_zero = false;
        for (const auto& row : result.rows) {
            bool contains_zero = false;
            for (const auto& run : row.white_runs) {
                if (run.left_m <= 0.0F && run.right_m >= 0.0F) {
                    contains_zero = true;
                    break;
                }
            }
            if (!contains_zero) {
                have_previous_zero = false;
                continue;
            }
            const ls2k::port::BEVPoint current_zero{row.forward_m, 0.0F};
            if (have_previous_zero) {
                const auto segment = connectivity.Evaluate(
                    previous_zero,
                    current_zero,
                    ls2k::vision::BEVSegmentVisibilityPolicy::kRequireFullSegment);
                std::cout << '[' << previous_zero.forward_m << "->" << row.forward_m
                          << ",status=" << static_cast<int>(segment.status) << ']';
            }
            previous_zero = current_zero;
            have_previous_zero = true;
        }
        std::cout << '\n';

        for (std::size_t row_index = 0; row_index < result.rows.size(); ++row_index) {
            const auto& row = result.rows[row_index];
            std::cout << "row=" << row_index << " f=" << row.forward_m
                      << " sampleable=" << row.sampleable_count;
            for (const auto& selected : result.road_path_facts.center) {
                if (selected.present && selected.point.forward_m == row.forward_m) {
                    std::cout << " selected=" << selected.point.lateral_m;
                }
            }
            std::cout << " jumps=";
            for (const auto& jump : row.jumps) {
                std::cout << '[' << jump.lateral_m << ',' << Polarity(jump.polarity) << ']';
            }
            std::cout << " spans=";
            for (const auto& span : row.spans) {
                std::cout << '[' << span.left_m << ',' << span.right_m
                          << ",c=" << span.center_m << ']';
            }
            std::cout << " runs=";
            for (const auto& run : row.white_runs) {
                std::cout << '[' << run.left_m << ',' << run.right_m << ','
                          << Endpoint(run.left_endpoint) << ','
                          << Endpoint(run.right_endpoint) << ','
                          << Origin(run.origin_connectivity) << ']';
            }
            std::cout << '\n';
        }
    } catch (const std::exception& error) {
        std::cerr << "cross_path_row_probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
