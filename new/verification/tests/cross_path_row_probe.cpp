#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "platform/bootstrap.hpp"
#include "vision/bev/bev_simple_perception.hpp"
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

        const ls2k::vision::OtsuThresholdResult threshold =
            ls2k::vision::ComputeSparseOtsuThreshold(frame);
        Require(threshold.valid, "Otsu invalid");
        const ls2k::port::OtsuThresholdState state{
            true,
            threshold.threshold,
            ls2k::port::OtsuThresholdSource::kCurrent,
            0U,
        };
        ls2k::vision::BEVSampleProjectionLut lut{};
        const ls2k::vision::BEVSimplePerceptionResult result =
            ls2k::vision::RunBEVSimplePerception(frame, state, params, projector, &lut);

        std::cout << std::fixed << std::setprecision(6)
                  << "otsu=" << threshold.threshold
                  << " selected_points=";
        std::size_t selected_count = 0U;
        for (const auto& selected : result.road_path_facts.center) {
            selected_count += selected.present ? 1U : 0U;
        }
        std::cout << selected_count << '\n';

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
