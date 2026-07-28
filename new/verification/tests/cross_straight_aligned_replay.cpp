#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
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

std::vector<std::uint8_t> ReadGray8(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    Require(input.is_open(), "cannot open frame: " + path.string());
    const std::streamsize size = input.tellg();
    Require(size == 320 * 240, "expected aligned 320x240 gray8 frame");
    input.seekg(0);
    std::vector<std::uint8_t> gray(static_cast<std::size_t>(size));
    input.read(reinterpret_cast<char*>(gray.data()), size);
    Require(input.gcount() == size, "truncated frame: " + path.string());
    return gray;
}

std::vector<std::uint8_t> RotateGrayInBev(
    const std::vector<std::uint8_t>& gray,
    double angle_deg,
    const ls2k::vision::BEVProjector& projector) {
    if (angle_deg == 0.0) {
        return gray;
    }
    constexpr double kPi = 3.14159265358979323846;
    const double angle_rad = angle_deg * kPi / 180.0;
    const double cosine = std::cos(angle_rad);
    const double sine = std::sin(angle_rad);
    std::vector<std::uint8_t> rotated(gray.size(), 0U);
    for (int row = 0; row < 240; ++row) {
        for (int col = 0; col < 320; ++col) {
            ls2k::port::BEVPoint destination{};
            if (!projector.ProjectImageToVehicle(
                    ls2k::port::ImagePoint{static_cast<float>(row), static_cast<float>(col)},
                    destination)) {
                continue;
            }
            const ls2k::port::BEVPoint source{
                static_cast<float>(-sine * destination.lateral_m +
                                   cosine * destination.forward_m),
                static_cast<float>(cosine * destination.lateral_m +
                                   sine * destination.forward_m)};
            ls2k::port::ImagePoint source_image{};
            if (!projector.ProjectVehicleToImage(source, source_image)) {
                continue;
            }
            const int source_row = std::clamp(static_cast<int>(std::lround(source_image.row_px)),
                                              0,
                                              239);
            const int source_col = std::clamp(static_cast<int>(std::lround(source_image.col_px)),
                                              0,
                                              319);
            rotated[static_cast<std::size_t>(row * 320 + col)] =
                gray[static_cast<std::size_t>(source_row * 320 + source_col)];
        }
    }
    return rotated;
}

std::vector<std::uint8_t> GrayToYuyv(const std::vector<std::uint8_t>& gray) {
    std::vector<std::uint8_t> yuyv(gray.size() * 2U, 128U);
    for (std::size_t index = 0U; index < gray.size(); ++index) {
        yuyv[index * 2U] = gray[index];
    }
    return yuyv;
}

struct ReplayResult {
    bool cross_present = false;
    ls2k::port::VisualReferenceCandidate candidate{};
    ls2k::port::BEVReferencePath ordinary_path{};
    std::vector<ls2k::vision::BEVSimpleRowScan> rows{};
};

ReplayResult Replay(const std::vector<std::uint8_t>& gray,
                    const ls2k::port::RuntimeParameters& params,
                    const ls2k::vision::BEVProjector& projector) {
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
    const auto facts =
        ls2k::vision::RunBEVSimplePerception(frame, model, params, projector, &lut);
    auto evidence = ls2k::vision::DetectCrossExitEvidence(
        facts.rows, facts.origin_to_cross_sample_midpoint_connectivity, params);
    const ls2k::vision::BEVImageSegmentConnectivity connectivity(
        frame, projector, model);
    ReplayResult result{};
    result.cross_present = evidence.present;
    result.ordinary_path = facts.reference_path;
    result.rows = facts.rows;
    result.candidate = ls2k::vision::BuildCrossStraightPathCandidate(
        facts.rows, evidence, params, connectivity);
    return result;
}

void DumpRows(const ReplayResult& result) {
    for (std::size_t row_index = 0U; row_index < result.rows.size(); ++row_index) {
        const auto& row = result.rows[row_index];
        std::cout << "  row=" << row_index << " f=" << row.forward_m << " runs=";
        for (const auto& run : row.white_runs) {
            std::cout << '[' << run.left_m << ',' << run.right_m
                      << ",le=" << static_cast<int>(run.left_endpoint)
                      << ",re=" << static_cast<int>(run.right_endpoint)
                      << ",oc=" << static_cast<int>(run.origin_connectivity) << ']';
        }
        std::cout << '\n';
    }
}

std::size_t SampleCount(const ls2k::port::BEVReferencePath& path) {
    std::size_t count = 0U;
    for (const auto& sample : path.sampled_path) {
        if (!sample.present) {
            break;
        }
        ++count;
    }
    return count;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Require(argc == 3 || argc == 4,
                "usage: cross_straight_aligned_replay FRAME_DIR PARAMS.json [--rotations]");
        const bool run_rotations = argc == 4 && std::string(argv[3]) == "--rotations";
        const bool expect_ordinary =
            argc == 4 && std::string(argv[3]) == "--expect-ordinary";
        Require(argc == 3 || run_rotations || expect_ordinary,
                "optional argument must be --rotations or --expect-ordinary");
        Diagnostics diagnostics{};
        std::unique_ptr<ls2k::port::IParamStore> store = ls2k::platform::MakeParamStore();
        ls2k::port::RuntimeParameters params{};
        Require(store && store->LoadRuntimeParameters(argv[2], params, diagnostics),
                "failed to load runtime parameters");
        ls2k::vision::BEVProjector projector{};
        Require(projector.Configure(params.bev_projector), "projector configure failed");

        std::vector<std::filesystem::path> frames{};
        for (const auto& entry : std::filesystem::directory_iterator(argv[1])) {
            if (entry.is_regular_file() && entry.path().extension() == ".raw") {
                frames.push_back(entry.path());
            }
        }
        std::sort(frames.begin(), frames.end());
        Require(!frames.empty(), "no raw frames found");

        std::size_t cross_count = 0U;
        std::size_t planned_count = 0U;
        std::size_t wrong_exit_count = 0U;
        std::string first_failure = "none";
        for (const auto& frame_path : frames) {
            const ReplayResult result = Replay(ReadGray8(frame_path), params, projector);
            if (expect_ordinary) {
                if (result.cross_present) {
                    ++cross_count;
                    if (first_failure == "none") {
                        first_failure = frame_path.filename().string() + ":false_cross";
                    }
                }
                if (SampleCount(result.ordinary_path) > 0U) {
                    ++planned_count;
                } else if (first_failure == "none") {
                    first_failure = frame_path.filename().string() + ":ordinary_path_absent";
                }
                continue;
            }
            if (!result.cross_present) {
                continue;
            }
            ++cross_count;
            if (!result.candidate.present) {
                if (first_failure == "none") {
                    first_failure = frame_path.filename().string() + ":" + result.candidate.reason;
                }
                continue;
            }
            ++planned_count;
            const std::size_t count = SampleCount(result.candidate.reference_path);
            if (count < 2U ||
                result.candidate.reference_path.sampled_path[count - 1U].point.lateral_m >=
                    result.candidate.reference_path.sampled_path[0U].point.lateral_m) {
                ++wrong_exit_count;
            }
        }

        std::cout << (expect_ordinary ? "ordinary" : "aligned")
                  << " frames=" << frames.size()
                  << " cross=" << cross_count
                  << " planned=" << planned_count
                  << " wrong_exit=" << wrong_exit_count
                  << " first_failure=" << first_failure << '\n';
        if (expect_ordinary) {
            Require(cross_count == 0U,
                    "ordinary single-boundary evidence must not be detected as Cross");
            Require(planned_count == frames.size(),
                    "every ordinary single-boundary frame must retain a line path");
        } else {
            Require(cross_count > 0U, "aligned evidence contains no detected Cross frames");
            Require(planned_count == cross_count,
                    "every detected Cross frame must produce a connected straight path");
            Require(wrong_exit_count == 0U,
                    "the path must terminate in the selected left straight-through exit");
        }

        if (run_rotations) {
            const std::vector<std::uint8_t> representative = ReadGray8(frames.front());
            for (double angle_deg : {-15.0, 0.0, 15.0}) {
                const ReplayResult rotated = Replay(RotateGrayInBev(representative,
                                                                    angle_deg,
                                                                    projector),
                                                    params,
                                                    projector);
                const std::size_t count = SampleCount(rotated.candidate.reference_path);
                std::cout << "rotation_deg=" << angle_deg
                          << " cross=" << rotated.cross_present
                          << " planned=" << rotated.candidate.present
                          << " samples=" << count
                          << " reason=" << rotated.candidate.reason << '\n';
                if (!rotated.cross_present || !rotated.candidate.present) {
                    DumpRows(rotated);
                }
                Require(rotated.cross_present,
                        "rotated Cross must retain current-frame detection");
                Require(rotated.candidate.present,
                        "every rotated Cross must retain a connected straight path");
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "cross_straight_aligned_replay failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
