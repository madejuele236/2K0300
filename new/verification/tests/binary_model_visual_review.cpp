#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "vision/image/illumination_binary_model.hpp"

namespace {

constexpr int kWidth = 320;
constexpr int kHeight = 240;
constexpr int kReviewImageCount = 5;

struct FrameStats {
    std::filesystem::path path{};
    double mean_y = 0.0;
    double white_fraction = 0.0;
    int residual_threshold = 0;
};

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::vector<std::uint8_t> ReadGray(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    Require(input.is_open(), "cannot open " + path.string());
    const std::streamsize size = input.tellg();
    Require(size == kWidth * kHeight,
            "raw frame must be 320x240 gray8: " + path.string());
    input.seekg(0);
    std::vector<std::uint8_t> gray(static_cast<std::size_t>(size));
    input.read(reinterpret_cast<char*>(gray.data()), size);
    Require(input.gcount() == size, "truncated frame: " + path.string());
    return gray;
}

struct EvaluatedFrame {
    ls2k::port::BinaryModelState model{};
    std::vector<std::uint8_t> binary{};
};

EvaluatedFrame Evaluate(
    const std::vector<std::uint8_t>& gray,
    int threshold_margin = ls2k::vision::kBinaryResidualThresholdMargin,
    float blur_radius_px = ls2k::vision::kBinaryIlluminationBlurRadiusPx,
    int illumination_weight =
        ls2k::port::kBinaryDefaultIlluminationWeight) {
    std::vector<std::uint8_t> yuyv(gray.size() * 2U, 128U);
    for (std::size_t index = 0; index < gray.size(); ++index) {
        yuyv[index * 2U] = gray[index];
    }
    ls2k::port::CameraPixelFrameView frame{};
    frame.valid = true;
    frame.format = ls2k::port::CameraFrameFormat::kYuyv;
    frame.data = yuyv.data();
    frame.width = kWidth;
    frame.height = kHeight;
    frame.stride = kWidth * 2;

    ls2k::vision::BinaryModelTracker tracker{};
    EvaluatedFrame evaluated{};
    ls2k::vision::BinaryModelParameters parameters{};
    parameters.illumination_blur_radius_px = blur_radius_px;
    parameters.residual_threshold_margin = threshold_margin;
    parameters.illumination_weight = illumination_weight;
    evaluated.model = tracker.Update(
        ls2k::vision::ComputeIlluminationBinaryModel(frame, parameters));
    if (!evaluated.model.valid) {
        return evaluated;
    }
    evaluated.binary.resize(gray.size(), 0U);
    for (int row = 0; row < kHeight; ++row) {
        for (int col = 0; col < kWidth; ++col) {
            std::uint8_t y = 0U;
            bool white = false;
            Require(ls2k::vision::ClassifyImagePixel(
                        frame, row, col, evaluated.model, y, white),
                    "production classifier rejected a valid image pixel");
            evaluated.binary[static_cast<std::size_t>(row * kWidth + col)] =
                white ? 255U : 0U;
        }
    }
    return evaluated;
}

FrameStats Measure(const std::filesystem::path& path) {
    const std::vector<std::uint8_t> gray = ReadGray(path);
    const EvaluatedFrame evaluated = Evaluate(gray);
    Require(evaluated.model.valid,
            "binary model invalid for " + path.string());
    std::uint64_t luma_sum = 0U;
    std::size_t white_count = 0U;
    for (std::size_t index = 0; index < gray.size(); ++index) {
        luma_sum += gray[index];
        white_count += evaluated.binary[index] != 0U ? 1U : 0U;
    }
    FrameStats stats{};
    stats.path = path;
    stats.mean_y =
        static_cast<double>(luma_sum) / static_cast<double>(gray.size());
    stats.white_fraction =
        static_cast<double>(white_count) / static_cast<double>(gray.size());
    stats.residual_threshold = evaluated.model.residual_threshold;
    return stats;
}

double Normalize(double value, double minimum, double maximum) {
    return maximum > minimum ? (value - minimum) / (maximum - minimum) : 0.5;
}

std::vector<std::size_t> SelectDiverseFrames(
    const std::vector<FrameStats>& frames) {
    double min_mean = std::numeric_limits<double>::infinity();
    double max_mean = -std::numeric_limits<double>::infinity();
    double min_white = std::numeric_limits<double>::infinity();
    double max_white = -std::numeric_limits<double>::infinity();
    int min_threshold = std::numeric_limits<int>::max();
    int max_threshold = std::numeric_limits<int>::min();
    for (const FrameStats& frame : frames) {
        min_mean = std::min(min_mean, frame.mean_y);
        max_mean = std::max(max_mean, frame.mean_y);
        min_white = std::min(min_white, frame.white_fraction);
        max_white = std::max(max_white, frame.white_fraction);
        min_threshold = std::min(min_threshold, frame.residual_threshold);
        max_threshold = std::max(max_threshold, frame.residual_threshold);
    }

    const auto coordinate = [&](std::size_t index) {
        const FrameStats& frame = frames[index];
        return std::array<double, 3>{
            Normalize(frame.mean_y, min_mean, max_mean),
            Normalize(frame.white_fraction, min_white, max_white),
            Normalize(static_cast<double>(frame.residual_threshold),
                      static_cast<double>(min_threshold),
                      static_cast<double>(max_threshold)),
        };
    };
    const auto distance_squared = [](const std::array<double, 3>& lhs,
                                     const std::array<double, 3>& rhs) {
        double distance = 0.0;
        for (std::size_t axis = 0; axis < lhs.size(); ++axis) {
            const double delta = lhs[axis] - rhs[axis];
            distance += delta * delta;
        }
        return distance;
    };

    std::size_t center_index = 0U;
    double center_distance = std::numeric_limits<double>::infinity();
    constexpr std::array<double, 3> kCenter{0.5, 0.5, 0.5};
    for (std::size_t index = 0; index < frames.size(); ++index) {
        const double distance = distance_squared(coordinate(index), kCenter);
        if (distance < center_distance) {
            center_distance = distance;
            center_index = index;
        }
    }

    std::vector<std::size_t> selected{center_index};
    while (selected.size() <
           std::min<std::size_t>(kReviewImageCount, frames.size())) {
        std::size_t next_index = 0U;
        double next_distance = -1.0;
        for (std::size_t index = 0; index < frames.size(); ++index) {
            if (std::find(selected.begin(), selected.end(), index) !=
                selected.end()) {
                continue;
            }
            double nearest = std::numeric_limits<double>::infinity();
            for (const std::size_t chosen : selected) {
                nearest = std::min(
                    nearest,
                    distance_squared(coordinate(index), coordinate(chosen)));
            }
            if (nearest > next_distance) {
                next_distance = nearest;
                next_index = index;
            }
        }
        selected.push_back(next_index);
    }
    return selected;
}

void WriteCombinedPgm(const std::filesystem::path& path,
                      const std::vector<std::uint8_t>& gray,
                      const std::vector<std::uint8_t>& binary) {
    constexpr int kSeparatorWidth = 4;
    constexpr int kOutputWidth = kWidth * 2 + kSeparatorWidth;
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    Require(output.is_open(), "cannot create " + path.string());
    output << "P5\n" << kOutputWidth << ' ' << kHeight << "\n255\n";
    const std::array<std::uint8_t, kSeparatorWidth> separator{};
    for (int row = 0; row < kHeight; ++row) {
        const std::size_t offset = static_cast<std::size_t>(row * kWidth);
        output.write(
            reinterpret_cast<const char*>(gray.data() + offset), kWidth);
        output.write(reinterpret_cast<const char*>(separator.data()),
                     separator.size());
        output.write(
            reinterpret_cast<const char*>(binary.data() + offset), kWidth);
    }
    Require(output.good(), "failed to write " + path.string());
}

void WriteMarginSweepPgm(const std::filesystem::path& path,
                         const std::vector<std::uint8_t>& gray) {
    constexpr std::array<int, 5> kMargins{22, 0, -20, -40, -60};
    constexpr int kSeparatorWidth = 4;
    constexpr int kPanelCount = 1 + static_cast<int>(kMargins.size());
    constexpr int kOutputWidth =
        kWidth * kPanelCount + kSeparatorWidth * (kPanelCount - 1);
    std::array<std::vector<std::uint8_t>, kMargins.size()> binaries{};
    for (std::size_t index = 0; index < kMargins.size(); ++index) {
        const EvaluatedFrame evaluated = Evaluate(gray, kMargins[index]);
        Require(evaluated.model.valid,
                "selected frame became invalid during margin sweep");
        binaries[index] = evaluated.binary;
    }

    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    Require(output.is_open(), "cannot create " + path.string());
    output << "P5\n" << kOutputWidth << ' ' << kHeight << "\n255\n";
    const std::array<std::uint8_t, kSeparatorWidth> separator{};
    for (int row = 0; row < kHeight; ++row) {
        const std::size_t offset = static_cast<std::size_t>(row * kWidth);
        output.write(
            reinterpret_cast<const char*>(gray.data() + offset), kWidth);
        for (const std::vector<std::uint8_t>& binary : binaries) {
            output.write(reinterpret_cast<const char*>(separator.data()),
                         separator.size());
            output.write(
                reinterpret_cast<const char*>(binary.data() + offset), kWidth);
        }
    }
    Require(output.good(), "failed to write " + path.string());
}

void WriteBlurSweepPgm(const std::filesystem::path& path,
                       const std::vector<std::uint8_t>& gray) {
    constexpr std::array<float, 5> kRadiiPx{12.0F, 18.0F, 24.0F, 35.0F, 48.0F};
    constexpr int kSeparatorWidth = 4;
    constexpr int kPanelCount = 1 + static_cast<int>(kRadiiPx.size());
    constexpr int kOutputWidth =
        kWidth * kPanelCount + kSeparatorWidth * (kPanelCount - 1);
    std::array<std::vector<std::uint8_t>, kRadiiPx.size()> binaries{};
    for (std::size_t index = 0; index < kRadiiPx.size(); ++index) {
        const EvaluatedFrame evaluated = Evaluate(
            gray,
            ls2k::vision::kBinaryResidualThresholdMargin,
            kRadiiPx[index]);
        Require(evaluated.model.valid,
                "selected frame became invalid during blur sweep");
        binaries[index] = evaluated.binary;
    }

    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    Require(output.is_open(), "cannot create " + path.string());
    output << "P5\n" << kOutputWidth << ' ' << kHeight << "\n255\n";
    const std::array<std::uint8_t, kSeparatorWidth> separator{};
    for (int row = 0; row < kHeight; ++row) {
        const std::size_t offset = static_cast<std::size_t>(row * kWidth);
        output.write(
            reinterpret_cast<const char*>(gray.data() + offset), kWidth);
        for (const std::vector<std::uint8_t>& binary : binaries) {
            output.write(reinterpret_cast<const char*>(separator.data()),
                         separator.size());
            output.write(
                reinterpret_cast<const char*>(binary.data() + offset), kWidth);
        }
    }
    Require(output.good(), "failed to write " + path.string());
}

void WriteParameterGridPgm(const std::filesystem::path& path,
                           const std::vector<std::uint8_t>& gray) {
    struct Candidate {
        float blur_radius_px;
        int threshold_margin;
    };
    constexpr std::array<Candidate, 6> kCandidates{{
        {18.0F, 22},
        {18.0F, 40},
        {18.0F, 60},
        {24.0F, 22},
        {24.0F, 40},
        {24.0F, 60},
    }};
    constexpr int kSeparatorWidth = 4;
    constexpr int kPanelCount = 1 + static_cast<int>(kCandidates.size());
    constexpr int kOutputWidth =
        kWidth * kPanelCount + kSeparatorWidth * (kPanelCount - 1);
    std::array<std::vector<std::uint8_t>, kCandidates.size()> binaries{};
    for (std::size_t index = 0; index < kCandidates.size(); ++index) {
        const EvaluatedFrame evaluated = Evaluate(
            gray,
            kCandidates[index].threshold_margin,
            kCandidates[index].blur_radius_px);
        Require(evaluated.model.valid,
                "selected frame became invalid during parameter grid");
        binaries[index] = evaluated.binary;
    }

    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    Require(output.is_open(), "cannot create " + path.string());
    output << "P5\n" << kOutputWidth << ' ' << kHeight << "\n255\n";
    const std::array<std::uint8_t, kSeparatorWidth> separator{};
    for (int row = 0; row < kHeight; ++row) {
        const std::size_t offset = static_cast<std::size_t>(row * kWidth);
        output.write(
            reinterpret_cast<const char*>(gray.data() + offset), kWidth);
        for (const std::vector<std::uint8_t>& binary : binaries) {
            output.write(reinterpret_cast<const char*>(separator.data()),
                         separator.size());
            output.write(
                reinterpret_cast<const char*>(binary.data() + offset), kWidth);
        }
    }
    Require(output.good(), "failed to write " + path.string());
}

void WriteWeightSweepPgm(const std::filesystem::path& path,
                         const std::vector<std::uint8_t>& gray) {
    constexpr std::array<int, 5> kWeights{3, 5, 7, 8, 9};
    constexpr int kSeparatorWidth = 4;
    constexpr int kPanelCount = 1 + static_cast<int>(kWeights.size());
    constexpr int kOutputWidth =
        kWidth * kPanelCount + kSeparatorWidth * (kPanelCount - 1);
    std::array<std::vector<std::uint8_t>, kWeights.size()> binaries{};
    for (std::size_t index = 0; index < kWeights.size(); ++index) {
        const EvaluatedFrame evaluated = Evaluate(
            gray,
            ls2k::vision::kBinaryResidualThresholdMargin,
            ls2k::vision::kBinaryIlluminationBlurRadiusPx,
            kWeights[index]);
        Require(evaluated.model.valid,
                "selected frame became invalid during weight sweep");
        binaries[index] = evaluated.binary;
    }

    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    Require(output.is_open(), "cannot create " + path.string());
    output << "P5\n" << kOutputWidth << ' ' << kHeight << "\n255\n";
    const std::array<std::uint8_t, kSeparatorWidth> separator{};
    for (int row = 0; row < kHeight; ++row) {
        const std::size_t offset = static_cast<std::size_t>(row * kWidth);
        output.write(
            reinterpret_cast<const char*>(gray.data() + offset), kWidth);
        for (const std::vector<std::uint8_t>& binary : binaries) {
            output.write(reinterpret_cast<const char*>(separator.data()),
                         separator.size());
            output.write(
                reinterpret_cast<const char*>(binary.data() + offset), kWidth);
        }
    }
    Require(output.good(), "failed to write " + path.string());
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Require(argc == 3,
                "usage: binary_model_visual_review RAW_DIRECTORY OUTPUT_DIRECTORY");
        std::vector<std::filesystem::path> paths{};
        for (const auto& entry :
             std::filesystem::directory_iterator(argv[1])) {
            if (entry.is_regular_file() &&
                entry.path().extension() == ".raw") {
                paths.push_back(entry.path());
            }
        }
        std::sort(paths.begin(), paths.end());
        Require(!paths.empty(), "no raw frames found");

        std::vector<FrameStats> frames{};
        frames.reserve(paths.size());
        for (const std::filesystem::path& path : paths) {
            frames.push_back(Measure(path));
        }
        const std::vector<std::size_t> selected =
            SelectDiverseFrames(frames);
        const std::filesystem::path output_directory = argv[2];
        std::filesystem::create_directories(output_directory);
        std::ofstream manifest(output_directory / "selection.tsv",
                               std::ios::trunc);
        Require(manifest.is_open(), "cannot create selection manifest");
        manifest << "rank\tframe\tmean_y\twhite_fraction\tresidual_threshold"
                 << "\tcombined_pgm\n";
        manifest << std::fixed << std::setprecision(6);

        for (std::size_t rank = 0; rank < selected.size(); ++rank) {
            const FrameStats& stats = frames[selected[rank]];
            const std::vector<std::uint8_t> gray = ReadGray(stats.path);
            const EvaluatedFrame evaluated = Evaluate(gray);
            Require(evaluated.model.valid,
                    "selected frame became invalid");
            std::ostringstream name{};
            name << std::setfill('0') << std::setw(2) << rank + 1U << '-'
                 << stats.path.stem().string() << "-raw-left-binary-right.pgm";
            const std::filesystem::path combined = output_directory / name.str();
            WriteCombinedPgm(combined, gray, evaluated.binary);
            const EvaluatedFrame candidate = Evaluate(
                gray,
                ls2k::vision::kBinaryResidualThresholdMargin,
                ls2k::vision::kBinaryIlluminationBlurRadiusPx,
                5);
            std::ostringstream candidate_name{};
            candidate_name << std::setfill('0') << std::setw(2) << rank + 1U
                           << '-' << stats.path.stem().string()
                           << "-candidate-w5.pgm";
            WriteCombinedPgm(
                output_directory / candidate_name.str(),
                gray,
                candidate.binary);
            std::ostringstream sweep_name{};
            sweep_name << std::setfill('0') << std::setw(2) << rank + 1U
                       << '-' << stats.path.stem().string()
                       << "-margin-sweep.pgm";
            WriteMarginSweepPgm(output_directory / sweep_name.str(), gray);
            std::ostringstream blur_sweep_name{};
            blur_sweep_name << std::setfill('0') << std::setw(2) << rank + 1U
                            << '-' << stats.path.stem().string()
                            << "-blur-sweep.pgm";
            WriteBlurSweepPgm(
                output_directory / blur_sweep_name.str(), gray);
            std::ostringstream grid_name{};
            grid_name << std::setfill('0') << std::setw(2) << rank + 1U
                      << '-' << stats.path.stem().string()
                      << "-parameter-grid.pgm";
            WriteParameterGridPgm(output_directory / grid_name.str(), gray);
            std::ostringstream weight_sweep_name{};
            weight_sweep_name
                << std::setfill('0') << std::setw(2) << rank + 1U
                << '-' << stats.path.stem().string()
                << "-weight-sweep.pgm";
            WriteWeightSweepPgm(
                output_directory / weight_sweep_name.str(), gray);
            manifest << rank + 1U << '\t' << stats.path.filename().string()
                     << '\t' << stats.mean_y
                     << '\t' << stats.white_fraction
                     << '\t' << stats.residual_threshold
                     << '\t' << combined.filename().string() << '\n';
        }
        std::cout << "PASS: selected " << selected.size()
                  << " representative frames from " << frames.size() << '\n';
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
