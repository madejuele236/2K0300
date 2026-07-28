#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "vision/image/illumination_binary_model.hpp"

namespace {

constexpr int kWidth = 320;
constexpr int kHeight = 240;
constexpr std::size_t kTimeBinCount = 5U;

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

ls2k::port::CameraPixelFrameView MakeFrame(
    const std::vector<std::uint8_t>& yuyv) {
    ls2k::port::CameraPixelFrameView frame{};
    frame.valid = true;
    frame.format = ls2k::port::CameraFrameFormat::kYuyv;
    frame.data = yuyv.data();
    frame.width = kWidth;
    frame.height = kHeight;
    frame.stride = kWidth * 2;
    return frame;
}

std::vector<std::uint8_t> GrayToYuyv(
    const std::vector<std::uint8_t>& gray) {
    std::vector<std::uint8_t> yuyv(gray.size() * 2U, 128U);
    for (std::size_t index = 0; index < gray.size(); ++index) {
        yuyv[index * 2U] = gray[index];
    }
    return yuyv;
}

struct GlobalOtsuResult {
    bool valid = false;
    int threshold = 0;
    std::vector<std::uint8_t> binary{};
};

GlobalOtsuResult ComputeGlobalOtsu(
    const std::vector<std::uint8_t>& gray) {
    std::array<std::uint32_t, 256> histogram{};
    std::int64_t total_sum = 0;
    for (const std::uint8_t y : gray) {
        ++histogram[y];
        total_sum += y;
    }

    std::uint32_t left_weight = 0U;
    std::int64_t left_sum = 0;
    long double best_score = -1.0L;
    int best_threshold = 0;
    for (int threshold = 0; threshold < 255; ++threshold) {
        left_weight += histogram[static_cast<std::size_t>(threshold)];
        left_sum += static_cast<std::int64_t>(threshold) *
                    histogram[static_cast<std::size_t>(threshold)];
        const std::uint32_t right_weight =
            static_cast<std::uint32_t>(gray.size()) - left_weight;
        if (left_weight == 0U || right_weight == 0U) {
            continue;
        }
        const std::int64_t right_sum = total_sum - left_sum;
        const long double score =
            static_cast<long double>(left_sum) * left_sum / left_weight +
            static_cast<long double>(right_sum) * right_sum / right_weight;
        if (score > best_score) {
            best_score = score;
            best_threshold = threshold;
        }
    }

    GlobalOtsuResult result{};
    if (best_score < 0.0L) {
        return result;
    }
    result.valid = true;
    result.threshold = best_threshold;
    result.binary.resize(gray.size(), 0U);
    for (std::size_t index = 0; index < gray.size(); ++index) {
        result.binary[index] =
            gray[index] > result.threshold ? 255U : 0U;
    }
    return result;
}

struct CurrentResult {
    ls2k::port::BinaryModelState model{};
    std::vector<std::uint8_t> binary{};
    int illumination_p90_p10 = 0;
};

CurrentResult ComputeCurrent(const std::vector<std::uint8_t>& gray) {
    const std::vector<std::uint8_t> yuyv = GrayToYuyv(gray);
    const ls2k::port::CameraPixelFrameView frame = MakeFrame(yuyv);
    ls2k::vision::BinaryModelTracker tracker{};
    CurrentResult result{};
    result.model = tracker.Update(
        ls2k::vision::ComputeIlluminationBinaryModel(frame));
    Require(result.model.valid, "current binary model invalid");
    result.binary.resize(gray.size(), 0U);
    for (int row = 0; row < kHeight; ++row) {
        for (int col = 0; col < kWidth; ++col) {
            std::uint8_t y = 0U;
            bool white = false;
            Require(ls2k::vision::ClassifyImagePixel(
                        frame, row, col, result.model, y, white),
                    "current classifier rejected valid pixel");
            result.binary[static_cast<std::size_t>(row * kWidth + col)] =
                white ? 255U : 0U;
        }
    }
    auto illumination = result.model.illumination;
    std::sort(illumination.begin(), illumination.end());
    const std::size_t p10 = illumination.size() / 10U;
    const std::size_t p90 = illumination.size() * 9U / 10U;
    result.illumination_p90_p10 =
        static_cast<int>(illumination[p90]) -
        static_cast<int>(illumination[p10]);
    return result;
}

double WhiteFraction(const std::vector<std::uint8_t>& binary) {
    std::size_t white = 0U;
    for (const std::uint8_t value : binary) {
        white += value != 0U ? 1U : 0U;
    }
    return static_cast<double>(white) /
           static_cast<double>(binary.size());
}

void WriteComparisonPgm(
    const std::filesystem::path& path,
    const std::vector<std::uint8_t>& gray,
    const std::vector<std::uint8_t>& global_otsu,
    const std::vector<std::uint8_t>& current) {
    constexpr int kSeparatorWidth = 4;
    constexpr int kPanelCount = 5;
    constexpr int kOutputWidth =
        kWidth * kPanelCount + kSeparatorWidth * (kPanelCount - 1);
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    Require(output.is_open(), "cannot create " + path.string());
    output << "P5\n" << kOutputWidth << ' ' << kHeight << "\n255\n";
    const std::array<std::uint8_t, kSeparatorWidth> separator{};
    std::array<std::uint8_t, kWidth> current_only_white{};
    std::array<std::uint8_t, kWidth> global_only_white{};
    for (int row = 0; row < kHeight; ++row) {
        const std::size_t offset = static_cast<std::size_t>(row * kWidth);
        for (int col = 0; col < kWidth; ++col) {
            const std::size_t index =
                offset + static_cast<std::size_t>(col);
            current_only_white[static_cast<std::size_t>(col)] =
                current[index] != 0U && global_otsu[index] == 0U
                    ? 255U
                    : 0U;
            global_only_white[static_cast<std::size_t>(col)] =
                global_otsu[index] != 0U && current[index] == 0U
                    ? 255U
                    : 0U;
        }
        output.write(
            reinterpret_cast<const char*>(gray.data() + offset), kWidth);
        output.write(reinterpret_cast<const char*>(separator.data()),
                     separator.size());
        output.write(
            reinterpret_cast<const char*>(global_otsu.data() + offset),
            kWidth);
        output.write(reinterpret_cast<const char*>(separator.data()),
                     separator.size());
        output.write(
            reinterpret_cast<const char*>(current.data() + offset), kWidth);
        output.write(reinterpret_cast<const char*>(separator.data()),
                     separator.size());
        output.write(
            reinterpret_cast<const char*>(current_only_white.data()),
            current_only_white.size());
        output.write(reinterpret_cast<const char*>(separator.data()),
                     separator.size());
        output.write(
            reinterpret_cast<const char*>(global_only_white.data()),
            global_only_white.size());
    }
    Require(output.good(), "failed to write " + path.string());
}

struct Candidate {
    std::filesystem::path path{};
    int illumination_span = 0;
    std::string selection{};
    double recovery_fraction = 0.0;
    double suppression_fraction = 0.0;
};

std::vector<Candidate> SelectByTimeBins(
    const std::vector<std::filesystem::path>& paths) {
    std::vector<Candidate> selected{};
    for (std::size_t bin = 0U; bin < kTimeBinCount; ++bin) {
        const std::size_t begin = paths.size() * bin / kTimeBinCount;
        const std::size_t end =
            paths.size() * (bin + 1U) / kTimeBinCount;
        Candidate best{};
        best.illumination_span = -1;
        best.selection = "max_low_frequency_span_in_time_bin";
        for (std::size_t index = begin; index < end; ++index) {
            const CurrentResult current =
                ComputeCurrent(ReadGray(paths[index]));
            if (current.illumination_p90_p10 >
                best.illumination_span) {
                best.path = paths[index];
                best.illumination_span =
                    current.illumination_p90_p10;
            }
        }
        Require(!best.path.empty(), "empty time bin");
        selected.push_back(best);
    }
    return selected;
}

double RecoveryFraction(const GlobalOtsuResult& global,
                        const CurrentResult& current) {
    std::size_t recovered = 0U;
    std::size_t inspected = 0U;
    for (int row = kHeight / 3; row < kHeight; ++row) {
        for (int col = 0; col < kWidth; ++col) {
            const std::size_t index =
                static_cast<std::size_t>(row * kWidth + col);
            recovered += current.binary[index] != 0U &&
                                 global.binary[index] == 0U
                             ? 1U
                             : 0U;
            ++inspected;
        }
    }
    return static_cast<double>(recovered) /
           static_cast<double>(inspected);
}

double SuppressionFraction(const GlobalOtsuResult& global,
                           const CurrentResult& current) {
    std::size_t suppressed = 0U;
    std::size_t inspected = 0U;
    for (int row = kHeight / 3; row < kHeight; ++row) {
        for (int col = 0; col < kWidth; ++col) {
            const std::size_t index =
                static_cast<std::size_t>(row * kWidth + col);
            suppressed += global.binary[index] != 0U &&
                                  current.binary[index] == 0U
                              ? 1U
                              : 0U;
            ++inspected;
        }
    }
    return static_cast<double>(suppressed) /
           static_cast<double>(inspected);
}

std::vector<Candidate> SelectRecoveryByTimeBins(
    const std::vector<std::filesystem::path>& paths) {
    std::vector<Candidate> selected{};
    for (std::size_t bin = 0U; bin < kTimeBinCount; ++bin) {
        const std::size_t begin = paths.size() * bin / kTimeBinCount;
        const std::size_t end =
            paths.size() * (bin + 1U) / kTimeBinCount;
        Candidate best{};
        best.selection = "max_current_recovery_in_time_bin";
        best.recovery_fraction = -1.0;
        for (std::size_t index = begin; index < end; ++index) {
            const std::vector<std::uint8_t> gray = ReadGray(paths[index]);
            const GlobalOtsuResult global = ComputeGlobalOtsu(gray);
            const CurrentResult current = ComputeCurrent(gray);
            const double recovery = RecoveryFraction(global, current);
            if (recovery > best.recovery_fraction) {
                best.path = paths[index];
                best.illumination_span =
                    current.illumination_p90_p10;
                best.recovery_fraction = recovery;
            }
        }
        Require(!best.path.empty(), "empty recovery time bin");
        selected.push_back(best);
    }
    return selected;
}

std::vector<Candidate> SelectSuppressionByTimeBins(
    const std::vector<std::filesystem::path>& paths) {
    std::vector<Candidate> selected{};
    for (std::size_t bin = 0U; bin < kTimeBinCount; ++bin) {
        const std::size_t begin = paths.size() * bin / kTimeBinCount;
        const std::size_t end =
            paths.size() * (bin + 1U) / kTimeBinCount;
        Candidate best{};
        best.selection = "max_current_suppression_in_time_bin";
        best.suppression_fraction = -1.0;
        for (std::size_t index = begin; index < end; ++index) {
            const std::vector<std::uint8_t> gray = ReadGray(paths[index]);
            const GlobalOtsuResult global = ComputeGlobalOtsu(gray);
            const CurrentResult current = ComputeCurrent(gray);
            const double suppression =
                SuppressionFraction(global, current);
            if (suppression > best.suppression_fraction) {
                best.path = paths[index];
                best.illumination_span =
                    current.illumination_p90_p10;
                best.suppression_fraction = suppression;
            }
        }
        Require(!best.path.empty(), "empty suppression time bin");
        selected.push_back(best);
    }
    return selected;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Require(argc >= 4,
                "usage: review RAW_DIRECTORY KNOWN_HARSH_RAW OUTPUT_DIRECTORY "
                "[USER_LABELED_HARSH_RAW ...]");
        std::vector<std::filesystem::path> paths{};
        for (const auto& entry :
             std::filesystem::directory_iterator(argv[1])) {
            if (entry.is_regular_file() &&
                entry.path().extension() == ".raw") {
                paths.push_back(entry.path());
            }
        }
        std::sort(paths.begin(), paths.end());
        Require(!paths.empty(), "no replay raw frames found");
        std::vector<Candidate> selected = SelectByTimeBins(paths);
        const std::vector<Candidate> recovery_selected =
            SelectRecoveryByTimeBins(paths);
        selected.insert(selected.end(),
                        recovery_selected.begin(),
                        recovery_selected.end());
        const std::vector<Candidate> suppression_selected =
            SelectSuppressionByTimeBins(paths);
        selected.insert(selected.end(),
                        suppression_selected.begin(),
                        suppression_selected.end());
        const std::filesystem::path known_harsh = argv[2];
        const CurrentResult known_current =
            ComputeCurrent(ReadGray(known_harsh));
        selected.push_back(
            {known_harsh,
             known_current.illumination_p90_p10,
             "known_harsh",
             0.0,
             0.0});
        for (int argument = 4; argument < argc; ++argument) {
            const std::filesystem::path user_labeled = argv[argument];
            const CurrentResult current =
                ComputeCurrent(ReadGray(user_labeled));
            selected.push_back(
                {user_labeled,
                 current.illumination_p90_p10,
                 "user_labeled_harsh",
                 0.0,
                 0.0});
        }

        const std::filesystem::path output_directory = argv[3];
        std::filesystem::create_directories(output_directory);
        std::ofstream manifest(output_directory / "comparison.tsv",
                               std::ios::trunc);
        Require(manifest.is_open(), "cannot create comparison manifest");
        manifest
            << "rank\tframe\tselection\tillumination_p90_p10"
            << "\tglobal_threshold\tglobal_white_fraction"
            << "\tcurrent_threshold\tcurrent_white_fraction"
            << "\trecovery_fraction_lower_two_thirds"
            << "\tsuppression_fraction_lower_two_thirds\tpgm\n";
        manifest << std::fixed << std::setprecision(6);

        for (std::size_t index = 0U; index < selected.size(); ++index) {
            const std::vector<std::uint8_t> gray =
                ReadGray(selected[index].path);
            const GlobalOtsuResult global = ComputeGlobalOtsu(gray);
            Require(global.valid, "global Otsu invalid");
            const CurrentResult current = ComputeCurrent(gray);
            const double recovery = RecoveryFraction(global, current);
            const double suppression =
                SuppressionFraction(global, current);
            std::string stem = selected[index].path.stem().string();
            if (selected[index].selection !=
                "max_low_frequency_span_in_time_bin") {
                stem = selected[index].selection + '-' + stem;
            }
            const std::string name =
                (index < 9U ? "0" : "") +
                std::to_string(index + 1U) + '-' + stem +
                "-raw-global-current.pgm";
            WriteComparisonPgm(
                output_directory / name,
                gray,
                global.binary,
                current.binary);
            manifest << index + 1U << '\t'
                     << selected[index].path.filename().string() << '\t'
                     << selected[index].selection
                     << '\t' << selected[index].illumination_span
                     << '\t' << global.threshold
                     << '\t' << WhiteFraction(global.binary)
                     << '\t' << current.model.residual_threshold
                     << '\t' << WhiteFraction(current.binary)
                     << '\t' << recovery
                     << '\t' << suppression
                     << '\t' << name << '\n';
        }
        std::cout << "PASS: global Otsu comparison frames="
                  << selected.size() << '\n';
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
