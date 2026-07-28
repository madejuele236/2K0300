#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "vision/image/illumination_binary_model.hpp"

namespace {

constexpr int kWidth = 320;
constexpr int kHeight = 240;

std::uint64_t Fnv1a(const std::uint8_t* data, std::size_t size) {
    std::uint64_t hash = 1469598103934665603ULL;
    for (std::size_t index = 0; index < size; ++index) {
        hash ^= data[index];
        hash *= 1099511628211ULL;
    }
    return hash;
}

std::vector<std::uint8_t> ReadGray(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input.is_open()) {
        return {};
    }
    const std::streamsize size = input.tellg();
    if (size != kWidth * kHeight) {
        return {};
    }
    input.seekg(0);
    std::vector<std::uint8_t> gray(static_cast<std::size_t>(size));
    input.read(reinterpret_cast<char*>(gray.data()), size);
    return input.good() ? gray : std::vector<std::uint8_t>{};
}

std::uint64_t Process(const std::filesystem::path& path) {
    const std::vector<std::uint8_t> gray = ReadGray(path);
    if (gray.empty()) {
        std::cerr << "invalid gray8 frame: " << path << '\n';
        std::exit(2);
    }
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

    const auto begin = std::chrono::steady_clock::now();
    const auto computed =
        ls2k::vision::ComputeIlluminationBinaryModel(frame);
    const auto end = std::chrono::steady_clock::now();
    ls2k::port::BinaryModelState model{};
    model.valid = computed.valid;
    model.residual_threshold = computed.residual_threshold;
    model.illumination_weight = computed.illumination_weight;
    model.source = computed.valid
                       ? ls2k::port::BinaryModelSource::kCurrent
                       : ls2k::port::BinaryModelSource::kNone;
    model.illumination = computed.illumination;

    std::vector<std::uint8_t> binary(gray.size(), 0U);
    if (computed.valid) {
        for (int row = 0; row < kHeight; ++row) {
            for (int col = 0; col < kWidth; ++col) {
                std::uint8_t y = 0U;
                bool white = false;
                if (!ls2k::vision::ClassifyImagePixel(
                        frame, row, col, model, y, white)) {
                    std::cerr << "classification failed: " << path << '\n';
                    std::exit(3);
                }
                binary[static_cast<std::size_t>(row * kWidth + col)] =
                    white ? 1U : 0U;
            }
        }
    }
    std::cout << path.filename().string()
              << '\t' << (computed.valid ? 1 : 0)
              << '\t' << computed.residual_threshold
              << '\t' << Fnv1a(computed.illumination.data(),
                                computed.illumination.size())
              << '\t' << Fnv1a(binary.data(), binary.size())
              << '\n';
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(end - begin)
            .count());
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: binary_model_replay_probe RAW_DIRECTORY\n";
        return 1;
    }
    std::vector<std::filesystem::path> paths{};
    for (const auto& entry : std::filesystem::directory_iterator(argv[1])) {
        if (entry.is_regular_file() && entry.path().extension() == ".raw") {
            paths.push_back(entry.path());
        }
    }
    std::sort(paths.begin(), paths.end());
    std::vector<std::uint64_t> durations_us{};
    durations_us.reserve(paths.size());
    for (const auto& path : paths) {
        durations_us.push_back(Process(path));
    }
    if (!durations_us.empty()) {
        std::sort(durations_us.begin(), durations_us.end());
        std::uint64_t sum = 0U;
        for (const std::uint64_t duration : durations_us) {
            sum += duration;
        }
        const std::size_t p95_index =
            (durations_us.size() * 95U + 99U) / 100U - 1U;
        std::cerr << "binary_model_perf frames=" << durations_us.size()
                  << " avg_us=" << sum / durations_us.size()
                  << " p95_us=" << durations_us[p95_index]
                  << " max_us=" << durations_us.back() << '\n';
    }
    return 0;
}
