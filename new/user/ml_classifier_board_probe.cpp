#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <string>

#include "port/ml_types.hpp"
#include "vision/ml/selected_ml_classifier.hpp"

namespace {

using Clock = std::chrono::steady_clock;

const char* ClassName(int class_id) {
    if (class_id == 0) return "supplies";
    if (class_id == 1) return "vehicle";
    if (class_id == 2) return "weapon";
    return "invalid";
}

bool ParsePositiveInt(const char* token, int& value) {
    const char* end = token + std::char_traits<char>::length(token);
    const auto parsed = std::from_chars(token, end, value);
    return parsed.ec == std::errc{} && parsed.ptr == end && value > 0;
}

bool LoadRoi(const char* path, ls2k::port::MlGrayRoi32& roi) {
    std::ifstream input(path, std::ios::binary);
    std::array<std::uint8_t, ls2k::port::kMlRoiPixelCount> bytes{};
    input.read(reinterpret_cast<char*>(bytes.data()), bytes.size());
    if (!input || input.peek() != std::ifstream::traits_type::eof()) return false;
    roi.valid = true;
    roi.gray = bytes;
    roi.reason = "saved_roi";
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " [--iterations N] ROI.raw [ROI.raw ...]\n";
        return 2;
    }
    int iterations = 1;
    int first_roi = 1;
    if (argc >= 4 && std::string(argv[1]) == "--iterations") {
        if (!ParsePositiveInt(argv[2], iterations)) {
            std::cerr << "iterations must be positive\n";
            return 2;
        }
        first_roi = 3;
    }
    if (first_roi >= argc) return 2;

    ls2k::vision::ml::SelectedMlClassifier classifier;
    if (!classifier.Initialize()) {
        std::cerr << "classifier initialization failed backend="
                  << classifier.BackendName() << '\n';
        return 3;
    }
    std::cout << "classifier backend=" << classifier.BackendName()
              << " artifact_id=" << classifier.ArtifactId()
              << " artifact_sha256=" << classifier.ArtifactSha256()
              << " artifact_items=" << classifier.ArtifactItemCount()
              << " working_memory_bytes=" << classifier.WorkingMemoryBytes() << '\n';

    for (int argument = first_roi; argument < argc; ++argument) {
        ls2k::port::MlGrayRoi32 roi{};
        if (!LoadRoi(argv[argument], roi)) {
            std::cerr << "ROI must contain exactly " << roi.gray.size()
                      << " bytes path=" << argv[argument] << '\n';
            return 4;
        }
        std::uint64_t total_ns = 0;
        ls2k::port::MlClassifierOutput expected{};
        for (int iteration = 0; iteration < iterations; ++iteration) {
            const auto start = Clock::now();
            const auto output = classifier.Predict(roi);
            total_ns += static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count());
            if (!output.classification.valid) {
                std::cerr << "classification failed path=" << argv[argument]
                          << " reason=" << output.classification.reason << '\n';
                return 5;
            }
            if (iteration == 0) {
                expected = output;
            } else if (output.classification.class_id != expected.classification.class_id ||
                       output.classification.margin != expected.classification.margin ||
                       output.classification.class_scores != expected.classification.class_scores ||
                       output.tflite_feature.valid != expected.tflite_feature.valid ||
                       output.tflite_feature.values != expected.tflite_feature.values) {
                std::cerr << "classification parity failed path=" << argv[argument] << '\n';
                return 6;
            }
        }
        std::cout << std::fixed << std::setprecision(3)
                  << "result path=" << argv[argument]
                  << " raw_class=" << expected.classification.class_id
                  << " class_name=" << ClassName(expected.classification.class_id)
                  << " margin=" << expected.classification.margin
                  << " score0=" << expected.classification.class_scores[0]
                  << " score1=" << expected.classification.class_scores[1]
                  << " score2=" << expected.classification.class_scores[2]
                  << " feature_valid=" << (expected.tflite_feature.valid ? "true" : "false");
        if (expected.tflite_feature.valid) {
            for (std::size_t index = 0; index < expected.tflite_feature.values.size(); ++index) {
                std::cout << " feature" << index << '='
                          << static_cast<int>(expected.tflite_feature.values[index]);
            }
        }
        std::cout
                  << " avg_us=" << total_ns / (static_cast<double>(iterations) * 1000.0)
                  << " parity=true\n";
    }
    return 0;
}
