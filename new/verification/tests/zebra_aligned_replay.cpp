#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "platform/bootstrap.hpp"
#include "vision/bev/bev_simple_perception.hpp"
#include "vision/elements/zebra_element_evidence.hpp"
#include "vision/image/illumination_binary_model.hpp"

namespace {

struct Diagnostics final : ls2k::port::DiagnosticSink {
    void Emit(const ls2k::port::DiagnosticEvent&) override {}
};

enum class Label {
    kNegative,
    kZebra,
    kMaybe,
};

struct Sample {
    std::filesystem::path png_path{};
    Label label = Label::kNegative;
};

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

const char* LabelToken(Label label) {
    switch (label) {
        case Label::kNegative:
            return "negative";
        case Label::kZebra:
            return "zebra";
        case Label::kMaybe:
            return "maybe";
    }
    return "unknown";
}

std::vector<std::uint8_t> ReadGray8(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    Require(input.is_open(), "cannot open raw frame: " + path.string());
    const std::streamsize size = input.tellg();
    Require(size == 320 * 240,
            "expected aligned 320x240 gray8 frame: " + path.string());
    input.seekg(0);
    std::vector<std::uint8_t> gray(static_cast<std::size_t>(size));
    input.read(reinterpret_cast<char*>(gray.data()), size);
    Require(input.gcount() == size, "truncated frame: " + path.string());
    return gray;
}

std::vector<std::uint8_t> GrayToYuyv(const std::vector<std::uint8_t>& gray) {
    std::vector<std::uint8_t> yuyv(gray.size() * 2U, 128U);
    for (std::size_t index = 0U; index < gray.size(); ++index) {
        yuyv[index * 2U] = gray[index];
    }
    return yuyv;
}

void AddPngSamples(const std::filesystem::path& directory,
                   Label label,
                   bool recursive,
                   std::vector<Sample>& samples) {
    Require(std::filesystem::is_directory(directory),
            "missing label directory: " + directory.string());
    if (recursive) {
        for (const auto& entry :
             std::filesystem::recursive_directory_iterator(directory)) {
            if (entry.is_regular_file() && entry.path().extension() == ".png") {
                samples.push_back({entry.path(), label});
            }
        }
        return;
    }
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file() && entry.path().extension() == ".png") {
            samples.push_back({entry.path(), label});
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Require(argc == 4,
                "usage: zebra_aligned_replay STEERING_MEDIA_DIR PARAMS.json REPORT.tsv");
        const std::filesystem::path media_dir(argv[1]);
        const std::filesystem::path png_dir = media_dir / "png";
        const std::filesystem::path frames_dir = media_dir / "frames";

        std::vector<Sample> samples;
        AddPngSamples(png_dir, Label::kNegative, false, samples);
        AddPngSamples(png_dir / "zebra", Label::kZebra, false, samples);
        AddPngSamples(png_dir / "maybe-zebra", Label::kMaybe, false, samples);
        std::sort(samples.begin(), samples.end(), [](const Sample& lhs, const Sample& rhs) {
            if (lhs.label != rhs.label) {
                return static_cast<int>(lhs.label) < static_cast<int>(rhs.label);
            }
            return lhs.png_path.filename() < rhs.png_path.filename();
        });
        Require(!samples.empty(), "no labeled PNG samples found");

        Diagnostics diagnostics{};
        std::unique_ptr<ls2k::port::IParamStore> store =
            ls2k::platform::MakeParamStore();
        ls2k::port::RuntimeParameters params{};
        Require(store && store->LoadRuntimeParameters(argv[2], params, diagnostics),
                "failed to load runtime parameters");
        ls2k::vision::BEVProjector projector{};
        Require(projector.Configure(params.bev_projector),
                "projector configure failed");
        ls2k::vision::BEVSampleProjectionLut lut{};

        std::ofstream report(argv[3]);
        Require(report.is_open(), "cannot open report output");
        report << "label\tframe\tpresent\treason\tforward_min_m\tforward_max_m"
                  "\tlateral_min_m\tlateral_max_m\tjump_count\tsampleable_count\n";

        std::size_t negative_count = 0U;
        std::size_t positive_count = 0U;
        std::size_t maybe_count = 0U;
        std::size_t false_positive_count = 0U;
        std::size_t false_negative_count = 0U;
        std::size_t maybe_present_count = 0U;

        for (const Sample& sample : samples) {
            const std::filesystem::path raw_path =
                frames_dir / sample.png_path.filename().replace_extension(".raw");
            const std::vector<std::uint8_t> gray = ReadGray8(raw_path);
            std::vector<std::uint8_t> yuyv = GrayToYuyv(gray);
            ls2k::port::CameraPixelFrameView frame{};
            frame.valid = true;
            frame.format = ls2k::port::CameraFrameFormat::kYuyv;
            frame.data = yuyv.data();
            frame.width = 320;
            frame.height = 240;
            frame.stride = 640;

            ls2k::vision::BinaryModelTracker tracker{};
            const ls2k::port::BinaryModelState binary_model =
                tracker.Update(ls2k::vision::ComputeIlluminationBinaryModel(frame));
            Require(binary_model.valid,
                    "binary model invalid: " + raw_path.string());
            const ls2k::vision::BEVSimplePerceptionResult perception =
                ls2k::vision::RunBEVSimplePerception(
                    frame, binary_model, params, projector, &lut);
            const ls2k::port::VisualElementEvidenceRecord evidence =
                ls2k::vision::DetectZebraEvidence(perception.rows, params);

            report << LabelToken(sample.label) << '\t'
                   << sample.png_path.stem().string() << '\t'
                   << (evidence.present ? 1 : 0) << '\t'
                   << evidence.reason << '\t' << std::setprecision(9)
                   << evidence.bounds.forward_min_m << '\t'
                   << evidence.bounds.forward_max_m << '\t'
                   << evidence.bounds.lateral_min_m << '\t'
                   << evidence.bounds.lateral_max_m << '\t'
                   << evidence.support.boundary_jump_count << '\t'
                   << evidence.support.sampleable_count << '\n';

            if (sample.label == Label::kNegative) {
                ++negative_count;
                if (evidence.present) {
                    ++false_positive_count;
                    std::cout << "false_positive "
                              << sample.png_path.filename().string() << '\n';
                }
            } else if (sample.label == Label::kZebra) {
                ++positive_count;
                if (!evidence.present) {
                    ++false_negative_count;
                    std::cout << "false_negative "
                              << sample.png_path.filename().string() << ' '
                              << evidence.reason << '\n';
                }
            } else {
                ++maybe_count;
                maybe_present_count += evidence.present ? 1U : 0U;
            }
        }

        std::cout << "zebra_aligned_replay samples=" << samples.size()
                  << " negatives=" << negative_count
                  << " positives=" << positive_count
                  << " maybe=" << maybe_count
                  << " false_positives=" << false_positive_count
                  << " false_negatives=" << false_negative_count
                  << " maybe_present=" << maybe_present_count << '\n';
        Require(negative_count == 906U,
                "strict negative set must contain exactly 906 root PNG files");
        Require(positive_count == 7U,
                "strict positive set must contain exactly 7 Zebra PNG files");
        Require(maybe_count == 33U,
                "unscored maybe set must contain exactly 33 PNG files");
        Require(false_positive_count == 0U,
                "strict negative set contains Zebra false positives");
        Require(false_negative_count == 0U,
                "strict positive set contains Zebra false negatives");
    } catch (const std::exception& error) {
        std::cerr << "zebra_aligned_replay failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
