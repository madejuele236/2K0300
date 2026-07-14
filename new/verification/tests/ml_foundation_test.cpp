#include <array>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "port/runtime_parameter_validation.hpp"
#include "vision/image/color_sampler.hpp"
#include "vision/ml/v9_descriptor.hpp"
#include "vision/ml/v9_replay.hpp"
#include "vision/ml/ml_class_mapping.hpp"
#include "vision/ml/selected_ml_classifier.hpp"

namespace {
void Expect(bool value, const char* message) { if (!value) throw std::runtime_error(message); }

int ReferencePopcount(std::uint8_t value) {
    int count = 0;
    while (value != 0U) {
        value = static_cast<std::uint8_t>(value & static_cast<std::uint8_t>(value - 1U));
        ++count;
    }
    return count;
}

ls2k::port::V9ReplayResult ReferenceReplay(
    const ls2k::port::V9Descriptor& query,
    const ls2k::port::V9ArtifactView& artifact) {
    ls2k::port::V9ReplayResult out{};
    if (!query.valid || !ls2k::vision::ml::ValidateV9Artifact(artifact)) return out;
    std::array<int, 3> best{std::numeric_limits<int>::max(),
                            std::numeric_limits<int>::max(),
                            std::numeric_limits<int>::max()};
    std::array<int, 3> index{-1, -1, -1};
    for (std::size_t prototype = 0; prototype < artifact.prototype_count; ++prototype) {
        const int cls = artifact.template_parent[prototype];
        int distance = 0;
        for (std::size_t byte = 0; byte < artifact.byte_count; ++byte) {
            distance += ReferencePopcount(static_cast<std::uint8_t>(
                query.bytes[byte] ^ artifact.template_codes[prototype * artifact.byte_count + byte]));
        }
        if (distance < best[static_cast<std::size_t>(cls)]) {
            best[static_cast<std::size_t>(cls)] = distance;
            index[static_cast<std::size_t>(cls)] = static_cast<int>(prototype);
        }
    }
    int predicted = 0;
    if (best[1] < best[predicted]) predicted = 1;
    if (best[2] < best[predicted]) predicted = 2;
    int second = std::numeric_limits<int>::max();
    for (int cls = 0; cls < 3; ++cls) {
        if (cls != predicted && best[static_cast<std::size_t>(cls)] < second) {
            second = best[static_cast<std::size_t>(cls)];
        }
    }
    out.valid = index[static_cast<std::size_t>(predicted)] >= 0 &&
                second != std::numeric_limits<int>::max();
    out.class_id = predicted;
    out.best_distance = best[static_cast<std::size_t>(predicted)];
    out.margin = out.valid ? second - out.best_distance : 0;
    out.prototype_index = index[static_cast<std::size_t>(predicted)];
    return out;
}

bool SameReplay(const ls2k::port::V9ReplayResult& first,
                const ls2k::port::V9ReplayResult& second) {
    return first.valid == second.valid && first.class_id == second.class_id &&
           first.best_distance == second.best_distance && first.margin == second.margin &&
           first.prototype_index == second.prototype_index;
}

void TestDisabledAndEnabledValidation() {
    ls2k::port::RuntimeParameters params{};
    Expect(ls2k::port::ValidateMlParameters(params.ml, params.motion_odometry),
           "disabled default ML parameters must validate");
    params.ml.enabled = true;
    Expect(!ls2k::port::ValidateMlParameters(params.ml, params.motion_odometry),
           "enabled ML must reject zero calibration");
    auto& roi = params.ml.roi;
    roi.search_forward_min_m = 0.1; roi.search_forward_max_m = 1.0;
    roi.search_lateral_limit_m = 0.5;
    roi.grid_forward_step_m = 0.02; roi.grid_lateral_step_m = 0.02;
    roi.expected_long_edge_m = 0.2; roi.expected_short_edge_m = 0.1;
    roi.long_edge_tolerance_m = 0.05; roi.short_edge_tolerance_m = 0.03;
    roi.max_long_edge_to_lateral_rad = 0.3; roi.min_component_cells = 4;
    roi.min_rectangularity = 0.5; roi.min_red_fill_ratio = 0.5;
    params.motion_odometry.encoder_ticks_to_meter = 0.001;
    auto& maneuver = params.ml.maneuver;
    maneuver.speed_target = 100; maneuver.exit_forward_m = 0.5;
    maneuver.max_duration_ms = 1000; maneuver.max_integration_gap_ms = 20;
    Expect(ls2k::port::ValidateMlParameters(params.ml, params.motion_odometry),
           "complete enabled ML parameters must validate");
    std::swap(roi.expected_long_edge_m, roi.expected_short_edge_m);
    Expect(!ls2k::port::ValidateMlParameters(params.ml, params.motion_odometry),
           "enabled ML must reject inverted expected long/short edges");
    std::swap(roi.expected_long_edge_m, roi.expected_short_edge_m);
    params.ml.class_mapping.class_1_action = "invalid";
    Expect(!ls2k::port::ValidateMlParameters(params.ml, params.motion_odometry),
           "invalid action token must fail independently of mapping duplicates");
}

void TestYuyvSampling() {
    const std::array<std::uint8_t, 4> pixels{10, 128, 30, 128};
    ls2k::port::CameraPixelFrameView frame{};
    frame.valid = true; frame.format = ls2k::port::CameraFrameFormat::kYuyv;
    frame.data = pixels.data(); frame.width = 2; frame.height = 1; frame.stride = 4;
    ls2k::vision::CameraColorSample sample{};
    Expect(ls2k::vision::SampleColorAt(frame, 0.0F, 0.5F, sample), "sample should succeed");
    Expect(sample.y == 20 && sample.u == 128 && sample.v == 128, "bilinear YUYV sample mismatch");
    std::uint8_t gray = 0;
    Expect(ls2k::vision::SampleRgbMeanAt(frame, 0.0F, 0.5F, gray) && gray == 20,
           "neutral YUYV RGB mean mismatch");
    Expect(ls2k::vision::SampleColorAt(frame, 0.0F, 1.0F, sample) && sample.y == 30,
           "last even-width YUYV pixel must remain sampleable");
    frame.width = 1;
    frame.stride = 4;
    Expect(!ls2k::vision::SampleColorAt(frame, 0.0F, 0.0F, sample),
           "odd-width YUYV layout must be rejected before shared chroma access");
}

void TestDescriptorReplayAcceptanceAndMapping() {
    ls2k::port::MlGrayRoi32 roi{};
    roi.valid = true;
    for (int row = 0; row < 32; ++row)
        for (int col = 0; col < 32; ++col)
            roi.gray[static_cast<std::size_t>(row * 32 + col)] = static_cast<std::uint8_t>(row * 5 + col * 2);
    const auto descriptor = ls2k::vision::ml::BuildV9Descriptor(roi);
    Expect(descriptor.valid && (descriptor.bytes[15] & 0xC0U) == 0U,
           "descriptor shape/trailing bits mismatch");
    // Generated independently with the training-side fixed-point definition:
    // image_qbits=16, basis_qbits=15, 8x8 sums of 4x4 patches, little-endian bits.
    const std::array<std::uint8_t, 16> python_golden{
        0x00, 0xff, 0x05, 0x54, 0x66, 0x09, 0x7c, 0xd6,
        0xea, 0xc1, 0xc1, 0x49, 0xc1, 0x8b, 0x4f, 0x17};
    Expect(descriptor.bytes == python_golden,
           "fixed descriptor diverged from training-side Python golden");
    std::array<std::uint8_t, 48> codes{};
    std::copy(descriptor.bytes.begin(), descriptor.bytes.end(), codes.begin() + 16);
    codes[0] = 0xff; codes[32] = 0x0f;
    const std::array<std::uint8_t, 3> parents{0, 1, 2};
    const ls2k::port::V9ArtifactView artifact{codes.data(), parents.data(), 3, 16, 126};
    const auto result = ls2k::vision::ml::ReplayV9Descriptor(descriptor, artifact);
    Expect(result.valid && result.class_id == 1 && result.best_distance == 0,
           "replay must select exact class-1 prototype");
    ls2k::port::MlV9Parameters acceptance{};
    acceptance.min_margin = 1; acceptance.max_best_distance = 0; acceptance.confirm_frames = 2;
    ls2k::port::V9AcceptanceState state{};
    Expect(!ls2k::vision::ml::StepV9Acceptance(result, acceptance, state).accepted &&
           ls2k::vision::ml::StepV9Acceptance(result, acceptance, state).accepted,
           "acceptance confirmation mismatch");
    ls2k::port::MlClassificationResult classification{};
    classification.valid = true;
    classification.class_id = 1;
    classification.margin = result.margin;
    classification.distance_valid = true;
    classification.best_distance = result.best_distance;
    Expect(ls2k::vision::ml::AcceptMlClassification(classification, acceptance),
           "generic acceptance must preserve V9 distance and margin gates");
    classification.distance_valid = false;
    classification.best_distance = 126;
    Expect(ls2k::vision::ml::AcceptMlClassification(classification, acceptance),
           "non-distance classifier must not inherit the V9 Hamming distance gate");
    ls2k::port::MlClassMappingParameters mapping{};
    Expect(ls2k::vision::ml::MapMlClass(0, mapping) == ls2k::port::MlAction::kStraight &&
           ls2k::vision::ml::MapMlClass(1, mapping) == ls2k::port::MlAction::kLeft &&
           ls2k::vision::ml::MapMlClass(2, mapping) == ls2k::port::MlAction::kRight,
           "default class mapping mismatch");
}

void TestReplayWordParity() {
    constexpr std::size_t kPrototypeCount = 96;
    std::array<std::uint8_t, kPrototypeCount * 16U> codes{};
    std::array<std::uint8_t, kPrototypeCount> parents{};
    std::uint32_t state = 0x7a31c59dU;
    auto next_byte = [&state]() {
        state = state * 1664525U + 1013904223U;
        return static_cast<std::uint8_t>(state >> 24U);
    };
    for (std::size_t prototype = 0; prototype < kPrototypeCount; ++prototype) {
        parents[prototype] = static_cast<std::uint8_t>(prototype % 3U);
        for (std::size_t byte = 0; byte < 16U; ++byte) {
            codes[prototype * 16U + byte] = next_byte();
        }
        codes[prototype * 16U + 15U] &= 0x3fU;
    }
    const ls2k::port::V9ArtifactView artifact{
        codes.data(), parents.data(), kPrototypeCount, 16U, 126};
    for (int query_index = 0; query_index < 128; ++query_index) {
        ls2k::port::V9Descriptor query{};
        query.valid = true;
        for (std::uint8_t& byte : query.bytes) byte = next_byte();
        query.bytes[15] &= 0x3fU;
        const auto expected = ReferenceReplay(query, artifact);
        const auto actual = ls2k::vision::ml::ReplayV9Descriptor(query, artifact);
        Expect(SameReplay(actual, expected),
               "64-bit replay distance/class/tie-break diverged from bytewise reference");
    }
}
}

int main() {
    try {
        TestDisabledAndEnabledValidation();
        TestYuyvSampling();
        TestDescriptorReplayAcceptanceAndMapping();
        TestReplayWordParity();
    } catch (const std::exception& error) {
        std::cerr << "ml_foundation_test failed: " << error.what() << '\n';
        return 1;
    }
    std::cout << "ml_foundation_test passed\n";
    return 0;
}
