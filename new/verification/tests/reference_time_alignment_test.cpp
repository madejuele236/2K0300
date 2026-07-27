#include <cmath>
#include <iostream>
#include <stdexcept>

#include "reference/reference_time_alignment.hpp"

namespace {

void Expect(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void ExpectNear(double actual, double expected, double tolerance, const char* message) {
    if (std::fabs(actual - expected) > tolerance) {
        throw std::runtime_error(message);
    }
}

ls2k::port::BEVReferencePath MakeStraightPath() {
    ls2k::port::BEVReferencePath path{};
    path.mode = ls2k::port::ReferenceMode::kIntervalCenter;
    for (std::size_t index = 0; index < 4; ++index) {
        auto& sample = path.sampled_path[index];
        sample.present = true;
        sample.point.forward_m = 0.2F + static_cast<float>(index) * 0.2F;
        sample.point.lateral_m = 0.0F;
        sample.confidence = 1.0F;
        sample.source = ls2k::port::BEVPathPointSource::kIntervalCenter;
    }
    return path;
}

ls2k::port::RuntimeParameters EnabledParams() {
    ls2k::port::RuntimeParameters params{};
    params.reference_time_alignment.enabled = true;
    params.reference_time_alignment.max_age_ms = 120;
    params.reference_time_alignment.max_integration_gap_ms = 30;
    params.reference_time_alignment.min_aligned_samples = 3;
    params.reference_time_alignment.max_delta_forward_m = 1.0;
    params.reference_time_alignment.max_delta_lateral_m = 1.0;
    params.reference_time_alignment.max_delta_yaw_rad = 0.8;
    return params;
}

ls2k::port::VehiclePoseDelta MakePoseDelta(double forward_m,
                                           double lateral_m,
                                           double yaw_rad,
                                           std::uint64_t start_ms = 100,
                                           std::uint64_t now_ms = 130,
                                           std::uint64_t end_ms = 130) {
    ls2k::port::VehiclePoseDelta delta{};
    delta.valid = true;
    delta.reason = "test";
    delta.start_time_ms = start_ms;
    delta.now_time_ms = now_ms;
    delta.end_time_ms = end_ms;
    delta.measured_until_ms = now_ms;
    delta.delta_forward_m = forward_m;
    delta.delta_lateral_m = lateral_m;
    delta.delta_yaw_rad = yaw_rad;
    delta.measured_forward_mps = 1.0;
    delta.measured_yaw_rate_radps = yaw_rad == 0.0 ? 0.0 : 1.0;
    delta.used_encoder_forward = forward_m != 0.0;
    delta.used_imu_yaw = yaw_rad != 0.0;
    return delta;
}

void TestDisabledKeepsPath() {
    const auto path = MakeStraightPath();
    const auto result = ls2k::reference::AlignReferencePathToVehiclePoseDelta(
        path,
        100,
        130,
        130,
        MakePoseDelta(0.1, 0.0, 0.0),
        ls2k::port::RuntimeParameters{});
    Expect(result.facts.valid, "disabled alignment must be valid");
    Expect(result.facts.reason == "disabled", "disabled reason mismatch");
    Expect(result.reference_path.sampled_path[0].present, "disabled path sample missing");
    ExpectNear(result.reference_path.sampled_path[0].point.forward_m, 0.2, 1.0e-6, "disabled forward changed");
}

void TestIdentityAndYawSign() {
    const auto path = MakeStraightPath();
    auto params = EnabledParams();
    auto zero = ls2k::reference::AlignReferencePathToVehiclePoseDelta(
        path,
        100,
        130,
        130,
        MakePoseDelta(0.0, 0.0, 0.0),
        params);
    Expect(zero.facts.valid, "zero delta alignment invalid");
    Expect(zero.facts.reason == "aligned_measured_se2", "zero delta reason mismatch");
    ExpectNear(zero.reference_path.sampled_path[2].point.forward_m,
               path.sampled_path[2].point.forward_m,
               1.0e-6,
               "identity forward mismatch");
    ExpectNear(zero.reference_path.sampled_path[2].point.lateral_m, 0.0, 1.0e-6, "identity lateral mismatch");

    auto yaw = ls2k::reference::AlignReferencePathToVehiclePoseDelta(
        path,
        100,
        130,
        130,
        MakePoseDelta(0.0, 0.0, 0.03),
        params);
    Expect(yaw.facts.valid, "yaw alignment invalid");
    ExpectNear(yaw.facts.delta_yaw_rad, 0.03, 1.0e-6, "yaw delta mismatch");
    Expect(yaw.reference_path.sampled_path[2].point.lateral_m < 0.0F,
           "positive yaw should rotate old forward point to negative lateral in current frame");
}

void TestForwardShift() {
    const auto path = MakeStraightPath();
    auto params = EnabledParams();
    params.reference_time_alignment.min_aligned_samples = 1;
    auto result = ls2k::reference::AlignReferencePathToVehiclePoseDelta(
        path,
        100,
        130,
        130,
        MakePoseDelta(0.1, 0.0, 0.0),
        params);
    Expect(result.facts.valid, "forward SE2 alignment invalid");
    ExpectNear(result.reference_path.sampled_path[0].point.forward_m,
               0.1,
               1.0e-6,
               "forward shift mismatch");
}

void TestLateralShift() {
    const auto path = MakeStraightPath();
    auto params = EnabledParams();
    auto result = ls2k::reference::AlignReferencePathToVehiclePoseDelta(
        path,
        100,
        130,
        130,
        MakePoseDelta(0.0, 0.05, 0.0),
        params);
    Expect(result.facts.valid, "lateral SE2 alignment invalid");
    ExpectNear(result.reference_path.sampled_path[0].point.lateral_m,
               -0.05,
               1.0e-6,
               "lateral shift mismatch");
}

void TestFutureEffectiveTimeFacts() {
    const auto path = MakeStraightPath();
    auto params = EnabledParams();
    auto delta = MakePoseDelta(0.1, 0.0, 0.0, 100, 130, 150);
    delta.predicted_ms = 20;
    delta.predicted_forward_mps = 1.0;
    auto result = ls2k::reference::AlignReferencePathToVehiclePoseDelta(
        path,
        100,
        130,
        150,
        delta,
        params);
    Expect(result.facts.valid, "effective time alignment invalid");
    Expect(result.facts.control_time_ms == 130, "control_time_ms mismatch");
    Expect(result.facts.control_effective_time_ms == 150, "control_effective_time_ms mismatch");
    Expect(result.facts.predicted_ms == 20, "predicted_ms mismatch");
    Expect(result.facts.reason == "aligned_effective_se2", "reason mismatch");
}

void TestFailClosed() {
    const auto path = MakeStraightPath();
    auto params = EnabledParams();
    params.reference_time_alignment.max_age_ms = 20;
    auto aged = ls2k::reference::AlignReferencePathToVehiclePoseDelta(
        path,
        100,
        130,
        130,
        MakePoseDelta(0.0, 0.0, 0.0),
        params);
    Expect(!aged.facts.valid && aged.facts.reason == "age_exceeded", "age fail reason mismatch");

    params = EnabledParams();
    auto delta = MakePoseDelta(0.0, 0.0, 0.0);
    delta.valid = false;
    delta.reason = "yaw_history_unavailable";
    auto unavailable = ls2k::reference::AlignReferencePathToVehiclePoseDelta(
        path,
        100,
        130,
        130,
        delta,
        params);
    Expect(!unavailable.facts.valid &&
               unavailable.facts.reason == "pose_delta_yaw_history_unavailable",
           "pose delta fail reason mismatch");

    params = EnabledParams();
    params.reference_time_alignment.max_delta_forward_m = 0.05;
    auto exceeded = ls2k::reference::AlignReferencePathToVehiclePoseDelta(
        path,
        100,
        130,
        130,
        MakePoseDelta(0.1, 0.0, 0.0),
        params);
    Expect(!exceeded.facts.valid && exceeded.facts.reason == "delta_forward_exceeded",
           "forward delta fail reason mismatch");
}

void TestInputReferenceGapOnlyRemovesThatSample() {
    auto path = MakeStraightPath();
    path.sampled_path[1].present = false;
    auto params = EnabledParams();
    params.reference_time_alignment.min_aligned_samples = 1;
    auto result = ls2k::reference::AlignReferencePathToVehiclePoseDelta(
        path,
        100,
        130,
        130,
        MakePoseDelta(0.0, 0.0, 0.0),
        params);
    Expect(result.facts.valid, "remaining finite samples should remain alignable");
    Expect(result.facts.aligned_sample_count == 3,
           "alignment must exclude only the missing input sample");
    Expect(result.reference_path.sampled_path[0].present,
           "leading observed prefix sample must remain present");
    Expect(result.reference_path.sampled_path[1].present &&
               std::abs(result.reference_path.sampled_path[1].point.forward_m - 0.60F) < 1.0e-6F,
           "the sample after a gap must be compacted with its geometry preserved");
}

}  // namespace

int main() {
    try {
        TestDisabledKeepsPath();
        TestIdentityAndYawSign();
        TestForwardShift();
        TestLateralShift();
        TestFutureEffectiveTimeFacts();
        TestFailClosed();
        TestInputReferenceGapOnlyRemovesThatSample();
    } catch (const std::exception& error) {
        std::cerr << "reference_time_alignment_test failed: " << error.what() << "\n";
        return 1;
    }
    std::cout << "reference_time_alignment_test passed\n";
    return 0;
}
