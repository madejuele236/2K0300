#include <iostream>
#include <stdexcept>

#include "port/runtime_parameter_validation.hpp"

namespace {

void Expect(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

ls2k::port::ReferenceTimeAlignmentParameters EnabledParams() {
    ls2k::port::ReferenceTimeAlignmentParameters params{};
    params.enabled = true;
    params.max_age_ms = 120;
    params.effective_delay_ms = 0;
    params.future_prediction_max_ms = 80;
    params.max_integration_gap_ms = 30;
    params.min_aligned_samples = 3;
    params.use_encoder_forward = false;
    params.wheel_track_m = 0.0;
    params.use_imu_yaw = true;
    params.use_wheel_yaw_fallback = false;
    params.future_prediction_enabled = false;
    params.command_yaw_prediction_enabled = false;
    params.turn_output_to_yaw_rate_gain = 0.0;
    params.actuator_yaw_tau_ms = 35.0;
    params.max_delta_forward_m = 0.60;
    params.max_delta_lateral_m = 0.40;
    params.max_delta_yaw_rad = 0.80;
    return params;
}

ls2k::port::MotionOdometryParameters UncalibratedOdometry() {
    return {};
}

void TestDisabledAllowsUncalibratedOptionalSources() {
    auto params = EnabledParams();
    params.enabled = false;
    params.use_encoder_forward = true;
    params.use_wheel_yaw_fallback = true;
    params.command_yaw_prediction_enabled = true;
    Expect(ls2k::port::ValidateReferenceTimeAlignmentParameters(
               params, UncalibratedOdometry()),
           "disabled alignment should allow uncalibrated optional source fields");
}

void TestEnabledEncoderForwardRequiresScale() {
    auto params = EnabledParams();
    params.use_encoder_forward = true;
    auto odometry = UncalibratedOdometry();
    Expect(!ls2k::port::ValidateReferenceTimeAlignmentParameters(params, odometry),
           "enabled encoder-forward integration must require encoder scale at load time");

    odometry.encoder_ticks_to_meter = 0.001;
    Expect(ls2k::port::ValidateReferenceTimeAlignmentParameters(params, odometry),
           "encoder-forward integration should accept calibrated encoder scale");
}

void TestEnabledWheelYawFallbackRequiresScaleAndTrack() {
    auto params = EnabledParams();
    params.use_wheel_yaw_fallback = true;
    auto odometry = UncalibratedOdometry();
    odometry.encoder_ticks_to_meter = 0.001;
    params.wheel_track_m = 0.0;
    Expect(!ls2k::port::ValidateReferenceTimeAlignmentParameters(params, odometry),
           "wheel-yaw fallback must require wheel track at load time");

    params.wheel_track_m = 0.50;
    Expect(ls2k::port::ValidateReferenceTimeAlignmentParameters(params, odometry),
           "wheel-yaw fallback should accept calibrated encoder scale and wheel track");
}

void TestEnabledCommandYawPredictionRequiresFuturePredictionAndGain() {
    auto params = EnabledParams();
    params.command_yaw_prediction_enabled = true;
    params.future_prediction_enabled = false;
    params.turn_output_to_yaw_rate_gain = 0.01;
    Expect(!ls2k::port::ValidateReferenceTimeAlignmentParameters(
               params, UncalibratedOdometry()),
           "command-yaw prediction must require future prediction");

    params.future_prediction_enabled = true;
    params.turn_output_to_yaw_rate_gain = 0.0;
    Expect(!ls2k::port::ValidateReferenceTimeAlignmentParameters(
               params, UncalibratedOdometry()),
           "command-yaw prediction must require a calibrated yaw gain");

    params.turn_output_to_yaw_rate_gain = 0.01;
    Expect(ls2k::port::ValidateReferenceTimeAlignmentParameters(
               params, UncalibratedOdometry()),
           "command-yaw prediction should accept future prediction and calibrated yaw gain");
}

void TestTfliteIdentitySquaredL2ValidationRange() {
    ls2k::port::MlParameters params{};
    const auto odometry = UncalibratedOdometry();
    params.tflite_identity.min_margin = 260100;
    params.tflite_identity.max_best_distance = 260100;
    Expect(ls2k::port::ValidateMlParameters(params, odometry),
           "d4 squared-L2 mathematical maximum should validate");
    params.tflite_identity.max_best_distance = 260101;
    Expect(!ls2k::port::ValidateMlParameters(params, odometry),
           "distance above d4 squared-L2 mathematical maximum must fail");
    params.tflite_identity.max_best_distance = 2076;
    params.tflite_identity.min_margin = 260101;
    Expect(!ls2k::port::ValidateMlParameters(params, odometry),
           "margin above d4 squared-L2 mathematical maximum must fail");
}

}  // namespace

int main() {
    try {
        TestDisabledAllowsUncalibratedOptionalSources();
        TestEnabledEncoderForwardRequiresScale();
        TestEnabledWheelYawFallbackRequiresScaleAndTrack();
        TestEnabledCommandYawPredictionRequiresFuturePredictionAndGain();
        TestTfliteIdentitySquaredL2ValidationRange();
    } catch (const std::exception& error) {
        std::cerr << "runtime_parameter_validation_test failed: " << error.what() << "\n";
        return 1;
    }
    std::cout << "runtime_parameter_validation_test passed\n";
    return 0;
}
