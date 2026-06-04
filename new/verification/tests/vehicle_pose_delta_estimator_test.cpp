#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdexcept>

#include "estimation/vehicle_pose_delta_estimator.hpp"

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

ls2k::port::ReferenceTimeAlignmentParameters Params() {
    ls2k::port::ReferenceTimeAlignmentParameters params{};
    params.enabled = true;
    params.max_age_ms = 120;
    params.max_integration_gap_ms = 30;
    params.use_encoder_forward = true;
    params.encoder_ticks_to_meter = 0.001;
    params.use_imu_yaw = true;
    params.use_wheel_yaw_fallback = false;
    params.max_delta_forward_m = 1.0;
    params.max_delta_lateral_m = 1.0;
    params.max_delta_yaw_rad = 1.0;
    params.future_prediction_enabled = false;
    return params;
}

ls2k::port::MotionHistory MakeHistory() {
    ls2k::port::MotionHistory history{};
    history.Push({100, true, 0.0F, true, 0, 0});
    history.Push({110, true, 0.0F, true, 10, 10});
    history.Push({120, true, 0.0F, true, 10, 10});
    history.Push({130, true, 0.0F, true, 10, 10});
    return history;
}

void TestEncoderForward() {
    const auto result = ls2k::estimation::EstimateVehiclePoseDelta(
        100,
        130,
        130,
        MakeHistory(),
        ls2k::port::ControlCommandHistory{},
        Params());

    Expect(result.valid, "encoder forward delta should be valid");
    ExpectNear(result.delta_forward_m, 0.03, 1.0e-6, "forward delta mismatch");
    ExpectNear(result.delta_lateral_m, 0.0, 1.0e-6, "lateral delta mismatch");
    ExpectNear(result.delta_yaw_rad, 0.0, 1.0e-6, "yaw delta mismatch");
    Expect(result.used_encoder_forward, "encoder forward source flag missing");
}

void TestImuYaw() {
    auto history = MakeHistory();
    for (std::size_t index = 0; index < history.count; ++index) {
        history.samples[index].gyro_z = 1.0F;
    }

    const auto result = ls2k::estimation::EstimateVehiclePoseDelta(
        100,
        130,
        130,
        history,
        ls2k::port::ControlCommandHistory{},
        Params());

    Expect(result.valid, "imu yaw delta should be valid");
    ExpectNear(result.delta_yaw_rad, 0.03, 1.0e-6, "yaw delta mismatch");
    Expect(result.used_imu_yaw, "imu yaw source flag missing");
}

void TestWheelYawFallback() {
    auto params = Params();
    params.use_imu_yaw = false;
    params.use_wheel_yaw_fallback = true;
    params.wheel_track_m = 0.5;
    auto history = MakeHistory();
    history.samples[1].left_encoder_delta = 5;
    history.samples[1].right_encoder_delta = 15;
    history.samples[2].left_encoder_delta = 5;
    history.samples[2].right_encoder_delta = 15;
    history.samples[3].left_encoder_delta = 5;
    history.samples[3].right_encoder_delta = 15;

    const auto result = ls2k::estimation::EstimateVehiclePoseDelta(
        100,
        130,
        130,
        history,
        ls2k::port::ControlCommandHistory{},
        params);

    Expect(result.valid, "wheel yaw fallback should be valid");
    ExpectNear(result.delta_yaw_rad, 0.06, 1.0e-6, "wheel yaw fallback mismatch");
    Expect(result.used_wheel_yaw, "wheel yaw source flag missing");
}

void TestBodyFrameLateralSignFollowsRightPositiveProtocol() {
    auto params = Params();
    params.encoder_ticks_to_meter = 0.1;
    params.max_delta_lateral_m = 1.0;
    auto right_turn_history = MakeHistory();
    for (std::size_t index = 0; index < right_turn_history.count; ++index) {
        right_turn_history.samples[index].gyro_z = 1.0F;
        right_turn_history.samples[index].left_encoder_delta = 1;
        right_turn_history.samples[index].right_encoder_delta = 1;
    }

    auto result = ls2k::estimation::EstimateVehiclePoseDelta(
        100,
        130,
        130,
        right_turn_history,
        ls2k::port::ControlCommandHistory{},
        params);
    Expect(result.valid, "positive yaw body-frame delta should be valid");
    Expect(result.delta_lateral_m > 0.0,
           "positive yaw with forward motion should accumulate right-positive lateral delta");

    auto left_turn_history = right_turn_history;
    for (std::size_t index = 0; index < left_turn_history.count; ++index) {
        left_turn_history.samples[index].gyro_z = -1.0F;
    }
    result = ls2k::estimation::EstimateVehiclePoseDelta(
        100,
        130,
        130,
        left_turn_history,
        ls2k::port::ControlCommandHistory{},
        params);
    Expect(result.valid, "negative yaw body-frame delta should be valid");
    Expect(result.delta_lateral_m < 0.0,
           "negative yaw with forward motion should accumulate left-negative lateral delta");
}

void TestFuturePredictionDisabledFailsWhenNeeded() {
    const auto result = ls2k::estimation::EstimateVehiclePoseDelta(
        100,
        130,
        150,
        MakeHistory(),
        ls2k::port::ControlCommandHistory{},
        Params());

    Expect(!result.valid, "future prediction disabled should fail for future end time");
    Expect(result.reason == "future_prediction_disabled",
           "future prediction disabled reason mismatch");
}

void TestFutureConstantVelocity() {
    auto params = Params();
    params.future_prediction_enabled = true;
    params.future_prediction_max_ms = 80;

    const auto result = ls2k::estimation::EstimateVehiclePoseDelta(
        100,
        130,
        150,
        MakeHistory(),
        ls2k::port::ControlCommandHistory{},
        params);

    Expect(result.valid, "future constant velocity prediction should be valid");
    ExpectNear(result.delta_forward_m, 0.05, 1.0e-6, "future forward prediction mismatch");
    Expect(result.predicted_ms == 20, "predicted_ms mismatch");
}

void TestCommandPredictionFiltersInvalidCommands() {
    auto params = Params();
    params.future_prediction_enabled = true;
    params.command_yaw_prediction_enabled = true;
    params.future_prediction_max_ms = 80;
    params.turn_output_to_yaw_rate_gain = 0.01;
    params.actuator_yaw_tau_ms = 0.0;

    ls2k::port::ControlCommandHistory commands{};
    ls2k::port::ControlCommandHistorySample invalid{};
    invalid.time_ms = 130;
    invalid.valid = true;
    invalid.actuator_applied = false;
    invalid.applied_turn_output = 100;
    commands.Push(invalid);

    auto result = ls2k::estimation::EstimateVehiclePoseDelta(
        100,
        130,
        150,
        MakeHistory(),
        commands,
        params);
    Expect(result.valid, "prediction without usable command should still be valid");
    Expect(!result.used_command_prediction, "invalid command must not drive prediction");
    ExpectNear(result.predicted_yaw_rate_radps, 0.0, 1.0e-6, "invalid command yaw mismatch");

    ls2k::port::ControlCommandHistorySample valid{};
    valid.time_ms = 130;
    valid.valid = true;
    valid.actuator_applied = true;
    valid.applied_turn_output = 100;
    commands.Push(valid);

    result = ls2k::estimation::EstimateVehiclePoseDelta(
        100,
        130,
        150,
        MakeHistory(),
        commands,
        params);
    Expect(result.valid, "prediction with usable command should be valid");
    Expect(result.used_command_prediction, "valid command must drive prediction");
    ExpectNear(result.predicted_yaw_rate_radps, 1.0, 1.0e-6, "command yaw prediction mismatch");
}

}  // namespace

int main() {
    try {
        TestEncoderForward();
        TestImuYaw();
        TestWheelYawFallback();
        TestBodyFrameLateralSignFollowsRightPositiveProtocol();
        TestFuturePredictionDisabledFailsWhenNeeded();
        TestFutureConstantVelocity();
        TestCommandPredictionFiltersInvalidCommands();
    } catch (const std::exception& error) {
        std::cerr << "vehicle_pose_delta_estimator_test failed: " << error.what() << "\n";
        return 1;
    }
    std::cout << "vehicle_pose_delta_estimator_test passed\n";
    return 0;
}
