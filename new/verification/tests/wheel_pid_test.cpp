#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "control/actuator_command_builder.hpp"
#include "control/wheel_pid.hpp"

namespace {

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void ExpectNear(double actual, double expected, const std::string& message) {
    if (std::abs(actual - expected) > 1.0e-9) {
        throw std::runtime_error(message);
    }
}

ls2k::control::WheelPidController MakeIntegralController(double integral_gain = 1.0) {
    ls2k::port::WheelPidParameters params{};
    params.p = 0.0;
    params.i = integral_gain;
    params.d = 0.0;
    params.integral_limit = 1000.0;
    params.measurement_filter_alpha = 1.0;
    ls2k::control::WheelPidController controller;
    controller.Configure(params);
    return controller;
}

ls2k::control::DrivePwmShapingChannel Applied(int pwm) {
    ls2k::control::DrivePwmShapingChannel shaping{};
    shaping.requested_pwm = pwm;
    shaping.desired_pwm = pwm;
    shaping.applied_pwm = pwm;
    return shaping;
}

ls2k::control::WheelPidController MakeFilterController(double alpha) {
    ls2k::port::WheelPidParameters params{};
    params.p = 1.0;
    params.i = 0.0;
    params.d = 0.0;
    params.integral_limit = 1000.0;
    params.measurement_filter_alpha = alpha;
    ls2k::control::WheelPidController controller;
    controller.Configure(params);
    return controller;
}

void TestEvaluateDoesNotCommitIntegral() {
    auto controller = MakeIntegralController();
    const auto computation = controller.Evaluate(10.0, 0.0, 8000);
    ExpectNear(controller.integral(), 0.0, "Evaluate must not mutate committed integral");
    ExpectNear(computation.candidate_integral, 10.0, "candidate integral");
}

void TestUnconstrainedRoundedOutputCommitsNormally() {
    auto controller = MakeIntegralController(0.33);
    const auto computation = controller.Evaluate(1.0, 0.0, 8000);
    Require(computation.requested_pwm == 0, "test requires ordinary rounding to zero");
    const auto commit = controller.CommitAppliedOutput(computation, Applied(0), true);
    Require(!commit.anti_windup_active, "PWM quantization alone must not freeze integration");
    ExpectNear(commit.integral, 1.0, "nominal committed integral");
}

void TestGlobalStepConstraintClearsIntegral() {
    auto controller = MakeIntegralController(10.0);
    const auto seed = controller.Evaluate(10.0, 0.0, 8000);
    const auto seed_commit = controller.CommitAppliedOutput(seed, Applied(100), true);
    ExpectNear(seed_commit.integral, 10.0, "step-constrained seed integral");

    const auto computation = controller.Evaluate(100.0, 0.0, 8000);
    auto shaping = Applied(100);
    shaping.step_limited = true;
    shaping.positive_pid_output_blocked = true;
    const auto commit = controller.CommitAppliedOutput(computation, shaping, true);
    Require(commit.anti_windup_active, "step-constrained output must activate anti-windup");
    Require(commit.reason == ls2k::control::WheelPidAntiWindupReason::kActuatorLimit,
            "step-constrained reason");
    ExpectNear(commit.integral, 0.0, "step-constrained anti-windup must clear integral");
}

void TestReverseSuppressionFreezesNegativeWindup() {
    auto controller = MakeIntegralController();
    const auto seed = controller.Evaluate(10.0, 0.0, 8000);
    const auto seed_commit = controller.CommitAppliedOutput(seed, Applied(10), true);
    ExpectNear(seed_commit.integral, 10.0, "reverse-suppressed seed integral");

    const auto computation = controller.Evaluate(-20.0, 0.0, 8000);
    auto shaping = Applied(0);
    shaping.negative_pid_output_blocked = true;
    const auto commit = controller.CommitAppliedOutput(computation, shaping, true);
    Require(commit.anti_windup_active, "reverse-suppressed output must freeze negative windup");
    ExpectNear(commit.integral, 10.0, "reverse-suppressed integral must remain unchanged");
}

void TestIntegrationCanUnwindWhileActuatorRemainsLimited() {
    auto controller = MakeIntegralController();
    const auto seed = controller.Evaluate(100.0, 0.0, 8000);
    const auto seed_commit = controller.CommitAppliedOutput(seed, Applied(100), true);
    ExpectNear(seed_commit.integral, 100.0, "seed integral");

    const auto unwind = controller.Evaluate(-10.0, 0.0, 8000);
    auto shaping = Applied(50);
    shaping.positive_pid_output_blocked = true;
    const auto commit = controller.CommitAppliedOutput(unwind, shaping, true);
    Require(!commit.anti_windup_active,
            "integration that moves output back toward applied PWM must be allowed");
    ExpectNear(commit.integral, 90.0, "unwound integral");
}

void TestHardSaturationFreezesWindup() {
    auto controller = MakeIntegralController(10.0);
    const auto seed = controller.Evaluate(10.0, 0.0, 8000);
    const auto seed_commit = controller.CommitAppliedOutput(seed, Applied(100), true);
    ExpectNear(seed_commit.integral, 10.0, "hard-saturated seed integral");

    const auto computation = controller.Evaluate(100.0, 0.0, 100);
    Require(computation.hard_saturated && computation.requested_pwm == 100,
            "test requires hard saturation");
    const auto commit = controller.CommitAppliedOutput(computation, Applied(100), true);
    Require(commit.anti_windup_active, "hard saturation must freeze windup direction");
    ExpectNear(commit.integral, 10.0, "hard-saturated integral must remain unchanged");
}

void TestHardSaturationAllowsDesaturationDirection() {
    auto controller = MakeIntegralController();
    const auto seed = controller.Evaluate(100.0, 0.0, 8000);
    const auto seed_commit = controller.CommitAppliedOutput(seed, Applied(100), true);
    ExpectNear(seed_commit.integral, 100.0, "hard-desaturation seed integral");

    const auto computation = controller.Evaluate(-10.0, 0.0, 50);
    Require(computation.hard_saturated && computation.requested_pwm == 50,
            "test requires a positive hard limit during negative integral push");
    const auto commit = controller.CommitAppliedOutput(computation, Applied(50), true);
    Require(!commit.anti_windup_active,
            "hard saturation must allow integration back toward the admissible range");
    ExpectNear(commit.integral, 90.0, "hard-desaturated integral");
}

void TestNotAppliedFreezesWithDistinctReason() {
    auto controller = MakeIntegralController();
    const auto computation = controller.Evaluate(10.0, 0.0, 8000);
    const auto commit = controller.CommitAppliedOutput(computation, Applied(0), false);
    Require(commit.anti_windup_active, "unapplied output must freeze integration");
    Require(commit.reason == ls2k::control::WheelPidAntiWindupReason::kNotApplied,
            "unapplied output must have a distinct reason");
    ExpectNear(commit.integral, 0.0, "unapplied integral must remain unchanged");
}

void TestAlphaZeroKeepsInitializedMeasurementHistory() {
    auto controller = MakeFilterController(0.0);
    const auto first = controller.Evaluate(0.0, 10.0, 8000);
    Require(first.valid, "alpha=0 first sample must be valid");
    ExpectNear(first.filtered_measured_speed, 10.0,
               "first sample must initialize filter history");
    const auto second = controller.Evaluate(0.0, 20.0, 8000);
    Require(second.valid, "alpha=0 second sample must be valid");
    ExpectNear(second.filtered_measured_speed, 10.0,
               "alpha=0 must keep initialized filter history");
    Require(second.requested_pwm == -10,
            "alpha=0 PID output must consume the retained measurement");
}

void TestAlphaOneUsesNewestMeasurement() {
    auto controller = MakeFilterController(1.0);
    const auto first = controller.Evaluate(0.0, 10.0, 8000);
    Require(first.valid, "alpha=1 first sample must be valid");
    const auto second = controller.Evaluate(0.0, 20.0, 8000);
    Require(second.valid, "alpha=1 second sample must be valid");
    ExpectNear(second.filtered_measured_speed, 20.0,
               "alpha=1 must use the newest measurement");
    Require(second.requested_pwm == -20,
            "alpha=1 PID output must consume the newest measurement");
}

void TestNonFinitePidArithmeticFailsExplicitly() {
    ls2k::port::WheelPidParameters params{};
    params.p = std::numeric_limits<double>::max();
    params.i = 0.0;
    params.d = 0.0;
    params.integral_limit = 1000.0;
    params.measurement_filter_alpha = 1.0;
    ls2k::control::WheelPidController controller;
    controller.Configure(params);

    const auto computation = controller.Evaluate(2.0, 0.0, 8000);
    Require(!computation.valid,
            "finite configuration whose multiplication overflows must fail explicitly");
    Require(computation.requested_pwm == 0 &&
                std::isfinite(computation.unconstrained_output),
            "invalid computation must not emit a non-finite or nonzero PWM request");
    const auto commit = controller.CommitAppliedOutput(computation, Applied(0), true);
    Require(commit.anti_windup_active &&
                commit.reason ==
                    ls2k::control::WheelPidAntiWindupReason::kInvalidComputation,
            "invalid computation must remain observable and freeze integration");
    ExpectNear(commit.integral, 0.0,
               "invalid computation must not commit candidate integral");
}

}  // namespace

int main() {
    try {
        TestEvaluateDoesNotCommitIntegral();
        TestUnconstrainedRoundedOutputCommitsNormally();
        TestGlobalStepConstraintClearsIntegral();
        TestReverseSuppressionFreezesNegativeWindup();
        TestIntegrationCanUnwindWhileActuatorRemainsLimited();
        TestHardSaturationFreezesWindup();
        TestHardSaturationAllowsDesaturationDirection();
        TestNotAppliedFreezesWithDistinctReason();
        TestAlphaZeroKeepsInitializedMeasurementHistory();
        TestAlphaOneUsesNewestMeasurement();
        TestNonFinitePidArithmeticFailsExplicitly();
    } catch (const std::exception& error) {
        std::cerr << "wheel_pid_test failed: " << error.what() << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "wheel_pid_test passed\n";
    return EXIT_SUCCESS;
}
