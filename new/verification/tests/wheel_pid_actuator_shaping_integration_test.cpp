#include <cmath>
#include <cstdlib>
#include <iostream>
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

ls2k::port::WheelPidParameters Params(double p, double i) {
    ls2k::port::WheelPidParameters params{};
    params.p = p;
    params.i = i;
    params.d = 0.0;
    params.integral_limit = 1000.0;
    params.measurement_filter_alpha = 1.0;
    return params;
}

ls2k::port::ActuatorCommand Command(int pwm) {
    return {pwm, pwm, 0, 0, false};
}

ls2k::control::WheelPidCommitResult EvaluateShapeCommit(
    ls2k::control::WheelPidController& controller,
    double error,
    int pwm_limit,
    int previous_pwm,
    int pwm_floor,
    bool prohibit_reverse,
    int step_limit,
    ls2k::control::WheelPidComputation* computation_out = nullptr,
    ls2k::control::ActuatorCommandShapingResult* shaping_out = nullptr) {
    const auto computation = controller.Evaluate(error, 0.0, pwm_limit);
    const auto shaping = ls2k::control::ShapeActuatorCommand(
        Command(previous_pwm),
        Command(computation.requested_pwm),
        pwm_limit,
        pwm_floor,
        prohibit_reverse,
        step_limit);
    if (computation_out != nullptr) {
        *computation_out = computation;
    }
    if (shaping_out != nullptr) {
        *shaping_out = shaping;
    }
    return controller.CommitAppliedOutput(computation, shaping.left, true);
}

void SeedIntegral(ls2k::control::WheelPidController& controller, double integral) {
    controller.Configure(Params(0.0, 0.0));
    const auto commit = EvaluateShapeCommit(controller, integral, 8000, 0, 0, false, 8000);
    ExpectNear(commit.integral, integral, "seed integral");
}

void TestPositiveFloorAllowsReviewerExampleToUnwindThroughZero() {
    ls2k::control::WheelPidController controller;
    SeedIntegral(controller, 420.0);
    controller.Configure(Params(100.0, 5.0));

    int previous_pwm = 2000;
    for (int expected_integral = 419; expected_integral >= 20; --expected_integral) {
        ls2k::control::WheelPidComputation computation{};
        ls2k::control::ActuatorCommandShapingResult shaping{};
        const auto commit = EvaluateShapeCommit(
            controller, -1.0, 8000, previous_pwm, 2000, false, 8000, &computation, &shaping);
        if (expected_integral == 419) {
            ExpectNear(computation.unconstrained_output, 1995.0,
                       "reviewer positive-floor unconstrained output");
            Require(shaping.left.applied_pwm == 2000 && shaping.left.floor_adjusted,
                    "reviewer positive-floor shaping fact");
        }
        Require(!commit.anti_windup_active, "positive floor must allow negative integral push");
        ExpectNear(commit.integral, expected_integral, "positive-floor continuous unwind");
        previous_pwm = shaping.left.applied_pwm;
    }
    Require(previous_pwm == 0, "positive floor sequence must reach the exact zero request");

    const auto crossed = EvaluateShapeCommit(controller, -1.0, 8000, previous_pwm, 2000, false, 8000);
    Require(!crossed.anti_windup_active, "positive floor must allow integration through zero");
    ExpectNear(crossed.integral, 19.0, "positive floor crossed integral");
}

void TestNegativeFloorAllowsSymmetricExampleToUnwindThroughZero() {
    ls2k::control::WheelPidController controller;
    SeedIntegral(controller, -420.0);
    controller.Configure(Params(100.0, 5.0));

    int previous_pwm = -2000;
    for (int expected_integral = -419; expected_integral <= -20; ++expected_integral) {
        ls2k::control::WheelPidComputation computation{};
        ls2k::control::ActuatorCommandShapingResult shaping{};
        const auto commit = EvaluateShapeCommit(
            controller, 1.0, 8000, previous_pwm, 2000, false, 8000, &computation, &shaping);
        if (expected_integral == -419) {
            ExpectNear(computation.unconstrained_output, -1995.0,
                       "reviewer negative-floor unconstrained output");
            Require(shaping.left.applied_pwm == -2000 && shaping.left.floor_adjusted,
                    "reviewer negative-floor shaping fact");
        }
        Require(!commit.anti_windup_active, "negative floor must allow positive integral push");
        ExpectNear(commit.integral, expected_integral, "negative-floor continuous unwind");
        previous_pwm = shaping.left.applied_pwm;
    }
    Require(previous_pwm == 0, "negative floor sequence must reach the exact zero request");

    const auto crossed = EvaluateShapeCommit(controller, 1.0, 8000, previous_pwm, 2000, false, 8000);
    Require(!crossed.anti_windup_active, "negative floor must allow integration through zero");
    ExpectNear(crossed.integral, -19.0, "negative floor crossed integral");
}

void TestRealShapingFactsStillFreezeTrueWindup() {
    {
        ls2k::control::WheelPidController controller;
        controller.Configure(Params(0.0, 10.0));
        const auto commit = EvaluateShapeCommit(controller, 100.0, 100, 0, 0, false, 1000);
        Require(commit.anti_windup_active, "hard-limit positive windup must freeze");
        ExpectNear(commit.integral, 0.0, "hard-limit integral");
    }
    {
        ls2k::control::WheelPidController controller;
        SeedIntegral(controller, 25.0);
        controller.Configure(Params(0.0, 10.0));
        ls2k::control::ActuatorCommandShapingResult shaping{};
        const auto commit = EvaluateShapeCommit(
            controller, 100.0, 8000, 0, 0, false, 100, nullptr, &shaping);
        Require(shaping.left.step_limited && shaping.left.positive_pid_output_blocked,
                "real step shaping must report positive blockage");
        Require(commit.anti_windup_active, "step-limit positive windup must activate anti-windup");
        ExpectNear(commit.integral, 0.0, "step-limit anti-windup must clear seeded integral");
    }
    {
        ls2k::control::WheelPidController controller;
        controller.Configure(Params(0.0, 1.0));
        ls2k::control::ActuatorCommandShapingResult shaping{};
        const auto commit = EvaluateShapeCommit(
            controller, -20.0, 8000, 0, 0, true, 8000, nullptr, &shaping);
        Require(shaping.left.reverse_suppressed && shaping.left.negative_pid_output_blocked,
                "real reverse shaping must report negative blockage");
        Require(commit.anti_windup_active, "reverse-suppressed negative windup must freeze");
        ExpectNear(commit.integral, 0.0, "reverse-suppressed integral");
    }
}

}  // namespace

int main() {
    try {
        TestPositiveFloorAllowsReviewerExampleToUnwindThroughZero();
        TestNegativeFloorAllowsSymmetricExampleToUnwindThroughZero();
        TestRealShapingFactsStillFreezeTrueWindup();
    } catch (const std::exception& error) {
        std::cerr << "wheel_pid_actuator_shaping_integration_test failed: " << error.what() << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "wheel_pid_actuator_shaping_integration_test passed\n";
    return EXIT_SUCCESS;
}
