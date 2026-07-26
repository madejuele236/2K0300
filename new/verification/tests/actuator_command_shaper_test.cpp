#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "control/actuator_command_builder.hpp"

namespace {

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

ls2k::port::ActuatorCommand Command(int left,
                                    int right,
                                    int left_brushless = 0,
                                    int right_brushless = 0,
                                    bool emergency = false) {
    return {left, right, left_brushless, right_brushless, emergency};
}

void TestEveryOrdinaryDriveChangeUsesTheGlobalStepLimit() {
    const auto result = ls2k::control::ShapeActuatorCommand(
        Command(2000, -2000), Command(5000, -5000), 8000, 0, false, 1000);
    Require(result.command.left_drive_pwm == 3000, "left rise must be limited to one step");
    Require(result.command.right_drive_pwm == -3000, "right fall must be limited to one step");
    Require(result.left.step_limited && result.right.step_limited,
            "both constrained channels must report step_limited");
    Require(result.left.positive_pid_output_blocked &&
                result.right.negative_pid_output_blocked,
            "step owner must report the blocked PID directions");
}

void TestControlledStopRampsTowardZero() {
    const auto result = ls2k::control::ShapeActuatorCommand(
        Command(2500, 700), Command(0, 0), 8000, 0, true, 1000);
    Require(result.command.left_drive_pwm == 1500, "controlled stop left step");
    Require(result.command.right_drive_pwm == 0, "controlled stop right reaches zero");
    Require(result.left.step_limited, "left controlled stop must report limiting");
    Require(!result.right.step_limited, "right controlled stop reaches desired zero");
}

void TestAntiReverseChangesDesiredToZeroBeforeSlew() {
    const auto result = ls2k::control::ShapeActuatorCommand(
        Command(2400, 0), Command(-5000, -3000), 8000, 0, true, 1000);
    Require(result.left.requested_pwm == -5000 && result.right.requested_pwm == -3000,
            "raw hard-limited requests must remain observable");
    Require(result.left.desired_pwm == 0 && result.right.desired_pwm == 0,
            "reverse policy must produce zero desired PWM");
    Require(result.command.left_drive_pwm == 1400,
            "positive applied PWM must slew toward zero after reverse suppression");
    Require(result.command.right_drive_pwm == 0,
            "zero applied PWM must remain zero after reverse suppression");
    Require(result.left.reverse_suppressed && result.right.reverse_suppressed,
            "both negative requests must report reverse suppression");
    Require(result.left.negative_pid_output_blocked &&
                result.right.negative_pid_output_blocked,
            "reverse policy must report negative PID output as blocked");
}

void TestAllowedSignChangeStillUsesOneGlobalStep() {
    const auto result = ls2k::control::ShapeActuatorCommand(
        Command(1500, -1500), Command(-2500, 2500), 8000, 0, false, 1000);
    Require(result.command.left_drive_pwm == 500, "left sign change first step");
    Require(result.command.right_drive_pwm == -500, "right sign change first step");
}

void TestFloorPrecedesStepAndRemainsObservable() {
    const auto result = ls2k::control::ShapeActuatorCommand(
        Command(0, 0), Command(100, -100), 8000, 2000, false, 1000);
    Require(result.left.desired_pwm == 2000 && result.right.desired_pwm == -2000,
            "floor must define the desired PWM before slew");
    Require(result.command.left_drive_pwm == 1000 && result.command.right_drive_pwm == -1000,
            "floor-adjusted target must still obey the global step");
    Require(result.left.floor_adjusted && result.right.floor_adjusted,
            "floor adjustment must be observable");
    Require(!result.left.positive_pid_output_blocked &&
                !result.left.negative_pid_output_blocked &&
                !result.right.positive_pid_output_blocked &&
                !result.right.negative_pid_output_blocked,
            "floor plus slew must not claim a PID-direction constraint while applied exceeds request");
}

void TestBrushlessCommandsAreNotSlewLimited() {
    const auto result = ls2k::control::ShapeActuatorCommand(
        Command(0, 0, 500, 500), Command(5000, 5000, 900, 850), 8000, 0, true, 1000);
    Require(result.command.left_drive_pwm == 1000 && result.command.right_drive_pwm == 1000,
            "drive PWM must be stepped");
    Require(result.command.left_brushless_pwm == 900 && result.command.right_brushless_pwm == 850,
            "brushless ESC commands must pass through unchanged");
}

void TestEmergencyStopBypassesSlew() {
    const auto result = ls2k::control::ShapeActuatorCommand(
        Command(5000, 5000, 900, 900), Command(0, 0, 0, 0, true), 8000, 0, true, 1000);
    Require(result.command.emergency_stop, "emergency flag must be preserved");
    Require(result.command.left_drive_pwm == 0 && result.command.right_drive_pwm == 0,
            "emergency zero must bypass controlled slew");
    Require(!result.left.step_limited && !result.right.step_limited,
            "bypassed emergency command must not claim ordinary step limiting");
}

void TestInvalidStepLimitHoldsPreviousDriveCommand() {
    const auto result = ls2k::control::ShapeActuatorCommand(
        Command(1200, -800), Command(5000, 5000), 8000, 0, false, 0);
    Require(result.command.left_drive_pwm == 1200 && result.command.right_drive_pwm == -800,
            "non-positive step must fail closed by holding the previous drive PWM");
}

void TestHardwareDeadCycleBecomesAuthoritativeAppliedFact() {
    auto result = ls2k::control::ShapeActuatorCommand(
        Command(0, 0), Command(-300, 200), 8000, 0, false, 1000);
    ls2k::control::ReconcileAppliedDrivePwm(result.left, 0);
    ls2k::control::ReconcileAppliedDrivePwm(result.right, 200);

    Require(result.left.applied_pwm == 0 && result.left.negative_pid_output_blocked,
            "direction dead cycle must expose zero and blocked negative PID output");
    Require(result.right.applied_pwm == 200 &&
                !result.right.positive_pid_output_blocked &&
                !result.right.negative_pid_output_blocked,
            "normally applied peer channel must remain unconstrained");
}

}  // namespace

int main() {
    try {
        TestEveryOrdinaryDriveChangeUsesTheGlobalStepLimit();
        TestControlledStopRampsTowardZero();
        TestAntiReverseChangesDesiredToZeroBeforeSlew();
        TestAllowedSignChangeStillUsesOneGlobalStep();
        TestFloorPrecedesStepAndRemainsObservable();
        TestBrushlessCommandsAreNotSlewLimited();
        TestEmergencyStopBypassesSlew();
        TestInvalidStepLimitHoldsPreviousDriveCommand();
        TestHardwareDeadCycleBecomesAuthoritativeAppliedFact();
    } catch (const std::exception& error) {
        std::cerr << "actuator_command_shaper_test failed: " << error.what() << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "actuator_command_shaper_test passed\n";
    return EXIT_SUCCESS;
}
