#include "control/actuator_command_builder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ls2k::control {
namespace {

int ClampDrivePwm(int pwm, int pwm_limit) noexcept {
    const int limit = std::max(0, pwm_limit);
    return std::clamp(pwm, -limit, limit);
}

int ApplyPwmFloor(int pwm, int pwm_limit, int pwm_floor) noexcept {
    if (pwm == 0) {
        return 0;
    }
    const int floor = std::clamp(pwm_floor, 0, std::max(0, pwm_limit));
    if (floor == 0) {
        return pwm;
    }
    return pwm > 0 ? std::max(pwm, floor) : std::min(pwm, -floor);
}

int ApplyStepLimit(int previous, int desired, int step_limit) noexcept {
    if (step_limit <= 0) {
        return previous;
    }
    const std::int64_t delta = static_cast<std::int64_t>(desired) - previous;
    const std::int64_t step = step_limit;
    const std::int64_t limited_delta = std::clamp(delta, -step, step);
    return static_cast<int>(static_cast<std::int64_t>(previous) + limited_delta);
}

DrivePwmShapingChannel ShapeDriveChannel(int previous,
                                         int requested,
                                         int pwm_limit,
                                         int pwm_floor,
                                         bool prohibit_reverse,
                                         int step_limit) noexcept {
    DrivePwmShapingChannel channel{};
    channel.requested_pwm = ClampDrivePwm(requested, pwm_limit);
    int desired = channel.requested_pwm;
    if (prohibit_reverse && desired < 0) {
        desired = 0;
        channel.reverse_suppressed = true;
        channel.negative_pid_output_blocked = true;
    }
    const int floored = ApplyPwmFloor(desired, pwm_limit, pwm_floor);
    channel.floor_adjusted = floored != desired;
    channel.desired_pwm = floored;
    channel.applied_pwm = ApplyStepLimit(previous, channel.desired_pwm, step_limit);
    channel.step_limited = channel.applied_pwm != channel.desired_pwm;
    // Slew limiting is an anti-windup constraint only when it leaves the applied
    // command short of the PID request itself. A floor-adjusted desired value can
    // be beyond both values and must not turn floor traversal into false windup.
    if (channel.step_limited && channel.desired_pwm > channel.applied_pwm &&
        channel.requested_pwm > channel.applied_pwm) {
        channel.positive_pid_output_blocked = true;
    }
    if (channel.step_limited && channel.desired_pwm < channel.applied_pwm &&
        channel.requested_pwm < channel.applied_pwm) {
        channel.negative_pid_output_blocked = true;
    }
    return channel;
}

}  // namespace

port::ActuatorCommand ActuatorCommandBuilder::Compose(int left_drive_pwm,
                                                      int right_drive_pwm,
                                                      int left_brushless_pwm,
                                                      int right_brushless_pwm,
                                                      bool emergency_stop,
                                                      int drive_pwm_limit,
                                                      int brushless_pwm_limit) const {
    if (emergency_stop) {
        return {};
    }

    port::ActuatorCommand command{};
    command.left_drive_pwm = std::clamp(left_drive_pwm, -drive_pwm_limit, drive_pwm_limit);
    command.right_drive_pwm = std::clamp(right_drive_pwm, -drive_pwm_limit, drive_pwm_limit);
    command.left_brushless_pwm = std::clamp(left_brushless_pwm, 0, std::max(0, brushless_pwm_limit));
    command.right_brushless_pwm = std::clamp(right_brushless_pwm, 0, std::max(0, brushless_pwm_limit));
    command.emergency_stop = false;
    return command;
}

ActuatorCommandShapingResult ShapeActuatorCommand(
    const port::ActuatorCommand& previous_applied,
    port::ActuatorCommand requested,
    int drive_pwm_limit,
    int drive_pwm_floor,
    bool prohibit_reverse_pwm,
    int drive_pwm_step_limit) noexcept {
    ActuatorCommandShapingResult result{};
    if (requested.emergency_stop) {
        result.command = requested;
        return result;
    }

    result.left = ShapeDriveChannel(previous_applied.left_drive_pwm,
                                    requested.left_drive_pwm,
                                    drive_pwm_limit,
                                    drive_pwm_floor,
                                    prohibit_reverse_pwm,
                                    drive_pwm_step_limit);
    result.right = ShapeDriveChannel(previous_applied.right_drive_pwm,
                                     requested.right_drive_pwm,
                                     drive_pwm_limit,
                                     drive_pwm_floor,
                                     prohibit_reverse_pwm,
                                     drive_pwm_step_limit);
    requested.left_drive_pwm = result.left.applied_pwm;
    requested.right_drive_pwm = result.right.applied_pwm;
    result.command = requested;
    return result;
}

void ReconcileAppliedDrivePwm(DrivePwmShapingChannel& channel,
                              int applied_pwm) noexcept {
    channel.applied_pwm = applied_pwm;
    if (channel.requested_pwm > applied_pwm) {
        channel.positive_pid_output_blocked = true;
    } else if (channel.requested_pwm < applied_pwm) {
        channel.negative_pid_output_blocked = true;
    }
}

}  // namespace ls2k::control
