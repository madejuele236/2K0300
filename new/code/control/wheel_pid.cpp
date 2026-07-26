#include "control/wheel_pid.hpp"

#include "control/actuator_command_builder.hpp"

#include <algorithm>
#include <cmath>

namespace ls2k::control {
namespace {

bool FinitePidOutput(double p,
                     double i,
                     double d,
                     double error,
                     double integral,
                     double derivative,
                     double& output) {
    const double proportional = p * error;
    const double integral_term = i * integral;
    const double derivative_term = d * derivative;
    if (!std::isfinite(proportional) || !std::isfinite(integral_term) ||
        !std::isfinite(derivative_term)) {
        return false;
    }
    const double proportional_and_integral = proportional + integral_term;
    if (!std::isfinite(proportional_and_integral)) {
        return false;
    }
    output = proportional_and_integral + derivative_term;
    return std::isfinite(output);
}

}  // namespace

void WheelPidController::Configure(const port::WheelPidParameters& params) {
    p_ = params.p;
    i_ = params.i;
    d_ = params.d;
    integral_limit_ = params.integral_limit;
    measurement_filter_alpha_ = params.measurement_filter_alpha;
}

void WheelPidController::Reset() {
    last_error_ = 0.0;
    integral_ = 0.0;
    filtered_measured_speed_ = 0.0;
    filtered_measured_ready_ = false;
}

// PID 输出基于固定控制周期计算：P 使用当前误差，I 累积误差，D 使用相邻周期误差差分。
// 当前接口没有 dt 参数，因此 D 项不是按秒归一化的微分项，也不存在 dt 除零路径。
// 如果未来控制周期变为可变周期，应同步调整接口和 D 项语义，而不是只改这里的注释。
WheelPidComputation WheelPidController::Evaluate(double target_speed,
                                                 double measured_speed,
                                                 int pwm_limit) {
    WheelPidComputation computation{};
    if (!std::isfinite(target_speed) || !std::isfinite(measured_speed) ||
        !std::isfinite(p_) || !std::isfinite(i_) || !std::isfinite(d_) ||
        !std::isfinite(integral_limit_) || integral_limit_ < 0.0 ||
        !std::isfinite(measurement_filter_alpha_) ||
        measurement_filter_alpha_ < 0.0 || measurement_filter_alpha_ > 1.0 ||
        pwm_limit <= 0) {
        return computation;
    }

    double filtered_measured_speed = measured_speed;
    if (filtered_measured_ready_) {
        if (measurement_filter_alpha_ == 0.0) {
            filtered_measured_speed = filtered_measured_speed_;
        } else if (measurement_filter_alpha_ != 1.0) {
            filtered_measured_speed =
                measured_speed * measurement_filter_alpha_ +
                filtered_measured_speed_ * (1.0 - measurement_filter_alpha_);
        }
    }
    if (!std::isfinite(filtered_measured_speed)) {
        return computation;
    }

    const double error = target_speed - filtered_measured_speed;
    if (!std::isfinite(error)) {
        return computation;
    }
    double candidate_integral = integral_ + error;
    if (error > 0.0 && integral_ > integral_limit_ - error) {
        candidate_integral = integral_limit_;
    } else if (error < 0.0 && integral_ < -integral_limit_ - error) {
        candidate_integral = -integral_limit_;
    }
    if (!std::isfinite(candidate_integral)) {
        return computation;
    }
    const double derivative = error - last_error_;
    if (!std::isfinite(derivative)) {
        return computation;
    }
    double output_with_previous_integral = 0.0;
    double unconstrained_output = 0.0;
    if (!FinitePidOutput(p_, i_, d_, error, integral_, derivative,
                         output_with_previous_integral) ||
        !FinitePidOutput(p_, i_, d_, error, candidate_integral, derivative,
                         unconstrained_output)) {
        return computation;
    }

    computation.valid = true;
    computation.filtered_measured_speed = filtered_measured_speed;
    computation.error = error;
    computation.previous_integral = integral_;
    computation.candidate_integral = candidate_integral;
    computation.output_with_previous_integral = output_with_previous_integral;
    computation.unconstrained_output = unconstrained_output;
    filtered_measured_speed_ = filtered_measured_speed;
    filtered_measured_ready_ = true;
    last_error_ = error;
    const double clamped = std::clamp(computation.unconstrained_output,
                                      -static_cast<double>(pwm_limit),
                                      static_cast<double>(pwm_limit));
    computation.requested_pwm = static_cast<int>(std::round(clamped));
    computation.hard_saturated = clamped != computation.unconstrained_output;
    return computation;
}

WheelPidCommitResult WheelPidController::CommitAppliedOutput(
    const WheelPidComputation& computation,
    const DrivePwmShapingChannel& shaping,
    bool actuator_applied) {
    WheelPidCommitResult result{};
    if (!computation.valid) {
        result.anti_windup_active = true;
        result.reason = WheelPidAntiWindupReason::kInvalidComputation;
        result.integral = integral_;
        return result;
    }
    if (!actuator_applied) {
        result.anti_windup_active = true;
        result.reason = WheelPidAntiWindupReason::kNotApplied;
        result.integral = integral_;
        return result;
    }

    const double integral_push =
        computation.unconstrained_output - computation.output_with_previous_integral;
    const bool pushes_further_into_hard_limit =
        computation.hard_saturated &&
        ((computation.unconstrained_output > computation.requested_pwm && integral_push > 0.0) ||
         (computation.unconstrained_output < computation.requested_pwm && integral_push < 0.0));
    const bool pushes_into_downstream_constraint =
        (integral_push > 0.0 && shaping.positive_pid_output_blocked) ||
        (integral_push < 0.0 && shaping.negative_pid_output_blocked);
    // PID owns hard-limit direction. The shaper owns step/reverse direction and
    // explicitly excludes floor adjustment. Integer rounding is not a constraint.
    if (pushes_further_into_hard_limit || pushes_into_downstream_constraint) {
        result.anti_windup_active = true;
        result.reason = WheelPidAntiWindupReason::kActuatorLimit;
        // Step-limit anti-windup restarts I; other actuator constraints retain it.
        if (shaping.step_limited && pushes_into_downstream_constraint) {
            integral_ = 0.0;
        }
    } else {
        integral_ = computation.candidate_integral;
    }
    result.integral = integral_;
    return result;
}

const char* ToString(WheelPidAntiWindupReason reason) noexcept {
    switch (reason) {
        case WheelPidAntiWindupReason::kNone:
            return "none";
        case WheelPidAntiWindupReason::kActuatorLimit:
            return "actuator_limit";
        case WheelPidAntiWindupReason::kNotApplied:
            return "not_applied";
        case WheelPidAntiWindupReason::kInvalidComputation:
            return "invalid_computation";
    }
    return "unknown";
}

}  // namespace ls2k::control
