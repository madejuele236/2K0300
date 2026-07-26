#include "control/steering_yaw_controller.hpp"
#include "control/wheel_target_mixer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ls2k::control {
namespace {

bool ConvertFiniteFloat(double value, float& converted) noexcept {
    if (!std::isfinite(value) ||
        value < -static_cast<double>(std::numeric_limits<float>::max()) ||
        value > static_cast<double>(std::numeric_limits<float>::max())) {
        return false;
    }
    converted = static_cast<float>(value);
    return std::isfinite(converted);
}

}  // namespace

const char* ToString(SteeringYawStatus status) noexcept {
    switch (status) {
        case SteeringYawStatus::kNotComputed: return "not_computed";
        case SteeringYawStatus::kOk: return "ok";
        case SteeringYawStatus::kInvalidConfiguration: return "invalid_configuration";
        case SteeringYawStatus::kInvalidTurnInput: return "invalid_turn_input";
        case SteeringYawStatus::kInvalidGyroInput: return "invalid_gyro_input";
        case SteeringYawStatus::kInvalidControllerMemory: return "invalid_controller_memory";
        case SteeringYawStatus::kNonFiniteArithmetic: return "nonfinite_arithmetic";
    }
    return "unknown";
}

/// SteeringYawController::Configure 实现
/// 从运行时参数读取PID系数、速度目标和增益值
bool SteeringYawController::Configure(const port::RuntimeParameters& params) {
    float gyro_p = 0.0F;
    float gyro_i = 0.0F;
    float gyro_d = 0.0F;
    float running_speed_target = 0.0F;
    float lateral_gain = 0.0F;
    float heading_gain = 0.0F;
    float curvature_gain = 0.0F;
    configured_valid_ =
        YawRatePidArithmeticIsFinite(
            params.yaw_rate_pid_p, params.yaw_rate_pid_i, params.yaw_rate_pid_d) &&
        ConvertFiniteFloat(params.yaw_rate_pid_p, gyro_p) &&
        ConvertFiniteFloat(params.yaw_rate_pid_i, gyro_i) &&
        ConvertFiniteFloat(params.yaw_rate_pid_d, gyro_d) &&
        ConvertFiniteFloat(params.running_speed_target, running_speed_target) &&
        params.running_speed_target >= 0.0 && params.raw_turn_output_limit >= 0 &&
        ConvertFiniteFloat(params.bev_control_model.lateral_offset_to_wheel_delta_gain,
                           lateral_gain) &&
        ConvertFiniteFloat(params.bev_control_model.heading_error_to_wheel_delta_gain,
                           heading_gain) &&
        ConvertFiniteFloat(params.bev_control_model.curvature_to_wheel_delta_gain,
                           curvature_gain);
    if (!configured_valid_) {
        return false;
    }

    gyro_p_ = gyro_p;
    gyro_i_ = gyro_i;
    gyro_d_ = gyro_d;
    running_speed_target_ = std::max(1.0F, running_speed_target);
    raw_turn_output_limit_ = static_cast<float>(params.raw_turn_output_limit);
    lateral_offset_to_wheel_delta_gain_ = lateral_gain;
    heading_error_to_wheel_delta_gain_ = heading_gain;
    curvature_to_wheel_delta_gain_ = curvature_gain;
    return true;
}

/// SteeringYawController::Reset 实现
/// 重置控制器状态（当前为空操作）
void SteeringYawController::Reset() {}

/// SteeringYawController::ComputeTurnOutputTarget 实现
/// 根据参考跟踪几何和速度目标计算转向输出目标
TurnOutputTargetComputation SteeringYawController::ComputeTurnOutputTarget(
    const port::ReferenceTrackingGeometry& reference_tracking_geometry,
    double effective_speed_target,
    port::BEVControllerMemory& memory) {
    TurnOutputTargetComputation computation{};
    if (!configured_valid_) {
        computation.status = SteeringYawStatus::kInvalidConfiguration;
        return computation;
    }
    if (!reference_tracking_geometry.computed ||
        !std::isfinite(reference_tracking_geometry.lateral_offset_m) ||
        !std::isfinite(reference_tracking_geometry.heading_error_rad) ||
        !std::isfinite(reference_tracking_geometry.curvature_m_inv) ||
        !std::isfinite(effective_speed_target) || effective_speed_target < 0.0 ||
        effective_speed_target > static_cast<double>(std::numeric_limits<float>::max())) {
        computation.status = SteeringYawStatus::kInvalidTurnInput;
        return computation;
    }
    const float speed_scale =
        static_cast<float>(effective_speed_target) / std::max(running_speed_target_, 1.0F);
    const float lateral_term =
        lateral_offset_to_wheel_delta_gain_ * speed_scale * reference_tracking_geometry.lateral_offset_m;
    const float heading_term =
        heading_error_to_wheel_delta_gain_ * speed_scale * reference_tracking_geometry.heading_error_rad;
    const float curvature_term =
        curvature_to_wheel_delta_gain_ * speed_scale *
        reference_tracking_geometry.curvature_m_inv;
    const float turn_output_candidate =
        lateral_term + heading_term + curvature_term;
    if (!std::isfinite(speed_scale) || !std::isfinite(lateral_term) ||
        !std::isfinite(heading_term) || !std::isfinite(curvature_term) ||
        !std::isfinite(turn_output_candidate)) {
        computation.status = SteeringYawStatus::kNonFiniteArithmetic;
        return computation;
    }
    const float turn_output_target =
        std::clamp(turn_output_candidate, -raw_turn_output_limit_, raw_turn_output_limit_);
    if (!std::isfinite(turn_output_target)) {
        computation.status = SteeringYawStatus::kNonFiniteArithmetic;
        return computation;
    }
    memory.weighted_lateral_error_last = reference_tracking_geometry.lateral_offset_m;
    memory.last_gain_scale = speed_scale;
    memory.turn_output_target_last = turn_output_target;

    computation.valid = true;
    computation.status = SteeringYawStatus::kOk;
    computation.lateral_offset_gain = lateral_offset_to_wheel_delta_gain_;
    computation.heading_error_gain = heading_error_to_wheel_delta_gain_;
    computation.curvature_gain = curvature_to_wheel_delta_gain_;
    computation.speed_scale = speed_scale;
    computation.lateral_term = lateral_term;
    computation.heading_term = heading_term;
    computation.curvature_term = curvature_term;
    computation.turn_output_candidate = turn_output_candidate;
    computation.turn_output_target = turn_output_target;
    return computation;
}

/// SteeringYawController::ComputeGyroTurn 实现
/// 使用陀螺仪角速度反馈对转向输出进行PID补偿
/// 输出 = 转向目标 + P项 + I项 + D项
/// 积分项累加器被限制在[-1200, 1200]范围内
GyroTurnComputation SteeringYawController::ComputeGyroTurn(float turn_output_target,
                                                           float gyro_z,
                                                           port::BEVControllerMemory& memory) {
    GyroTurnComputation computation{};
    if (!configured_valid_) {
        computation.status = SteeringYawStatus::kInvalidConfiguration;
        return computation;
    }
    if (!std::isfinite(turn_output_target) ||
        std::abs(turn_output_target) > kMaximumTurnOutputTargetMagnitude) {
        computation.status = SteeringYawStatus::kInvalidTurnInput;
        return computation;
    }
    if (!std::isfinite(gyro_z) ||
        std::abs(gyro_z) > port::kMaximumProducedGyroMagnitudeRadPerSec) {
        computation.status = SteeringYawStatus::kInvalidGyroInput;
        return computation;
    }
    if (!std::isfinite(memory.gyro_i_accumulator) ||
        std::abs(memory.gyro_i_accumulator) > kYawIntegralAccumulatorMagnitudeLimit ||
        !std::isfinite(memory.gyro_error_last) ||
        std::abs(memory.gyro_error_last) > kMaximumYawErrorMagnitude) {
        computation.status = SteeringYawStatus::kInvalidControllerMemory;
        return computation;
    }
    const float measurement = gyro_z;
    const float error = -measurement;
    const float integral_accumulator =
        std::clamp(memory.gyro_i_accumulator + error,
                   -kYawIntegralAccumulatorMagnitudeLimit,
                   kYawIntegralAccumulatorMagnitudeLimit);
    const float p_term = gyro_p_ * error;
    const float d_term = gyro_d_ * (error - memory.gyro_error_last);
    const float i_term = gyro_i_ * integral_accumulator;
    float output = turn_output_target + p_term;
    output += i_term;
    output += d_term;
    if (!std::isfinite(integral_accumulator) || !std::isfinite(p_term) ||
        !std::isfinite(i_term) || !std::isfinite(d_term) || !std::isfinite(output)) {
        computation.status = SteeringYawStatus::kNonFiniteArithmetic;
        return computation;
    }
    memory.gyro_i_accumulator = integral_accumulator;
    memory.gyro_error_last = error;

    computation.valid = true;
    computation.status = SteeringYawStatus::kOk;
    computation.gyro_z = measurement;
    computation.gyro_error = error;
    computation.gyro_p_term = p_term;
    computation.gyro_d_term = d_term;
    constexpr float kTurnLimit = static_cast<float>(kAppliedTurnOutputMagnitudeLimit);
    computation.raw_turn_output = std::clamp(output, -kTurnLimit, kTurnLimit);
    return computation;
}

}  // namespace ls2k::control
