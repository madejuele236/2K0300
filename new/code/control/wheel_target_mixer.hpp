#ifndef LS2K_LEGACY_WHEEL_TARGET_MIXER_HPP
#define LS2K_LEGACY_WHEEL_TARGET_MIXER_HPP

#include <algorithm>
#include <cmath>

namespace ls2k::control {

// ComputeGyroTurn owns this hard production bound; parameter validation and the
// wheel-target arithmetic contract share it so the accepted scale domain cannot
// drift from the actual applied-turn producer.
inline constexpr int kAppliedTurnOutputMagnitudeLimit = 9000;
// RUNNING_SPEED_TARGET and an enabled ML maneuver target feed the same mixer
// base input, so they share this configured physical-domain upper bound.
inline constexpr double kWheelSpeedTargetMax = 5000.0;

/// 车轮速度目标，包含左右轮的速度值
struct WheelSpeedTargets {
    double left = 0.0;   ///< 左轮目标速度
    double right = 0.0;  ///< 右轮目标速度
};

/// 差速轮速目标混合参数
struct WheelTargetMixerParameters {
    double accel_delta_scale = 1.0;  ///< 加速侧 turn delta 缩放系数
    double decel_delta_scale = 1.0;  ///< 减速侧 turn delta 缩放系数
};

/// 生产链中可到达的 |applied_turn_output| 上界。
/// Gyro turn 先限于 kAppliedTurnOutputMagnitudeLimit，随后 control loop
/// 再按 [0,1] turn_limit_scale 和非负 raw_turn_output_limit 限制。
inline double MaximumAppliedTurnOutputMagnitude(int raw_turn_output_limit) noexcept {
    return static_cast<double>(std::min(std::max(0, raw_turn_output_limit),
                                        kAppliedTurnOutputMagnitudeLimit));
}

/// 验证在生产最大 base target 和 applied turn 下 mixer 的乘加减保持有限。
/// 对于非负 base/delta，decel 减法的结果介于 [-DBL_MAX, DBL_MAX]；
/// 因此 decel 只需证明乘法有限，accel 还必须证明 base + delta 有限。
inline bool WheelTargetArithmeticIsFinite(double maximum_base_target,
                                          int raw_turn_output_limit,
                                          const WheelTargetMixerParameters& params) noexcept {
    if (!std::isfinite(maximum_base_target) || maximum_base_target < 0.0 ||
        raw_turn_output_limit < 0 ||
        !std::isfinite(params.accel_delta_scale) || params.accel_delta_scale < 0.0 ||
        !std::isfinite(params.decel_delta_scale) || params.decel_delta_scale < 0.0) {
        return false;
    }
    const double maximum_turn_delta =
        MaximumAppliedTurnOutputMagnitude(raw_turn_output_limit);
    const double maximum_accel_delta = maximum_turn_delta * params.accel_delta_scale;
    const double maximum_decel_delta = maximum_turn_delta * params.decel_delta_scale;
    return std::isfinite(maximum_accel_delta) &&
           std::isfinite(maximum_base_target + maximum_accel_delta) &&
           std::isfinite(maximum_decel_delta);
}

/// 车轮目标混合器，将速度目标和转向输出混合为左右轮独立速度目标
class WheelTargetMixer {
public:
    /// 计算左右轮速度目标
    /// 公式：加速侧 = 速度目标 + |转向输出| * accel_delta_scale,
    ///      减速侧 = 速度目标 - |转向输出| * decel_delta_scale
    /// @param effective_speed_target 有效速度目标（生产调用者保证有限，此处钳制为非负）
    /// @param applied_turn_output 施加的转向输出（生产上界由 MaximumAppliedTurnOutputMagnitude 定义）
    /// @param params 差速 turn delta 缩放参数（生产参数层必须通过 WheelTargetArithmeticIsFinite）
    /// @return 左右轮速度目标（均被钳制为非负）
    WheelSpeedTargets Compute(double effective_speed_target,
                              int applied_turn_output,
                              const WheelTargetMixerParameters& params = WheelTargetMixerParameters{}) const;
};

}  // namespace ls2k::control

#endif  // LS2K_LEGACY_WHEEL_TARGET_MIXER_HPP
