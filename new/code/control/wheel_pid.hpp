#ifndef LS2K_LEGACY_WHEEL_PID_HPP
#define LS2K_LEGACY_WHEEL_PID_HPP

#include "port/runtime_parameter_types.hpp"

namespace ls2k::control {

struct DrivePwmShapingChannel;

enum class WheelPidAntiWindupReason {
    kNone = 0,
    kActuatorLimit,
    kNotApplied,
    kInvalidComputation,
};

struct WheelPidComputation final {
    bool valid = false;
    double error = 0.0;
    double filtered_measured_speed = 0.0;
    double previous_integral = 0.0;
    double candidate_integral = 0.0;
    double output_with_previous_integral = 0.0;
    double unconstrained_output = 0.0;
    int requested_pwm = 0;
    bool hard_saturated = false;
};

struct WheelPidCommitResult final {
    bool anti_windup_active = false;
    WheelPidAntiWindupReason reason = WheelPidAntiWindupReason::kNone;
    double integral = 0.0;
};

/// 旧版车轮PID控制器，用于控制电机转速跟踪目标速度
class WheelPidController {
public:
    /// 从参数结构配置PID系数
    void Configure(const port::WheelPidParameters& params);
    /// 重置控制器内部状态（误差、积分、滤波）
    void Reset();
    /// 计算 PID 候选输出；积分仅在 CommitAppliedOutput 后提交。
    /// @param target_speed 目标速度
    /// @param measured_speed 测量速度（先经过一阶低通滤波）
    /// @param pwm_limit PWM输出限幅
    /// 输入或中间结果非有限时返回 valid=false、requested_pwm=0，且不推进控制器状态。
    /// @return 候选积分、未限幅输出、PWM 请求及计算有效性
    WheelPidComputation Evaluate(double target_speed, double measured_speed, int pwm_limit);
    /// 根据真实执行器结果提交、冻结候选积分，或在步进限幅 anti-windup 时清零积分。
    WheelPidCommitResult CommitAppliedOutput(const WheelPidComputation& computation,
                                             const DrivePwmShapingChannel& shaping,
                                             bool actuator_applied);
    [[nodiscard]] double integral() const noexcept { return integral_; }

private:
    double p_ = 84.0;                     ///< 比例系数
    double i_ = 2.4;                      ///< 积分系数
    double d_ = 0.75;                     ///< 微分系数
    double integral_limit_ = 5000.0;      ///< 积分限幅
    double measurement_filter_alpha_ = 0.4;  ///< 测量值低通滤波系数（0=完全信任历史，1=完全信任新值）
    double last_error_ = 0.0;             ///< 上一帧误差（用于微分项）
    double integral_ = 0.0;               ///< 积分累加器
    double filtered_measured_speed_ = 0.0;  ///< 滤波后的测量速度
    bool filtered_measured_ready_ = false;  ///< 滤波是否已初始化
};

[[nodiscard]] const char* ToString(WheelPidAntiWindupReason reason) noexcept;

}  // namespace ls2k::control

#endif  // LS2K_LEGACY_WHEEL_PID_HPP
