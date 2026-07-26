#ifndef LS2K_LEGACY_ACTUATOR_COMMAND_BUILDER_HPP
#define LS2K_LEGACY_ACTUATOR_COMMAND_BUILDER_HPP

#include "port/actuator_command_types.hpp"

namespace ls2k::control {

/// 执行器命令构造器，负责将各输出通道 PWM 与急停信号组合成统一执行器指令
class ActuatorCommandBuilder {
public:
    /// 组合左右驱动PWM、左右无刷电调PWM与急停信号，生成统一执行器指令。
    port::ActuatorCommand Compose(int left_drive_pwm,
                                  int right_drive_pwm,
                                  int left_brushless_pwm,
                                  int right_brushless_pwm,
                                  bool emergency_stop,
                                  int drive_pwm_limit,
                                  int brushless_pwm_limit) const;
};

/// 单个有刷驱动通道在统一命令整形链中的可观测结果。
struct DrivePwmShapingChannel final {
    int requested_pwm = 0;
    int desired_pwm = 0;
    int applied_pwm = 0;
    bool reverse_suppressed = false;
    bool floor_adjusted = false;
    bool step_limited = false;
    /// 下游整形是否真实阻止 PID 请求继续向正/负方向变化。
    /// floor 只改变执行器的最低非零幅值，不设置这两个 anti-windup 事实。
    bool positive_pid_output_blocked = false;
    bool negative_pid_output_blocked = false;
};

/// 统一执行器命令整形结果。步进限制只作用于左右有刷驱动 PWM。
struct ActuatorCommandShapingResult final {
    port::ActuatorCommand command{};
    DrivePwmShapingChannel left{};
    DrivePwmShapingChannel right{};
};

[[nodiscard]] ActuatorCommandShapingResult ShapeActuatorCommand(
    const port::ActuatorCommand& previous_applied,
    port::ActuatorCommand requested,
    int drive_pwm_limit,
    int drive_pwm_floor,
    bool prohibit_reverse_pwm,
    int drive_pwm_step_limit) noexcept;

/// 将设备层实际应用的 PWM 回写为控制与 anti-windup 的权威事实。
void ReconcileAppliedDrivePwm(DrivePwmShapingChannel& channel,
                              int applied_pwm) noexcept;

}  // namespace ls2k::control

#endif  // LS2K_LEGACY_ACTUATOR_COMMAND_BUILDER_HPP
