#include "port/platform_adapter.hpp"
#include "platform/true_ls2k0300/motor_device.hpp"
#include "platform/true_ls2k0300/vendor_paths.hpp"

#include <string>

namespace ls2k::platform {
namespace {

const char* MotorStatusText(true_ls2k0300::MotorStatus status) {
    using true_ls2k0300::MotorStatus;
    switch (status) {
        case MotorStatus::kOk: return "ok";
        case MotorStatus::kNotInitialized: return "not initialized";
        case MotorStatus::kOpenFailed: return "device open failed";
        case MotorStatus::kWriteFailed: return "device write failed";
        case MotorStatus::kCloseFailed: return "device close failed";
        case MotorStatus::kSafeStopFailed: return "safe PWM=0 rollback failed";
    }
    return "unknown motor status";
}

class ActuatorAdapter final : public port::IActuatorAdapter {
public:
    bool Initialize(const port::HardwareProfile& profile, port::DiagnosticSink& diagnostics) override {
        if (!port::IsEnabled(profile.actuator)) {
            diagnostics.Emit({port::DiagnosticLevel::kInfo,
                              "actuator.disabled",
                              "actuator subsystem disabled by hardware profile",
                              port::NowMs()});
            enabled_ = false;
            ready_ = false;
            return true;
        }

        enabled_ = true;
        adaptation_hook_ = profile.actuator.mode == port::SubsystemMode::kAdaptationHook;
        hook_name_ = profile.actuator.hook;

        if (adaptation_hook_) {
            ready_ = true;
            diagnostics.Emit({port::DiagnosticLevel::kWarning,
                              "actuator.init.hook",
                              "actuator direct path bypassed; adaptation hook selected: " + hook_name_,
                              port::NowMs()});
            return true;
        }

        if (hook_name_ != "differential-motor-plus-brushless-esc") {
            ready_ = false;
            diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                              "actuator.init.unsupported_hook",
                              "unsupported actuator direct-match hook: " + hook_name_,
                              port::NowMs()});
            return false;
        }

        const true_ls2k0300::MotorResult motor_init = motor_.Initialize();
        if (!motor_init.ok()) {
            ready_ = false;
            diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                              "actuator.init.motor",
                              std::string("motor backend unavailable: ") +
                                  MotorStatusText(motor_init.status),
                              port::NowMs()});
            return false;
        }

        ready_ = true;
        diagnostics.Emit({port::DiagnosticLevel::kInfo,
                          "actuator.init",
                          "actuator initialized: logical_left maps to pwm=" +
                              std::string(true_ls2k0300::kRightMotorPwmPath) +
                              ", gpio=" + std::string(true_ls2k0300::kRightMotorGpioPath) +
                              "; logical_right maps to pwm=" +
                              std::string(true_ls2k0300::kLeftMotorPwmPath) +
                              ", gpio=" + std::string(true_ls2k0300::kLeftMotorGpioPath) +
                              "; brushless_esc=" + std::string(true_ls2k0300::kBrushlessEsc1PwmPath) +
                              "," + std::string(true_ls2k0300::kBrushlessEsc2PwmPath),
                          port::NowMs()});
        return true;
    }

    port::ActuatorApplyResult Apply(const port::ActuatorCommand& command,
                                    port::DiagnosticSink& diagnostics) override {
        if (!enabled_ || !ready_) {
            port::EmitRateLimited(diagnostics,
                                  {port::DiagnosticLevel::kFailSafe,
                                   "actuator.apply.unavailable",
                                   "actuator apply requested while actuator adapter not ready",
                                   port::NowMs()},
                                  1000);
            return {};
        }

        if (adaptation_hook_) {
            port::EmitRateLimited(diagnostics,
                                  {port::DiagnosticLevel::kFailSafe,
                                   "actuator.hook.apply",
                                   "actuator adaptation hook selected with no concrete implementation: " +
                                       hook_name_ + "; suppressing actuator output",
                                   port::NowMs()},
                                  1000);
            return {};
        }

        if (command.emergency_stop) {
            return {DisableAllForApply(diagnostics, "actuator.emergency_stop.failed"), {}};
        }

        const true_ls2k0300::MotorResult result = motor_.Apply(
            {command.left_drive_pwm,
             command.right_drive_pwm,
             command.left_brushless_pwm,
             command.right_brushless_pwm});
        if (!result.ok()) {
            ready_ = false;
            port::EmitRateLimited(diagnostics,
                                  {port::DiagnosticLevel::kFailSafe,
                                   "actuator.apply.motor_failed",
                                   MotorStatusText(result.status),
                                   port::NowMs()},
                                  1000);
            return {};
        }

        return {
            true,
            {result.applied_command.left_drive_pwm,
             result.applied_command.right_drive_pwm,
             result.applied_command.left_esc_pwm,
             result.applied_command.right_esc_pwm,
             false},
        };
    }

    bool Disable(port::DiagnosticSink& diagnostics) override {
        if (!enabled_ || adaptation_hook_) {
            return true;
        }
        const true_ls2k0300::MotorResult result = motor_.Apply({});
        if (!result.ok()) {
            ready_ = false;
            port::EmitRateLimited(diagnostics,
                                  {port::DiagnosticLevel::kWarning,
                                   "actuator.disable.failed",
                                   MotorStatusText(result.status),
                                   port::NowMs()},
                                  1000);
            return false;
        }
        return true;
    }

    bool Shutdown(port::DiagnosticSink& diagnostics) override {
        bool stop_ok = true;
        if (enabled_ && !adaptation_hook_) {
            const true_ls2k0300::MotorResult result = motor_.Stop();
            stop_ok = result.ok();
            if (!stop_ok) {
                port::EmitRateLimited(diagnostics,
                                      {port::DiagnosticLevel::kFailSafe,
                                       "actuator.shutdown.failed",
                                       MotorStatusText(result.status),
                                       port::NowMs()},
                                      1000);
            }
        }
        ready_ = false;
        diagnostics.Emit({stop_ok ? port::DiagnosticLevel::kInfo
                                 : port::DiagnosticLevel::kFailSafe,
                          stop_ok ? "actuator.shutdown" : "actuator.shutdown.unconfirmed",
                          stop_ok ? "actuator adapter shutdown complete"
                                  : "actuator adapter resources released but final motor stop was not confirmed",
                          port::NowMs()});
        return stop_ok;
    }

    bool Ready() const override { return ready_; }

private:
    bool DisableAllForApply(port::DiagnosticSink& diagnostics, const char* diagnostic_code) {
        const true_ls2k0300::MotorResult result = motor_.Apply({});
        const bool ok = result.ok();
        if (!ok) {
            ready_ = false;
            port::EmitRateLimited(diagnostics,
                                  {port::DiagnosticLevel::kFailSafe,
                                   diagnostic_code,
                                   MotorStatusText(result.status),
                                   port::NowMs()},
                                  1000);
        }
        return ok;
    }

    true_ls2k0300::MotorDevice motor_{};
    bool enabled_ = false;
    bool ready_ = false;
    bool adaptation_hook_ = false;
    std::string hook_name_ = "direct-match";
};

}  // namespace

std::unique_ptr<port::IActuatorAdapter> MakeActuatorAdapter() {
    return std::make_unique<ActuatorAdapter>();
}

}  // namespace ls2k::platform
