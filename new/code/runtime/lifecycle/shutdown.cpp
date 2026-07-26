#include "runtime/lifecycle/shutdown.hpp"


namespace ls2k::runtime {

/// 执行运行时关闭：设置停止/退出标志 → 停止定时器 → 禁用执行器 →
/// 在安全结果确认时清空运行时状态，否则保留终态证据 → 关闭各硬件适配器 → 发布结果诊断
/// @param platform     平台适配器集合（含 timer/actuator/camera/imu/encoder）
/// @param state        运行时状态（将被清理）
/// @param diagnostics  诊断输出接口
ShutdownResult RunShutdown(port::PlatformBundle& platform,
                           RuntimeState& state,
                           port::DiagnosticSink& diagnostics) {
    state.stop_requested.store(true);
    state.exit_requested.store(true);
    if (platform.timer) {
        platform.timer->Stop(diagnostics);
    }
    state.timer_started = false;

    ShutdownResult result{};
    result.prior_terminal_failure = HasTerminalActuatorFailure(state);
    if (platform.actuator) {
        result.actuator_disable_ok = platform.actuator->Disable(diagnostics);
        if (!result.actuator_disable_ok) {
            diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                              "shutdown.actuator.disable_failed",
                              "shutdown could not confirm safe actuator disable before resource release",
                              port::NowMs()});
        }
    }
    const bool preserve_terminal_failure = result.prior_terminal_failure;
    state.perception = {};
    if (!preserve_terminal_failure && result.actuator_disable_ok) {
        state.actuators_armed = false;
        state.last_command = {};
        state.control_observation = {};
        state.control_debug_snapshot = {};
        state.command_history.Clear();
    }
    state.perception_memory_reset_generation.fetch_add(1);

    if (platform.camera) {
        platform.camera->Shutdown(diagnostics);
    }
    if (platform.imu) {
        platform.imu->Shutdown(diagnostics);
    }
    if (platform.encoder) {
        platform.encoder->Shutdown(diagnostics);
    }
    if (platform.actuator) {
        result.actuator_shutdown_ok = platform.actuator->Shutdown(diagnostics);
    }

    if (!result.ok()) {
        PreserveTerminalActuatorFailure(state, control::MotionPhase::kDisarmed, port::NowMs());
    }

    diagnostics.Emit({result.ok() ? port::DiagnosticLevel::kInfo
                                  : port::DiagnosticLevel::kFailSafe,
                      result.ok() ? "shutdown.complete" : "shutdown.complete.unconfirmed",
                      result.ok() ? "actuators disabled and resources released"
                                  : "resources released but final actuator safety was not confirmed",
                      port::NowMs()});
    return result;
}

}  // namespace ls2k::runtime
