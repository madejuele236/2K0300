#ifndef LS2K_RUNTIME_SHUTDOWN_HPP
#define LS2K_RUNTIME_SHUTDOWN_HPP

#include "port/platform_adapter.hpp"
#include "runtime/runtime_state.hpp"

namespace ls2k::runtime {

struct ShutdownResult {
    bool prior_terminal_failure = false;
    bool actuator_disable_ok = true;
    bool actuator_shutdown_ok = true;

    [[nodiscard]] bool ok() const noexcept {
        return !prior_terminal_failure && actuator_disable_ok && actuator_shutdown_ok;
    }
};

/// 执行运行时关闭流程并聚合既有终态、Disable 与 actuator Shutdown 结果。
/// @param platform     平台适配器集合
/// @param state        运行时状态
/// @param diagnostics  诊断输出接口
[[nodiscard]] ShutdownResult RunShutdown(port::PlatformBundle& platform,
                                         RuntimeState& state,
                                         port::DiagnosticSink& diagnostics);

}  // namespace ls2k::runtime

#endif  // LS2K_RUNTIME_SHUTDOWN_HPP
