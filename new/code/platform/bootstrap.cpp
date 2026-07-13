#include "platform/bootstrap.hpp"
#include "platform/true_ls2k0300/timer_device.hpp"

#include <atomic>
#include <functional>
#include <string>

namespace ls2k::platform {

namespace {

/// @brief 定时器适配器类
///
/// 实现 port::ITimerAdapter 接口，基于 true_ls2k0300::TimerDevice
/// 提供周期性定时器功能。支持硬件适配钩子（kAdaptationHook）和
/// 直接匹配（direct-match）两种工作模式。
class TimerAdapter final : public port::ITimerAdapter {
public:
    /// @brief 启动周期性定时器
    /// @param profile 子系统描述（决定模式：direct-match 或 adaptation-hook）
    /// @param period_ms 定时周期（毫秒）
    /// @param callback 每次定时触发的回调函数
    /// @param on_failure 定时器异常退出时的回调函数
    /// @param diagnostics 诊断输出接口
    /// @return 启动成功返回 true
    bool Start(const port::SubsystemProfile& profile,
               uint32_t period_ms,
               std::function<void()> callback,
               std::function<void()> on_failure,
               port::DiagnosticSink& diagnostics) override {
        Stop(diagnostics);

        if (!port::IsEnabled(profile)) {
            diagnostics.Emit({port::DiagnosticLevel::kWarning,
                              "timer.disabled",
                              "timer subsystem disabled by hardware profile",
                              port::NowMs()});
            return false;
        }

        callback_ = std::move(callback);
        on_failure_ = std::move(on_failure);
        diagnostics_sink_ = &diagnostics;

        if (profile.mode == port::SubsystemMode::kAdaptationHook) {
            const bool started = device_.Start(period_ms,
                                               &TimerAdapter::HandleTick,
                                               this,
                                               &TimerAdapter::HandleFailure,
                                               this);
            running_.store(started);
            diagnostics.Emit({started ? port::DiagnosticLevel::kWarning : port::DiagnosticLevel::kFailSafe,
                              started ? "timer.start.hook" : "timer.start.hook.failed",
                              started ? "timer routed through adaptation hook: " + profile.hook
                                      : "timer adaptation hook failed to start: " + profile.hook,
                              port::NowMs()});
            return started;
        }

        const bool started = device_.Start(period_ms,
                                           &TimerAdapter::HandleTick,
                                           this,
                                           &TimerAdapter::HandleFailure,
                                           this);
        running_.store(started);
        diagnostics.Emit({started ? port::DiagnosticLevel::kInfo : port::DiagnosticLevel::kFailSafe,
                          started ? "timer.start.pit" : "timer.start.pit.failed",
                          started ? "timer started with true_ls2k0300 timerfd bridge"
                                  : "timer direct-match timerfd bridge failed to start",
                          port::NowMs()});
        return started;
    }

    /// @brief 停止定时器
    /// @param diagnostics 诊断输出接口
    void Stop(port::DiagnosticSink& diagnostics) override {
        const bool was_running = running_.exchange(false);
        device_.Stop();
        if (was_running) {
            diagnostics.Emit({port::DiagnosticLevel::kInfo,
                              "timer.stop",
                              "timer bridge stopped",
                              port::NowMs()});
        }
    }

    /// @brief 检查定时器是否正在运行
    /// @return true 表示定时器正在运行
    bool Running() const override { return running_.load() && device_.Running(); }

private:
    static void HandleTick(void* context, true_ls2k0300::TimerTick) noexcept {
        static_cast<TimerAdapter*>(context)->callback_();
    }

    static void HandleFailure(void* context,
                              true_ls2k0300::TimerFailure failure) noexcept {
        auto& adapter = *static_cast<TimerAdapter*>(context);
        const bool was_running = adapter.running_.exchange(false);
        if (!was_running) {
            return;
        }

        const char* reason = "poll failed or timer fd reported an error";
        if (failure == true_ls2k0300::TimerFailure::kTimerRead) {
            reason = "timerfd read failed or returned partial data";
        }

        adapter.diagnostics_sink_->Emit(
            {port::DiagnosticLevel::kFailSafe,
             "timer.runtime.failure",
             std::string("timer backend exited unexpectedly: ") + reason +
                 "; escalating to control fail-safe handling",
             port::NowMs()});
        if (adapter.on_failure_) {
            adapter.on_failure_();
        }
    }

    /// 原子运行状态标志
    std::atomic<bool> running_{false};
    std::function<void()> callback_{};
    std::function<void()> on_failure_{};
    port::DiagnosticSink* diagnostics_sink_{nullptr};
    /// 底层定时器设备
    true_ls2k0300::TimerDevice device_{};
};

}  // namespace

/// @brief 创建平台适配器组合包
///
/// 根据硬件描述文件和诊断输出构造所有平台适配器实例（相机、IMU、
/// 编码器、执行器、电源、参数存储、定时器），打包为 PlatformBundle 返回。
/// @param profile 硬件描述文件（当前未使用，由各适配器独立解析）
/// @param diagnostics 诊断输出接口
/// @return 平台适配器组合包
port::PlatformBundle CreatePlatformBundle(const port::HardwareProfile&, port::DiagnosticSink&) {
    port::PlatformBundle bundle{};
    bundle.camera = MakeCameraAdapter();
    bundle.imu = MakeImuAdapter();
    bundle.encoder = MakeEncoderAdapter();
    bundle.actuator = MakeActuatorAdapter();
    bundle.power = MakePowerMonitorAdapter();
    bundle.params = MakeParamStore();
    bundle.timer = std::make_unique<TimerAdapter>();
    return bundle;
}

}  // namespace ls2k::platform
