#include "port/platform_adapter.hpp"
#include "platform/true_ls2k0300/adc_device.hpp"
#include "platform/true_ls2k0300/vendor_paths.hpp"

#include <cctype>
#include <cstdlib>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

namespace ls2k::platform {
namespace {

constexpr int kDefaultLowVoltageRawThreshold = 400;

const char* AdcErrorDetail(true_ls2k0300::AdcReadError error) noexcept {
    switch (error) {
        case true_ls2k0300::AdcReadError::kNone:
            return "none";
        case true_ls2k0300::AdcReadError::kOpenFailed:
            return "path unavailable";
        case true_ls2k0300::AdcReadError::kReadFailed:
            return "read failed";
        case true_ls2k0300::AdcReadError::kEmpty:
            return "empty input";
        case true_ls2k0300::AdcReadError::kParseFailed:
            return "invalid integer";
        case true_ls2k0300::AdcReadError::kOutOfRange:
            return "integer out of range";
    }
    return "unknown error";
}

/**
 * 尝试将 C 字符串解析为正整数。
 * 要求字符串全部被解析且结果 > 0。
 * @param text 待解析的 C 字符串
 * @param out 输出参数，解析得到的正整数
 * @return true 表示解析成功
 */
bool TryParsePositiveInt(const char* text, int& out) {
    if (text == nullptr || text[0] == '\0') {
        return false;
    }
    try {
        std::size_t parsed = 0;
        const int value = std::stoi(text, &parsed);
        if (text[parsed] != '\0' || value <= 0) {
            return false;
        }
        out = value;
        return true;
    } catch (...) {
        return false;
    }
}

/**
 * 读取环境变量并解析为布尔值。
 * 支持的值包括：1/true/yes/on（返回 true），0/false/no/off（返回 false）。
 * 环境变量不存在或无法识别时返回 nullopt。
 * @param key 环境变量名称
 * @return 解析后的布尔值（std::optional），失败时为 nullopt
 */
std::optional<bool> ReadBoolEnv(const char* key) {
    const char* raw = std::getenv(key);
    if (raw == nullptr) {
        return std::nullopt;
    }
    std::string token(raw);
    for (char& c : token) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    if (token == "1" || token == "true" || token == "yes" || token == "on") {
        return true;
    }
    if (token == "0" || token == "false" || token == "no" || token == "off") {
        return false;
    }
    return std::nullopt;
}

/**
 * 电源监控适配器 —— 实现 port::IPowerMonitorAdapter 接口。
 * 负责读取电池 ADC 值、检测低电压状态，
 * 支持环境变量覆盖阈值和强制低电压状态注入（用于测试）。
 */
class PowerMonitorAdapter final : public port::IPowerMonitorAdapter {
public:
    /**
     * 配置低电压原始阈值。
     * 如果传入的阈值 <= 0，则使用内置默认值（kDefaultLowVoltageRawThreshold = 400）。
     * @param raw_threshold 原始 ADC 阈值
     * @param diagnostics 诊断输出接收器
     */
    void ConfigureLowVoltageThreshold(int raw_threshold, port::DiagnosticSink& diagnostics) override {
        if (raw_threshold > 0) {
            configured_raw_threshold_ = raw_threshold;
            return;
        }
        configured_raw_threshold_ = kDefaultLowVoltageRawThreshold;
        diagnostics.Emit({port::DiagnosticLevel::kWarning,
                          "power.low_voltage.threshold_config_invalid",
                          "invalid low-voltage raw threshold; using built-in fail-safe default",
                          port::NowMs()});
    }

    /**
     * 初始化电源监控适配器。
     * 通过 true_ls2k0300 桥接读取电池 ADC 路径（支持环境变量 LS2K_LOW_VOLTAGE_RAW_PATH 覆盖）。
     * @param diagnostics 诊断输出接收器
     * @return true 表示初始化完成（即使后端不可用也返回 true，就绪状态由 Ready() 指示）
     */
    bool Initialize(port::DiagnosticSink& diagnostics) override {
        initialized_ = true;
        const char* override_path = std::getenv("LS2K_LOW_VOLTAGE_RAW_PATH");
        const char* adc_path =
            (override_path != nullptr && override_path[0] != '\0') ? override_path : true_ls2k0300::kBatteryAdcPath;
        adc_.emplace(adc_path);
        const true_ls2k0300::AdcSampleResult probe = adc_->ReadRaw();
        ready_ = probe.ok();
        if (ready_) {
            diagnostics.Emit({port::DiagnosticLevel::kInfo,
                              "power.init",
                              "power monitor initialized with true_ls2k0300 adc device path=" + adc_->path(),
                              port::NowMs()});
            return true;
        }
        diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                          "power.init",
                          "battery ADC " + std::string(AdcErrorDetail(probe.error)) + ": " + adc_->path(),
                          port::NowMs()});
        return true;
    }

    /**
     * 采样低电压状态。
     * 首先检查环境变量 LS2K_FORCE_LOW_VOLTAGE（强制注入），
     * 然后通过环境变量 LS2K_LOW_VOLTAGE_RAW_THRESHOLD 覆盖阈值，
     * 最后通过底层桥接读取实际 ADC 值并与阈值比较判断是否低电压。
     * @param diagnostics 诊断输出接收器
     * @return 低电压采样结果，包含原始值、是否紧急等信息
     */
    port::LowVoltageSample SampleLowVoltage(port::DiagnosticSink& diagnostics) override {
        port::LowVoltageSample sample{};
        sample.capture_time_ms = port::NowMs();
        sample.threshold = configured_raw_threshold_;
        const char* threshold_env = std::getenv("LS2K_LOW_VOLTAGE_RAW_THRESHOLD");
        if (threshold_env != nullptr && threshold_env[0] != '\0') {
            int override_threshold = 0;
            if (TryParsePositiveInt(threshold_env, override_threshold)) {
                sample.threshold = override_threshold;
            } else {
                port::EmitRateLimited(diagnostics,
                                      {port::DiagnosticLevel::kWarning,
                                       "power.low_voltage.threshold_env_invalid",
                                       std::string("ignoring invalid LS2K_LOW_VOLTAGE_RAW_THRESHOLD value=") +
                                           threshold_env,
                                       sample.capture_time_ms},
                                      1000);
            }
        }

        const char* forced_raw = std::getenv("LS2K_FORCE_LOW_VOLTAGE");
        if (const std::optional<bool> forced = ReadBoolEnv("LS2K_FORCE_LOW_VOLTAGE"); forced.has_value()) {
            sample.valid = true;
            sample.emergency = *forced;
            sample.source = "forced-env";
            port::EmitRateLimited(diagnostics,
                                  {*forced ? port::DiagnosticLevel::kFailSafe : port::DiagnosticLevel::kInfo,
                                   "power.low_voltage.injected",
                                   std::string("forced low-voltage emergency=") +
                                       (*forced ? "true" : "false"),
                                   sample.capture_time_ms},
                                  1000);
            return sample;
        } else if (forced_raw != nullptr && forced_raw[0] != '\0') {
            port::EmitRateLimited(diagnostics,
                                  {port::DiagnosticLevel::kWarning,
                                   "power.low_voltage.invalid_env",
                                   std::string("ignoring invalid LS2K_FORCE_LOW_VOLTAGE value=") + forced_raw,
                                   sample.capture_time_ms},
                                  1000);
        }

        const true_ls2k0300::AdcSampleResult adc_sample =
            adc_ ? adc_->ReadRaw()
                 : true_ls2k0300::AdcSampleResult{-1, true_ls2k0300::AdcReadError::kOpenFailed, 0};

        if (adc_sample.ok()) {
            sample.raw_value = adc_sample.raw_value;
            sample.valid = true;
            sample.emergency = sample.raw_value <= sample.threshold;
            sample.source = adc_->path();
            std::ostringstream message;
            message << "low-voltage raw check path=" << sample.source << " raw=" << sample.raw_value
                    << " threshold=" << sample.threshold;
            port::EmitRateLimited(diagnostics,
                                  {sample.emergency ? port::DiagnosticLevel::kFailSafe
                                                    : port::DiagnosticLevel::kInfo,
                                   "startup.low_voltage.raw",
                                   message.str(),
                                   sample.capture_time_ms},
                                  1000);
            return sample;
        }

        sample.valid = false;
        sample.emergency = true;
        sample.source = adc_ ? adc_->path() : "unavailable";
        port::EmitRateLimited(diagnostics,
                              {port::DiagnosticLevel::kFailSafe,
                               "startup.low_voltage.unavailable",
                               "battery ADC " + std::string(AdcErrorDetail(adc_sample.error)) +
                                   "; forcing fail-safe emergency veto path=" + sample.source,
                               sample.capture_time_ms},
                              1000);
        return sample;
    }

    /**
     * 查询电源监控适配器是否已就绪。
     * @return true 表示已初始化且后端 ADC 读取可用
     */
    bool Ready() const override { return initialized_ && ready_; }

private:
    /** 是否已完成初始化调用 */
    bool initialized_ = false;
    /** 底层 ADC 读取是否可用 */
    bool ready_ = false;
    /** 初始化时绑定的 ADC 设备路径 */
    std::optional<true_ls2k0300::AdcDevice> adc_{};
    /** 已配置的低电压 ADC 阈值（可通过 ConfigureLowVoltageThreshold 设置） */
    int configured_raw_threshold_ = kDefaultLowVoltageRawThreshold;
};

}  // namespace

/**
 * 创建电源监控适配器实例（工厂函数）。
 * @return 指向 IPowerMonitorAdapter 接口的唯一指针
 */
std::unique_ptr<port::IPowerMonitorAdapter> MakePowerMonitorAdapter() {
    return std::make_unique<PowerMonitorAdapter>();
}

}  // namespace ls2k::platform
