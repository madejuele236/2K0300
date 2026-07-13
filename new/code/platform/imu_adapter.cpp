#include "port/platform_adapter.hpp"
#include "platform/true_ls2k0300/imu_device.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <exception>
#include <string>
#include <thread>

#include "port/numeric_parse.hpp"

namespace ls2k::platform {
namespace {

/// 重力加速度常数（m/s^2）
constexpr float kGravityMps2 = 9.80665F;
/// 加速度计每 LSB 对应的物理值（m/s^2）
constexpr float kAccelMetersPerSecPerCount = 0.0001220F * kGravityMps2;
/// 陀螺仪每 LSB 对应的物理值（rad/s）
constexpr float kGyroRadPerSecPerCount = 0.0010641F;
/// 加速度低通滤波器新数据权重
constexpr float kAccelFilterNewWeight = 0.9F;
/// 加速度低通滤波器旧数据权重
constexpr float kAccelFilterOldWeight = 0.1F;
/// 陀螺仪零偏校准需要的采样数量
constexpr int kImuBiasCalibrationSamples = 32;
/// 用于确认数据流连续性的有效样本数量阈值
constexpr uint32_t kImuContinuityEvidenceSamples = 32;
/// 异步 IMU 采样周期；与 200Hz 控制周期对齐
constexpr auto kImuSamplerPeriod = std::chrono::milliseconds(5);

enum class CachedInvalidKind : uint8_t {
    kNone = 0,
    kReadFailed = 1,
    kInjected = 2,
};

struct ImuSampleSnapshot {
    port::ImuSample sample{};
    uint64_t generation = 0;
    CachedInvalidKind invalid_kind = CachedInvalidKind::kNone;
};

uint32_t FloatBits(float value) {
    uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "float bit packing expects 32-bit float");
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

float FloatFromBits(uint32_t bits) {
    float value = 0.0F;
    static_assert(sizeof(bits) == sizeof(value), "float bit unpacking expects 32-bit float");
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

/// @brief 从环境变量加载正整数（用于故障注入间隔配置）
/// @param key 环境变量名
/// @param diagnostics 诊断输出接口
/// @param now_ms 当前时间戳
/// @return 解析得到的正整数，无效或未设置时返回 0
int LoadPositiveIntervalEnv(const char* key, port::DiagnosticSink& diagnostics, uint64_t now_ms) {
    const char* value = std::getenv(key);
    if (value == nullptr || value[0] == '\0') {
        return 0;
    }
    const std::optional<int> parsed = port::ParsePositiveIntStrict(value);
    if (parsed.has_value()) {
        return *parsed;
    }
    port::EmitRateLimited(diagnostics,
                          {port::DiagnosticLevel::kWarning,
                           "imu.inject.invalid_env",
                           std::string("ignoring invalid fault-injection interval for ") + key + "=" + value,
                           now_ms},
                          1000);
    return 0;
}

/// @brief 根据 IMU 类型码返回可读的名称字符串
/// @param imu_type IMU 类型标识字节
/// @return IMU 型号名称（如 "imu660ra"）
/// @brief IMU 适配器类
///
/// 实现 port::IImuAdapter 接口，持有具体 ImuDevice 资源 owner。
/// 支持 direct-match 和 adaptation-hook 两种模式。
/// 内部包含加速度低通滤波和陀螺仪零偏校准逻辑。
class ImuAdapter final : public port::IImuAdapter {
public:
    ~ImuAdapter() override { StopSampler(); }

    /// @brief 初始化 IMU 适配器
    /// @param profile 硬件描述文件（检查 IMU 子系统是否启用及其模式）
    /// @param diagnostics 诊断输出接口
    /// @return 初始化成功返回 true
    bool Initialize(const port::HardwareProfile& profile, port::DiagnosticSink& diagnostics) override {
        if (!port::IsEnabled(profile.imu)) {
            diagnostics.Emit({port::DiagnosticLevel::kInfo,
                              "imu.disabled",
                              "imu subsystem disabled by hardware profile",
                              port::NowMs()});
            enabled_ = false;
            ready_ = false;
            inject_invalid_every_n_ = 0;
            return true;
        }

        enabled_ = true;
        adaptation_hook_ = profile.imu.mode == port::SubsystemMode::kAdaptationHook;
        hook_name_ = profile.imu.hook;
        inject_invalid_every_n_ = LoadPositiveIntervalEnv("LS2K_FAULT_INJECT_IMU_INVALID_EVERY_N",
                                                          diagnostics,
                                                          port::NowMs());

        if (adaptation_hook_) {
            ready_ = true;
            diagnostics.Emit({port::DiagnosticLevel::kWarning,
                              "imu.init.hook",
                              "imu direct path bypassed; adaptation hook selected: " + hook_name_,
                              port::NowMs()});
            return true;
        }

        const true_ls2k0300::ImuInitResult init = imu_.Initialize();
        ready_ = init.ready;
        ResetCalibrationState();
        diagnostics.Emit({ready_ ? port::DiagnosticLevel::kInfo : port::DiagnosticLevel::kFailSafe,
                          "imu.init",
                          ready_ ? std::string("imu initialized: mode=") +
                                       true_ls2k0300::ImuIoModeName(init.mode)
                                 : std::string("imu unavailable: ") +
                                       true_ls2k0300::ImuStatusName(init.status),
                          port::NowMs()});
        diagnostics.Emit({ready_ ? port::DiagnosticLevel::kInfo : port::DiagnosticLevel::kWarning,
                          "imu.detect",
                          std::string("imu detection path selected: ") +
                              true_ls2k0300::ImuTypeName(init.type) + " source=" +
                              (imu_.source().empty() ? "unresolved" : imu_.source()),
                          port::NowMs()});
        if (ready_) {
            PrimeBiasCalibration(diagnostics);
            try {
                StartSampler();
            } catch (const std::exception& ex) {
                ready_ = false;
                diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                                  "imu.sampler.start_failed",
                                  std::string("imu async sampler failed to start: ") + ex.what(),
                                  port::NowMs()});
                return false;
            }
        }
        return ready_;
    }

    /// @brief 读取一帧 IMU 样本数据
    /// @param diagnostics 诊断输出接口
    /// @return 归一化后的 IMU 样本（含加速度和角速度）
    port::ImuSample Read(port::DiagnosticSink& diagnostics) override {
        port::ImuSample out{};
        out.capture_time_ms = port::NowMs();
        if (!enabled_ || !ready_) {
            return out;
        }

        if (adaptation_hook_) {
            out.valid = false;
            port::EmitRateLimited(diagnostics,
                                  {port::DiagnosticLevel::kWarning,
                                   "imu.hook.read",
                                   "imu adaptation hook selected with no concrete phase-1 implementation: " +
                                       hook_name_,
                                   out.capture_time_ms},
                                  1000);
            return out;
        }

        const ImuSampleSnapshot cached = SnapshotCachedSample();
        if (cached.generation == 0) {
            return out;
        }

        out = cached.sample;
        if (cached.generation == last_observed_generation_) {
            return out;
        }
        last_observed_generation_ = cached.generation;

        if (!out.valid) {
            if (valid_streak_ > 0) {
                continuity_reported_ = false;
            }
            valid_streak_ = 0;
            ++invalid_streak_;
            const bool injected = cached.invalid_kind == CachedInvalidKind::kInjected;
            port::EmitRateLimited(diagnostics,
                                  {port::DiagnosticLevel::kWarning,
                                   injected ? "imu.inject.invalid" : "imu.read.invalid",
                                   injected ? "injecting bounded Phase B invalid-IMU fault on the accepted runtime entrypoint"
                                            : "imu sample unavailable",
                                   out.capture_time_ms},
                                  1000);
            return out;
        }

        if (invalid_streak_ > 0) {
            diagnostics.Emit({port::DiagnosticLevel::kInfo,
                              "imu.read.recovered",
                              "imu sample stream recovered after " + std::to_string(invalid_streak_) +
                                  " invalid read(s)",
                              out.capture_time_ms});
            invalid_streak_ = 0;
        }

        ++valid_streak_;
        if (!continuity_reported_ && valid_streak_ >= kImuContinuityEvidenceSamples) {
            diagnostics.Emit({port::DiagnosticLevel::kInfo,
                              "imu.continuity.ready",
                              "imu sample stream stayed valid for " +
                                  std::to_string(kImuContinuityEvidenceSamples) +
                                  " consecutive reads after bridge normalization",
                              out.capture_time_ms});
            continuity_reported_ = true;
        }

        out.valid = true;
        return out;
    }

    /// @brief 关闭 IMU 适配器
    /// @param diagnostics 诊断输出接口
    void Shutdown(port::DiagnosticSink& diagnostics) override {
        ready_ = false;
        StopSampler();
        if (enabled_ && !adaptation_hook_) {
            imu_.Shutdown();
        }
        ResetCalibrationState();
        diagnostics.Emit({port::DiagnosticLevel::kInfo,
                          "imu.shutdown",
                          "imu adapter shutdown complete",
                          port::NowMs()});
    }

    /// @brief 检查 IMU 是否已就绪
    /// @return true 表示 IMU 可用
    bool Ready() const override { return ready_; }

private:
    /// @brief 重置所有校准和连续性状态
    void ResetCalibrationState() {
        gyro_bias_raw_ = {};
        filtered_acc_ = {};
        have_filtered_acc_ = false;
        valid_streak_ = 0;
        invalid_streak_ = 0;
        continuity_reported_ = false;
        read_count_ = 0;
        last_observed_generation_ = 0;
        cached_sequence_.store(0);
        cached_capture_time_ms_.store(0);
        cached_valid_.store(false);
        cached_invalid_kind_.store(static_cast<uint8_t>(CachedInvalidKind::kNone));
        cached_acc_x_bits_.store(FloatBits(0.0F));
        cached_acc_y_bits_.store(FloatBits(0.0F));
        cached_acc_z_bits_.store(FloatBits(0.0F));
        cached_gyro_x_bits_.store(FloatBits(0.0F));
        cached_gyro_y_bits_.store(FloatBits(0.0F));
        cached_gyro_z_bits_.store(FloatBits(0.0F));
    }

    /// @brief 启动异步采样线程
    void StartSampler() {
        StopSampler();
        sampler_running_.store(true);
        sampler_thread_ = std::thread([this]() { SamplerLoop(); });
    }

    /// @brief 停止异步采样线程
    void StopSampler() {
        sampler_running_.store(false);
        if (sampler_thread_.joinable()) {
            sampler_thread_.join();
        }
    }

    /// @brief IMU 采样线程主循环
    void SamplerLoop() {
        auto next_wakeup = std::chrono::steady_clock::now();
        while (sampler_running_.load()) {
            PublishCachedSample(PollImuSample());
            next_wakeup += kImuSamplerPeriod;
            std::this_thread::sleep_until(next_wakeup);
            if (std::chrono::steady_clock::now() > next_wakeup + kImuSamplerPeriod) {
                next_wakeup = std::chrono::steady_clock::now();
            }
        }
    }

    /// @brief 执行一次底层 IMU 采样并归一化
    ImuSampleSnapshot PollImuSample() {
        ImuSampleSnapshot cached{};
        cached.sample.capture_time_ms = port::NowMs();

        ++read_count_;
        if (inject_invalid_every_n_ > 0 &&
            read_count_ % static_cast<uint64_t>(inject_invalid_every_n_) == 0) {
            cached.invalid_kind = CachedInvalidKind::kInjected;
            return cached;
        }

        const true_ls2k0300::ImuRawSample sample = imu_.ReadRawSample();
        if (!sample.valid) {
            cached.invalid_kind = CachedInvalidKind::kReadFailed;
            return cached;
        }

        cached.sample = NormalizeBridgeSample(sample, cached.sample.capture_time_ms);
        return cached;
    }

    /// @brief 将桥接层原始样本转换为控制层 IMU 样本
    port::ImuSample NormalizeBridgeSample(const true_ls2k0300::ImuRawSample& sample, uint64_t capture_time_ms) {
        port::ImuSample out{};
        out.capture_time_ms = capture_time_ms;
        const std::array<float, 3> acc_mps2 = {static_cast<float>(sample.acc_x) * kAccelMetersPerSecPerCount,
                                               static_cast<float>(sample.acc_y) * kAccelMetersPerSecPerCount,
                                               static_cast<float>(sample.acc_z) * kAccelMetersPerSecPerCount};
        if (!have_filtered_acc_) {
            filtered_acc_ = acc_mps2;
            have_filtered_acc_ = true;
        } else {
            for (std::size_t i = 0; i < filtered_acc_.size(); ++i) {
                filtered_acc_[i] =
                    acc_mps2[i] * kAccelFilterNewWeight + filtered_acc_[i] * kAccelFilterOldWeight;
            }
        }

        out.valid = true;
        out.acc_x = filtered_acc_[0];
        out.acc_y = filtered_acc_[1];
        out.acc_z = filtered_acc_[2];
        out.gyro_x =
            (static_cast<float>(sample.gyro_x) - gyro_bias_raw_[0]) * kGyroRadPerSecPerCount;
        out.gyro_y =
            (static_cast<float>(sample.gyro_y) - gyro_bias_raw_[1]) * kGyroRadPerSecPerCount;
        out.gyro_z =
            (static_cast<float>(sample.gyro_z) - gyro_bias_raw_[2]) * kGyroRadPerSecPerCount;
        return out;
    }

    /// @brief 发布最新采样结果
    void PublishCachedSample(const ImuSampleSnapshot& cached) {
        const uint64_t current_sequence = cached_sequence_.load(std::memory_order_relaxed);
        cached_sequence_.store(current_sequence + 1, std::memory_order_release);
        cached_capture_time_ms_.store(cached.sample.capture_time_ms, std::memory_order_relaxed);
        cached_valid_.store(cached.sample.valid, std::memory_order_relaxed);
        cached_invalid_kind_.store(static_cast<uint8_t>(cached.invalid_kind), std::memory_order_relaxed);
        cached_acc_x_bits_.store(FloatBits(cached.sample.acc_x), std::memory_order_relaxed);
        cached_acc_y_bits_.store(FloatBits(cached.sample.acc_y), std::memory_order_relaxed);
        cached_acc_z_bits_.store(FloatBits(cached.sample.acc_z), std::memory_order_relaxed);
        cached_gyro_x_bits_.store(FloatBits(cached.sample.gyro_x), std::memory_order_relaxed);
        cached_gyro_y_bits_.store(FloatBits(cached.sample.gyro_y), std::memory_order_relaxed);
        cached_gyro_z_bits_.store(FloatBits(cached.sample.gyro_z), std::memory_order_relaxed);
        cached_sequence_.store(current_sequence + 2, std::memory_order_release);
    }

    /// @brief 读取最近一次采样结果
    ImuSampleSnapshot SnapshotCachedSample() const {
        for (int attempt = 0; attempt < 3; ++attempt) {
            const uint64_t sequence_before = cached_sequence_.load(std::memory_order_acquire);
            if (sequence_before == 0 || (sequence_before % 2) != 0) {
                continue;
            }

            ImuSampleSnapshot snapshot{};
            snapshot.generation = sequence_before / 2;
            snapshot.sample.capture_time_ms = cached_capture_time_ms_.load(std::memory_order_relaxed);
            snapshot.sample.valid = cached_valid_.load(std::memory_order_relaxed);
            snapshot.invalid_kind =
                static_cast<CachedInvalidKind>(cached_invalid_kind_.load(std::memory_order_relaxed));
            snapshot.sample.acc_x = FloatFromBits(cached_acc_x_bits_.load(std::memory_order_relaxed));
            snapshot.sample.acc_y = FloatFromBits(cached_acc_y_bits_.load(std::memory_order_relaxed));
            snapshot.sample.acc_z = FloatFromBits(cached_acc_z_bits_.load(std::memory_order_relaxed));
            snapshot.sample.gyro_x = FloatFromBits(cached_gyro_x_bits_.load(std::memory_order_relaxed));
            snapshot.sample.gyro_y = FloatFromBits(cached_gyro_y_bits_.load(std::memory_order_relaxed));
            snapshot.sample.gyro_z = FloatFromBits(cached_gyro_z_bits_.load(std::memory_order_relaxed));

            const uint64_t sequence_after = cached_sequence_.load(std::memory_order_acquire);
            if (sequence_before == sequence_after) {
                return snapshot;
            }
        }
        return {};
    }

    /// @brief 执行初始陀螺仪零偏校准
    ///
    /// 在适配器初始化时采集若干静态样本来估算陀螺仪的零偏，
    /// 后续读取时将减去该零偏值以获得更准确的角速度。
    /// @param diagnostics 诊断输出接口
    void PrimeBiasCalibration(port::DiagnosticSink& diagnostics) {
        std::array<double, 3> gyro_sum{};
        int collected = 0;
        for (int i = 0; i < kImuBiasCalibrationSamples; ++i) {
            const true_ls2k0300::ImuRawSample sample = imu_.ReadRawSample();
            if (!sample.valid) {
                continue;
            }
            if (sample.gyro_x == 0 && sample.gyro_y == 0 && sample.gyro_z == 0) {
                continue;
            }
            gyro_sum[0] += sample.gyro_x;
            gyro_sum[1] += sample.gyro_y;
            gyro_sum[2] += sample.gyro_z;
            ++collected;
        }

        if (collected == 0) {
            diagnostics.Emit({port::DiagnosticLevel::kWarning,
                              "imu.calibration.partial",
                              "imu gyro zero-bias calibration could not collect valid startup samples; using raw origin",
                              port::NowMs()});
            return;
        }

        for (std::size_t i = 0; i < gyro_bias_raw_.size(); ++i) {
            gyro_bias_raw_[i] = static_cast<float>(gyro_sum[i] / collected);
        }

        diagnostics.Emit({port::DiagnosticLevel::kInfo,
                          "imu.calibration.ready",
                          "imu gyro zero-bias calibrated from " + std::to_string(collected) +
                              " startup sample(s)",
                          port::NowMs()});
    }

    /// IMU 子系统是否启用
    bool enabled_ = false;
    /// IMU 是否已就绪
    bool ready_ = false;
    /// 是否使用适配钩子模式
    bool adaptation_hook_ = false;
    /// 适配钩子名称
    std::string hook_name_ = "direct-match";
    /// 陀螺仪 X/Y/Z 轴零偏原始计数值
    std::array<float, 3> gyro_bias_raw_{};
    /// 加速度低通滤波后的 X/Y/Z 值（m/s^2）
    std::array<float, 3> filtered_acc_{};
    /// 是否已获得初始加速度滤波值
    bool have_filtered_acc_ = false;
    /// 连续有效读取计数
    uint32_t valid_streak_ = 0;
    /// 连续无效读取计数
    uint32_t invalid_streak_ = 0;
    /// 是否已上报连续性就绪诊断
    bool continuity_reported_ = false;
    /// 读取计数（用于故障注入周期性）
    uint64_t read_count_ = 0;
    /// 已被控制循环观察到的异步采样代数
    uint64_t last_observed_generation_ = 0;
    /// 故障注入间隔；初始化时读取，避免热路径解析环境变量
    int inject_invalid_every_n_ = 0;
    /// 异步采样线程是否运行
    std::atomic<bool> sampler_running_{false};
    /// 异步采样线程
    std::thread sampler_thread_{};
    /// 异步采样发布序列号；奇数表示正在发布，偶数表示稳定
    std::atomic<uint64_t> cached_sequence_{0};
    /// 最近一次 IMU 采样时间
    std::atomic<uint64_t> cached_capture_time_ms_{0};
    /// 最近一次 IMU 采样是否有效
    std::atomic<bool> cached_valid_{false};
    /// 最近一次无效采样原因
    std::atomic<uint8_t> cached_invalid_kind_{static_cast<uint8_t>(CachedInvalidKind::kNone)};
    /// 最近一次加速度 X
    std::atomic<uint32_t> cached_acc_x_bits_{0};
    /// 最近一次加速度 Y
    std::atomic<uint32_t> cached_acc_y_bits_{0};
    /// 最近一次加速度 Z
    std::atomic<uint32_t> cached_acc_z_bits_{0};
    /// 最近一次角速度 X
    std::atomic<uint32_t> cached_gyro_x_bits_{0};
    /// 最近一次角速度 Y
    std::atomic<uint32_t> cached_gyro_y_bits_{0};
    /// 最近一次角速度 Z
    std::atomic<uint32_t> cached_gyro_z_bits_{0};
    /// 具体 IMU 资源 owner；型号、路径、通道 fd 与模式均为对象状态。
    true_ls2k0300::ImuDevice imu_{};
};

}  // namespace

/// @brief 创建 IMU 适配器实例
/// @return 新创建的 ImuAdapter 智能指针
std::unique_ptr<port::IImuAdapter> MakeImuAdapter() {
    return std::make_unique<ImuAdapter>();
}

}  // namespace ls2k::platform
