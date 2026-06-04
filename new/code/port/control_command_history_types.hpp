#ifndef LS2K_PORT_CONTROL_COMMAND_HISTORY_TYPES_HPP
#define LS2K_PORT_CONTROL_COMMAND_HISTORY_TYPES_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace ls2k::port {

/// 控制命令历史采样：只记录已经请求/已经施加到执行器路径的事实。
struct ControlCommandHistorySample {
    uint64_t time_ms = 0;

    bool valid = false;
    bool actuator_applied = false;
    bool diagnostics_only = false;
    bool hold_disarmed = false;
    bool emergency_stop = false;

    int raw_turn_output = 0;
    int applied_turn_output = 0;

    double left_wheel_target = 0.0;
    double right_wheel_target = 0.0;

    int left_drive_pwm = 0;
    int right_drive_pwm = 0;
    int left_brushless_pwm = 0;
    int right_brushless_pwm = 0;
};

struct ControlCommandHistory {
    static constexpr std::size_t kCapacity = 2048;

    void Push(const ControlCommandHistorySample& sample) {
        samples[next_index] = sample;
        next_index = (next_index + 1U) % kCapacity;
        if (count < kCapacity) {
            ++count;
        }
    }

    void Clear() {
        samples = {};
        next_index = 0;
        count = 0;
    }

    const ControlCommandHistorySample& OldestOffset(std::size_t offset) const {
        const std::size_t src = (next_index + kCapacity - count + offset) % kCapacity;
        return samples[src];
    }

    const ControlCommandHistorySample& NewestOffset(std::size_t offset) const {
        const std::size_t src = (next_index + kCapacity - 1U - offset) % kCapacity;
        return samples[src];
    }

    bool LatestBeforeOrAt(uint64_t time_ms, ControlCommandHistorySample& out) const {
        for (std::size_t offset = 0; offset < count; ++offset) {
            const ControlCommandHistorySample& sample = NewestOffset(offset);
            if (sample.time_ms <= time_ms) {
                out = sample;
                return true;
            }
        }
        return false;
    }

    std::array<ControlCommandHistorySample, kCapacity> samples{};
    std::size_t next_index = 0;
    std::size_t count = 0;
};

}  // namespace ls2k::port

#endif  // LS2K_PORT_CONTROL_COMMAND_HISTORY_TYPES_HPP
