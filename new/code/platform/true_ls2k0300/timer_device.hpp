#pragma once

#include <cstdint>
#include <memory>

namespace ls2k::platform::true_ls2k0300 {

struct TimerTick final {
    std::uint64_t expirations{0};
};

struct TimerFailure final {
    enum Code : std::uint8_t {
        kPoll = 0,
        kPollSystemCall = kPoll,
        kTimerFdPollEvent,
        kStopFdPollEvent,
        kTimerRead,
    };

    Code code{kPollSystemCall};
    int system_errno{0};
    int poll_result{0};
    std::int16_t timer_revents{0};
    std::int16_t stop_revents{0};
    std::int64_t read_bytes{0};

    friend constexpr bool operator==(TimerFailure failure, Code code) noexcept {
        return failure.code == code;
    }

    friend constexpr bool operator!=(TimerFailure failure, Code code) noexcept {
        return !(failure == code);
    }
};

class TimerDevice final {
public:
    using TickCallback = void (*)(void* context, TimerTick tick) noexcept;
    using FailureCallback = void (*)(void* context, TimerFailure failure) noexcept;

    TimerDevice();
    ~TimerDevice() noexcept;

    TimerDevice(const TimerDevice&) = delete;
    TimerDevice& operator=(const TimerDevice&) = delete;
    TimerDevice(TimerDevice&&) noexcept;
    TimerDevice& operator=(TimerDevice&&) noexcept;

    [[nodiscard]] bool Start(
        std::uint32_t period_ms,
        TickCallback callback,
        void* callback_context,
        FailureCallback on_failure = nullptr,
        void* failure_context = nullptr) noexcept;
    void Stop() noexcept;
    [[nodiscard]] bool Running() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace ls2k::platform::true_ls2k0300
