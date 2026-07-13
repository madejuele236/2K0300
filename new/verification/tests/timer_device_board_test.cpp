#include "platform/true_ls2k0300/timer_device.hpp"

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <thread>
#include <time.h>

namespace {

using ls2k::platform::true_ls2k0300::TimerDevice;
using ls2k::platform::true_ls2k0300::TimerFailure;
using ls2k::platform::true_ls2k0300::TimerTick;

// Source: new/config/default_params.json, control_period_ms.
constexpr std::uint32_t kRequestedPeriodMs = 5U;
constexpr std::size_t kSampleCount = 1000U;

struct BoardTimerState final {
    std::array<timespec, kSampleCount> timestamps{};
    std::array<std::uint64_t, kSampleCount> expirations{};
    std::atomic<std::size_t> recorded_count{0U};
    std::atomic<bool> clock_read_failed{false};
    std::atomic<int> clock_errno{0};
    std::atomic<bool> timer_failure_seen{false};
    TimerFailure timer_failure{};
};

void RecordTick(void* context, TimerTick tick) noexcept {
    auto& state = *static_cast<BoardTimerState*>(context);
    const std::size_t index = state.recorded_count.load(std::memory_order_relaxed);
    if (index >= state.timestamps.size()) {
        return;
    }

    timespec timestamp{};
    if (clock_gettime(CLOCK_MONOTONIC, &timestamp) != 0) {
        state.clock_errno.store(errno, std::memory_order_relaxed);
        state.clock_read_failed.store(true, std::memory_order_relaxed);
    }
    state.timestamps[index] = timestamp;
    state.expirations[index] = tick.expirations;
    state.recorded_count.store(index + 1U, std::memory_order_release);
}

void RecordFailure(void* context, TimerFailure failure) noexcept {
    auto& state = *static_cast<BoardTimerState*>(context);
    state.timer_failure = failure;
    state.timer_failure_seen.store(true, std::memory_order_release);
}

std::int64_t ToNanoseconds(const timespec& value) noexcept {
    return static_cast<std::int64_t>(value.tv_sec) * 1000000000LL +
           static_cast<std::int64_t>(value.tv_nsec);
}

}  // namespace

int main() {
    BoardTimerState state{};

    TimerDevice timer;
    const bool started = timer.Start(
        kRequestedPeriodMs, &RecordTick, &state, &RecordFailure, &state);

    // This is only a finite-run watchdog, not a jitter acceptance threshold.
    const auto watchdog = std::chrono::milliseconds(
        static_cast<std::int64_t>(kRequestedPeriodMs) *
            static_cast<std::int64_t>(kSampleCount) +
        5000LL);
    const auto deadline = std::chrono::steady_clock::now() + watchdog;
    while (started &&
           state.recorded_count.load(std::memory_order_acquire) < kSampleCount &&
           !state.timer_failure_seen.load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    timer.Stop();

    const std::size_t samples = state.recorded_count.load(std::memory_order_acquire);
    std::int64_t min_interval_ns = std::numeric_limits<std::int64_t>::max();
    std::int64_t max_interval_ns = std::numeric_limits<std::int64_t>::min();
    std::int64_t max_absolute_jitter_ns = 0;
    std::uint64_t missed_expirations = 0U;
    std::uint64_t max_expirations = 0U;
    bool timestamps_strictly_increasing = true;
    bool expiration_reads_valid = true;
    const std::int64_t requested_period_ns =
        static_cast<std::int64_t>(kRequestedPeriodMs) * 1000000LL;

    for (std::size_t index = 0U; index < samples; ++index) {
        const std::uint64_t count = state.expirations[index];
        expiration_reads_valid = expiration_reads_valid && count > 0U;
        if (count > max_expirations) {
            max_expirations = count;
        }
        if (count > 1U) {
            missed_expirations += count - 1U;
        }

        if (index == 0U) {
            continue;
        }
        const std::int64_t interval_ns =
            ToNanoseconds(state.timestamps[index]) -
            ToNanoseconds(state.timestamps[index - 1U]);
        timestamps_strictly_increasing = timestamps_strictly_increasing && interval_ns > 0;
        if (interval_ns < min_interval_ns) {
            min_interval_ns = interval_ns;
        }
        if (interval_ns > max_interval_ns) {
            max_interval_ns = interval_ns;
        }
        const std::int64_t jitter_ns = interval_ns >= requested_period_ns
                                           ? interval_ns - requested_period_ns
                                           : requested_period_ns - interval_ns;
        if (jitter_ns > max_absolute_jitter_ns) {
            max_absolute_jitter_ns = jitter_ns;
        }
    }

    const bool enough_intervals = samples >= 2U;
    const bool timed_out = started && samples < kSampleCount &&
                           !state.timer_failure_seen.load(std::memory_order_acquire);
    const bool stopped = !timer.Running();
    const bool lifecycle_read_pass =
        started && samples == kSampleCount && !timed_out && stopped &&
        !state.timer_failure_seen.load(std::memory_order_acquire) &&
        !state.clock_read_failed.load(std::memory_order_relaxed) && enough_intervals &&
        timestamps_strictly_increasing && expiration_reads_valid;

    std::cout << "sample_count=" << samples << '\n'
              << "requested_sample_count=" << kSampleCount << '\n'
              << "requested_period_ms=" << kRequestedPeriodMs << '\n'
              << "timestamp_clock=CLOCK_MONOTONIC\n"
              << "interval_count=" << (samples > 0U ? samples - 1U : 0U) << '\n'
              << "min_interval_us="
              << (enough_intervals ? min_interval_ns / 1000LL : 0LL) << '\n'
              << "max_interval_us="
              << (enough_intervals ? max_interval_ns / 1000LL : 0LL) << '\n'
              << "max_absolute_jitter_us=" << max_absolute_jitter_ns / 1000LL << '\n'
              << "missed_or_overrun=" << (missed_expirations > 0U ? "true" : "false") << '\n'
              << "missed_expirations=" << missed_expirations << '\n'
              << "max_expirations_per_callback=" << max_expirations << '\n'
              << "timer_failure="
              << (state.timer_failure_seen.load(std::memory_order_acquire) ? "true" : "false")
              << '\n';
    if (state.timer_failure_seen.load(std::memory_order_acquire)) {
        std::cout << "timer_failure_code="
                  << static_cast<unsigned int>(state.timer_failure.code)
                  << '\n'
                  << "timer_failure_errno=" << state.timer_failure.system_errno << '\n';
    }
    std::cout << "clock_read_failure="
              << (state.clock_read_failed.load(std::memory_order_relaxed) ? "true" : "false")
              << '\n'
              << "clock_errno=" << state.clock_errno.load(std::memory_order_relaxed) << '\n'
              << "timing_threshold=none\n"
              << "timing_assessment=METRICS_ONLY\n"
              << "verdict_scope=lifecycle_read\n"
              << "result=" << (lifecycle_read_pass ? "PASS" : "FAIL") << '\n';

    return lifecycle_read_pass ? 0 : 1;
}
