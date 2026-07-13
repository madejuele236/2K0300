#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "platform/true_ls2k0300/bridge.hpp"

namespace {

constexpr std::size_t kSamples = 1000;
constexpr std::int64_t kPeriodUs = 5000;

std::int64_t ToUs(const timespec& value) {
    return static_cast<std::int64_t>(value.tv_sec) * 1000000LL +
           static_cast<std::int64_t>(value.tv_nsec) / 1000LL;
}

}  // namespace

int main() {
    std::array<timespec, kSamples> timestamps{};
    std::atomic<std::size_t> count{0};
    std::atomic<bool> clock_failed{false};
    std::atomic<bool> timer_failed{false};
    ls2k::platform::true_ls2k0300::TimerBridge timer;
    if (!timer.Start(
            5,
            [&]() {
                const std::size_t index = count.load(std::memory_order_relaxed);
                if (index >= timestamps.size()) {
                    return;
                }
                if (clock_gettime(CLOCK_MONOTONIC, &timestamps[index]) != 0) {
                    clock_failed.store(true, std::memory_order_release);
                    return;
                }
                count.store(index + 1, std::memory_order_release);
            },
            [&](std::string) { timer_failed.store(true, std::memory_order_release); })) {
        std::puts("baseline_timer result=START_FAILED");
        return 1;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (count.load(std::memory_order_acquire) < kSamples &&
           !clock_failed.load(std::memory_order_acquire) &&
           !timer_failed.load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    timer.Stop();

    const std::size_t samples = count.load(std::memory_order_acquire);
    if (samples != kSamples || clock_failed.load() || timer_failed.load()) {
        std::printf("baseline_timer samples=%zu clock_failed=%d timer_failed=%d result=FAIL\n",
                    samples,
                    clock_failed.load() ? 1 : 0,
                    timer_failed.load() ? 1 : 0);
        return 2;
    }

    std::int64_t minimum = INT64_MAX;
    std::int64_t maximum = 0;
    std::int64_t maximum_jitter = 0;
    for (std::size_t index = 1; index < timestamps.size(); ++index) {
        const std::int64_t interval = ToUs(timestamps[index]) - ToUs(timestamps[index - 1]);
        if (interval < minimum) minimum = interval;
        if (interval > maximum) maximum = interval;
        const std::int64_t jitter = interval >= kPeriodUs
                                        ? interval - kPeriodUs
                                        : kPeriodUs - interval;
        if (jitter > maximum_jitter) maximum_jitter = jitter;
    }
    std::printf("baseline_timer samples=%zu period_us=%lld min_interval_us=%lld "
                "max_interval_us=%lld max_absolute_jitter_us=%lld result=PASS\n",
                samples,
                static_cast<long long>(kPeriodUs),
                static_cast<long long>(minimum),
                static_cast<long long>(maximum),
                static_cast<long long>(maximum_jitter));
    return 0;
}
