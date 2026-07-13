#include <atomic>
#include <chrono>
#include <cstdlib>
#include <thread>
#include <type_traits>

#include "platform/true_ls2k0300/timer_device.hpp"

namespace {

using ls2k::platform::true_ls2k0300::TimerDevice;
using ls2k::platform::true_ls2k0300::TimerFailure;
using ls2k::platform::true_ls2k0300::TimerTick;

void Expect(bool condition) {
    if (!condition) {
        std::abort();
    }
}

void NoopTick(void*, TimerTick) noexcept {}

struct PeriodState final {
    std::atomic<int> callbacks{0};
    std::atomic<std::uint64_t> largest_expiration_count{0};
    std::atomic<int> failures{0};
};

void RecordPeriodTick(void* context, TimerTick tick) noexcept {
    auto& state = *static_cast<PeriodState*>(context);
    ++state.callbacks;
    std::uint64_t observed = state.largest_expiration_count.load();
    while (observed < tick.expirations &&
           !state.largest_expiration_count.compare_exchange_weak(observed, tick.expirations)) {
    }
    if (state.callbacks.load() == 1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
}

void RecordFailure(void* context, TimerFailure) noexcept {
    ++static_cast<PeriodState*>(context)->failures;
}

template <typename Predicate>
bool WaitFor(Predicate predicate, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return predicate();
}

void TestContractAndValidation() {
    static_assert(std::is_trivially_copyable_v<TimerFailure>);
    static_assert(sizeof(TimerFailure) <= 32);
    static_assert(!std::is_copy_constructible_v<TimerDevice>);
    static_assert(!std::is_copy_assignable_v<TimerDevice>);
    static_assert(std::is_nothrow_move_constructible_v<TimerDevice>);
    static_assert(std::is_nothrow_move_assignable_v<TimerDevice>);
    static_assert(std::is_pointer_v<TimerDevice::TickCallback>);
    static_assert(std::is_pointer_v<TimerDevice::FailureCallback>);
    static_assert(std::is_nothrow_invocable_r_v<void,
                                                TimerDevice::TickCallback,
                                                void*,
                                                TimerTick>);
    static_assert(std::is_nothrow_invocable_r_v<void,
                                                TimerDevice::FailureCallback,
                                                void*,
                                                TimerFailure>);

    TimerDevice timer;
    Expect(!timer.Running());
    Expect(!timer.Start(0, &NoopTick, nullptr));
    Expect(!timer.Start(5, nullptr, nullptr));
    Expect(!timer.Running());
}

void TestPeriodOverrunStopAndThreadExit() {
    TimerDevice timer;
    PeriodState state{};

    Expect(timer.Start(5, &RecordPeriodTick, &state, &RecordFailure, &state));
    Expect(timer.Running());
    Expect(WaitFor([&] { return state.callbacks.load() >= 2; },
                   std::chrono::milliseconds(500)));
    Expect(state.largest_expiration_count.load() > 1);

    const auto stop_begin = std::chrono::steady_clock::now();
    timer.Stop();
    const auto stop_elapsed = std::chrono::steady_clock::now() - stop_begin;
    Expect(!timer.Running());
    Expect(stop_elapsed < std::chrono::milliseconds(200));
    Expect(state.failures.load() == 0);
    const int stopped_count = state.callbacks.load();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    Expect(state.callbacks.load() == stopped_count);
}

}  // namespace

int main() {
    TestContractAndValidation();
    TestPeriodOverrunStopAndThreadExit();
    return EXIT_SUCCESS;
}
