#include "platform/true_ls2k0300/timer_device.hpp"

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/timerfd.h>
#include <thread>
#include <utility>

#include "platform/linux/linux_io.hpp"
#include "port/thread_scheduling.hpp"

namespace ls2k::platform::true_ls2k0300 {
namespace {

timespec ToTimespec(std::uint32_t period_ms) noexcept {
    timespec spec{};
    spec.tv_sec = static_cast<time_t>(period_ms / 1000U);
    spec.tv_nsec = static_cast<long>((period_ms % 1000U) * 1000000UL);
    return spec;
}

bool ArmTimerFd(int timer_fd, std::uint32_t period_ms) noexcept {
    itimerspec schedule{};
    schedule.it_value = ToTimespec(period_ms);
    schedule.it_interval = schedule.it_value;
    return timerfd_settime(timer_fd, 0, &schedule, nullptr) == 0;
}

void EmitFailureDiagnostic(const TimerFailure& failure) noexcept {
    std::fprintf(stderr,
                 "[timer.device.failure] code=%u poll_result=%d errno=%d timer_revents=0x%x "
                 "stop_revents=0x%x read_bytes=%lld\n",
                 static_cast<unsigned int>(failure.code),
                 failure.poll_result,
                 failure.system_errno,
                 static_cast<unsigned int>(static_cast<std::uint16_t>(failure.timer_revents)),
                 static_cast<unsigned int>(static_cast<std::uint16_t>(failure.stop_revents)),
                 static_cast<long long>(failure.read_bytes));
}

}  // namespace

struct TimerDevice::Impl final {
    ~Impl() noexcept { Stop(); }

    bool Start(
        std::uint32_t period_ms,
        TickCallback callback,
        void* callback_context,
        FailureCallback on_failure,
        void* failure_context) noexcept {
        Stop();
        if (period_ms == 0U || !callback) {
            return false;
        }

        linux_io::UniqueFd timer_fd(timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC));
        if (!timer_fd || !ArmTimerFd(timer_fd.get(), period_ms)) {
            return false;
        }
        linux_io::UniqueFd stop_fd(eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK));
        if (!stop_fd) {
            return false;
        }

        timer_fd_ = std::move(timer_fd);
        stop_fd_ = std::move(stop_fd);
        callback_ = callback;
        callback_context_ = callback_context;
        on_failure_ = on_failure;
        failure_context_ = failure_context;
        running_.store(true);
        try {
            worker_ = std::thread([this]() noexcept { Run(); });
        } catch (...) {
            running_.store(false);
            ClearCallbacks();
            static_cast<void>(timer_fd_.Reset());
            static_cast<void>(stop_fd_.Reset());
            return false;
        }
        return true;
    }

    void Stop() noexcept {
        running_.store(false);
        SignalStop();
        if (worker_.joinable()) {
            if (worker_.get_id() == std::this_thread::get_id()) {
                return;
            }
            worker_.join();
        }
        static_cast<void>(timer_fd_.Reset());
        static_cast<void>(stop_fd_.Reset());
        ClearCallbacks();
    }

    [[nodiscard]] bool Running() const noexcept { return running_.load(); }

private:
    void SignalStop() noexcept {
        if (!stop_fd_) {
            return;
        }
        const std::uint64_t signal = 1;
        for (;;) {
            const linux_io::IoResult result =
                linux_io::WriteOnce(linux_io::ProductionSyscalls(), stop_fd_.get(), &signal, sizeof(signal));
            if (result.ok() || result.error == linux_io::IoError::kWouldBlock) {
                return;
            }
            if (result.error != linux_io::IoError::kInterrupted) {
                return;
            }
        }
    }

    static void DrainStopSignal(int stop_fd) noexcept {
        std::uint64_t signal = 0;
        for (;;) {
            const linux_io::IoResult result =
                linux_io::ReadOnce(linux_io::ProductionSyscalls(), stop_fd, &signal, sizeof(signal));
            if (result.ok() || result.error != linux_io::IoError::kInterrupted) {
                return;
            }
        }
    }

    void NotifyFailure(const TimerFailure& failure) noexcept {
        EmitFailureDiagnostic(failure);
        if (on_failure_ != nullptr) {
            on_failure_(failure_context_, failure);
        }
    }

    void Run() noexcept {
        port::ApplyThreadSchedulingProfile(port::ThreadSchedulingRole::kControlTimer);
        pollfd descriptors[2]{{timer_fd_.get(), POLLIN, 0}, {stop_fd_.get(), POLLIN, 0}};
        TimerFailure failure{};
        bool failed = false;

        while (running_.load()) {
            int poll_result = -1;
            do {
                poll_result = linux_io::ProductionSyscalls().functions().poll(descriptors, 2, -1);
            } while (poll_result < 0 && errno == EINTR && running_.load());

            if (poll_result < 0) {
                failed = running_.load();
                failure = TimerFailure{TimerFailure::kPollSystemCall,
                                       errno,
                                       poll_result,
                                       descriptors[0].revents,
                                       descriptors[1].revents,
                                       0};
                break;
            }
            if ((descriptors[1].revents & POLLIN) != 0) {
                DrainStopSignal(stop_fd_.get());
                break;
            }
            if ((descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                failed = running_.load();
                failure = TimerFailure{TimerFailure::kTimerFdPollEvent,
                                       0,
                                       poll_result,
                                       descriptors[0].revents,
                                       descriptors[1].revents,
                                       0};
                break;
            }
            if ((descriptors[1].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                failed = running_.load();
                failure = TimerFailure{TimerFailure::kStopFdPollEvent,
                                       0,
                                       poll_result,
                                       descriptors[0].revents,
                                       descriptors[1].revents,
                                       0};
                break;
            }
            if ((descriptors[0].revents & POLLIN) == 0) {
                continue;
            }

            std::uint64_t expirations = 0;
            linux_io::IoResult read_result{};
            do {
                read_result = linux_io::ReadOnce(
                    linux_io::ProductionSyscalls(), timer_fd_.get(), &expirations, sizeof(expirations));
            } while (read_result.error == linux_io::IoError::kInterrupted);
            if (!read_result.ok() || read_result.bytes_transferred != static_cast<ssize_t>(sizeof(expirations))) {
                failed = running_.load();
                failure = TimerFailure{TimerFailure::kTimerRead,
                                       read_result.system_errno,
                                       poll_result,
                                       descriptors[0].revents,
                                       descriptors[1].revents,
                                       read_result.bytes_transferred};
                break;
            }
            if (expirations == 0 || !running_.load()) {
                continue;
            }
            callback_(callback_context_, TimerTick{expirations});
        }

        running_.store(false);
        if (failed) {
            NotifyFailure(failure);
        }
    }

    void ClearCallbacks() noexcept {
        callback_ = nullptr;
        callback_context_ = nullptr;
        on_failure_ = nullptr;
        failure_context_ = nullptr;
    }

    std::atomic<bool> running_{false};
    linux_io::UniqueFd timer_fd_{};
    linux_io::UniqueFd stop_fd_{};
    std::thread worker_{};
    TickCallback callback_{nullptr};
    void* callback_context_{nullptr};
    FailureCallback on_failure_{nullptr};
    void* failure_context_{nullptr};
};

TimerDevice::TimerDevice() : impl_(std::make_unique<Impl>()) {}

TimerDevice::~TimerDevice() noexcept = default;

TimerDevice::TimerDevice(TimerDevice&&) noexcept = default;

TimerDevice& TimerDevice::operator=(TimerDevice&& other) noexcept {
    if (this != &other) {
        if (impl_) {
            impl_->Stop();
        }
        impl_ = std::move(other.impl_);
    }
    return *this;
}

bool TimerDevice::Start(
    std::uint32_t period_ms,
    TickCallback callback,
    void* callback_context,
    FailureCallback on_failure,
    void* failure_context) noexcept {
    return impl_ &&
           impl_->Start(period_ms, callback, callback_context, on_failure, failure_context);
}

void TimerDevice::Stop() noexcept {
    if (impl_) {
        impl_->Stop();
    }
}

bool TimerDevice::Running() const noexcept {
    return impl_ && impl_->Running();
}

}  // namespace ls2k::platform::true_ls2k0300
