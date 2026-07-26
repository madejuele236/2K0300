#include "platform/true_ls2k0300/motor_device.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <fcntl.h>

#include "port/actuator_command_types.hpp"

namespace ls2k::platform::true_ls2k0300 {
namespace {

constexpr int kEscDutyLimit = 1000;

MotorResult Ok() noexcept {
    return {MotorStatus::kOk, 0};
}

MotorResult Failure(MotorStatus status, int system_errno) noexcept {
    return {status, system_errno};
}

bool AcceptedWrite(const linux_io::IoResult& result, std::size_t size) noexcept {
    return result.ok() &&
           (result.bytes_transferred == 0 ||
            result.bytes_transferred == static_cast<ssize_t>(size));
}

}  // namespace

MotorDevice::MotorDevice(MotorPaths paths,
                         const linux_io::SyscallApi& syscalls,
                         MotorIoPolicy io_policy) noexcept
    : syscalls_(&syscalls),
      left_pwm_(paths.left_pwm, syscalls),
      right_pwm_(paths.right_pwm, syscalls),
      left_gpio_(paths.left_gpio, syscalls),
      right_gpio_(paths.right_gpio, syscalls),
      esc_left_pwm_(paths.esc_left_pwm, syscalls),
      esc_right_pwm_(paths.esc_right_pwm, syscalls),
      io_policy_(io_policy) {}

MotorDevice::~MotorDevice() noexcept {
    if (initialized_ || mode_ != MotorIoMode::kUninitialized) {
        static_cast<void>(Stop());
    }
}

MotorResult MotorDevice::Probe(const WritableDevice& device) noexcept {
    errno = 0;
    const int fd = syscalls_->functions().open(
        device.path, O_WRONLY | O_CLOEXEC, 0);
    if (fd < 0) {
        return Failure(MotorStatus::kOpenFailed, errno);
    }

    linux_io::UniqueFd owned(fd, *syscalls_);
    const linux_io::IoResult closed = owned.Reset();
    return closed.ok()
               ? Ok()
               : Failure(MotorStatus::kCloseFailed, closed.system_errno);
}

MotorResult MotorDevice::WriteOpenClose(const char* path,
                                        const void* payload,
                                        std::size_t size) noexcept {
    errno = 0;
    const int fd = syscalls_->functions().open(path, O_WRONLY | O_CLOEXEC, 0);
    if (fd < 0) {
        return Failure(MotorStatus::kOpenFailed, errno);
    }

    linux_io::UniqueFd owned(fd, *syscalls_);
    const linux_io::IoResult written =
        linux_io::WriteOnce(*syscalls_, fd, payload, size);
    const linux_io::IoResult closed = owned.Reset();
    if (!AcceptedWrite(written, size)) {
        return Failure(MotorStatus::kWriteFailed, written.system_errno);
    }
    if (!closed.ok()) {
        return Failure(MotorStatus::kCloseFailed, closed.system_errno);
    }
    return Ok();
}

MotorResult MotorDevice::ClosePersistentDriveOutputs() noexcept {
    MotorResult first_failure = Ok();
    WritableDevice* const devices[] = {
        &left_pwm_, &right_pwm_, &left_gpio_, &right_gpio_};
    for (WritableDevice* device : devices) {
        const linux_io::IoResult closed = device->fd.Reset();
        if (first_failure.ok() && !closed.ok()) {
            first_failure =
                Failure(MotorStatus::kCloseFailed, closed.system_errno);
        }
    }
    return first_failure;
}

bool MotorDevice::OpenPersistentDriveOutputs() noexcept {
    static_cast<void>(ClosePersistentDriveOutputs());
    WritableDevice* const devices[] = {
        &left_pwm_, &right_pwm_, &left_gpio_, &right_gpio_};
    for (WritableDevice* device : devices) {
        errno = 0;
        const int fd = syscalls_->functions().open(
            device->path, O_WRONLY | O_CLOEXEC, 0);
        if (fd < 0) {
            static_cast<void>(ClosePersistentDriveOutputs());
            return false;
        }
        static_cast<void>(device->fd.Reset(fd));
    }
    return true;
}

MotorResult MotorDevice::Write(WritableDevice& device,
                               const void* payload,
                               std::size_t size) noexcept {
    if (!device.fd) {
        return WriteOpenClose(device.path, payload, size);
    }

    const linux_io::IoResult written =
        linux_io::WriteOnce(*syscalls_, device.fd.get(), payload, size);
    if (AcceptedWrite(written, size)) {
        return Ok();
    }

    // Persistent descriptors are an optimization, not a distinct hardware
    // contract. A failed persistent write atomically downgrades the whole drive
    // group, then retries the same command through the verified fallback path.
    const MotorResult closed = ClosePersistentDriveOutputs();
    mode_ = MotorIoMode::kOpenWriteClose;
    const MotorResult retried = WriteOpenClose(device.path, payload, size);
    if (!retried.ok()) {
        return retried;
    }
    return closed.ok() ? Ok() : closed;
}

MotorResult MotorDevice::StopOutputs() noexcept {
    const std::uint16_t zero = 0;
    MotorResult first_failure = Ok();
    WritableDevice* const outputs[] = {
        &left_pwm_, &right_pwm_, &esc_left_pwm_, &esc_right_pwm_};
    for (WritableDevice* output : outputs) {
        const MotorResult result = Write(*output, &zero, sizeof(zero));
        if (first_failure.ok() && !result.ok()) {
            first_failure = result;
        }
    }

    left_ = {};
    right_ = {};
    return first_failure;
}

MotorResult MotorDevice::Initialize() noexcept {
    if (initialized_ || mode_ != MotorIoMode::kUninitialized) {
        const MotorResult stopped = Stop();
        if (!stopped.ok()) {
            return stopped;
        }
    }

    initialized_ = false;
    left_ = {};
    right_ = {};
    mode_ = MotorIoMode::kOpenWriteClose;

    const WritableDevice* const devices[] = {
        &left_pwm_,      &right_pwm_,     &left_gpio_,
        &right_gpio_,    &esc_left_pwm_,  &esc_right_pwm_,
    };
    for (const WritableDevice* device : devices) {
        const MotorResult probed = Probe(*device);
        if (!probed.ok()) {
            const MotorResult stopped = StopOutputs();
            const MotorResult closed = ClosePersistentDriveOutputs();
            mode_ = MotorIoMode::kUninitialized;
            if (!stopped.ok()) {
                return Failure(MotorStatus::kSafeStopFailed,
                               stopped.system_errno);
            }
            if (!closed.ok()) {
                return Failure(MotorStatus::kSafeStopFailed,
                               closed.system_errno);
            }
            return probed;
        }
    }

    if (io_policy_ == MotorIoPolicy::kPreferPersistent &&
        OpenPersistentDriveOutputs()) {
        mode_ = MotorIoMode::kPersistent;
    }

    const MotorResult stopped = StopOutputs();
    if (!stopped.ok()) {
        const MotorResult closed = ClosePersistentDriveOutputs();
        mode_ = MotorIoMode::kUninitialized;
        return Failure(MotorStatus::kSafeStopFailed,
                       stopped.system_errno != 0
                           ? stopped.system_errno
                           : closed.system_errno);
    }

    initialized_ = true;
    return Ok();
}

MotorResult MotorDevice::ApplyDrive(WritableDevice& pwm,
                                    WritableDevice& gpio,
                                    DriveState& state,
                                    int logical_duty) noexcept {
    const int clamped =
        std::clamp(logical_duty,
                   -port::kDrivePwmDutyCapability,
                   port::kDrivePwmDutyCapability);
    const int hardware_duty = clamped;
    const std::uint16_t duty =
        static_cast<std::uint16_t>(std::abs(hardware_duty));
    const std::uint16_t zero = 0;

    if (duty == 0) {
        const MotorResult stopped = Write(pwm, &zero, sizeof(zero));
        if (stopped.ok()) {
            state.pwm_zero = true;
            state.applied_duty = 0;
        }
        return stopped;
    }

    const int direction = hardware_duty < 0 ? -1 : 1;
    const std::uint8_t gpio_level =
        static_cast<std::uint8_t>(direction < 0 ? '0' : '1');
    const bool first_direction = !state.direction_known;
    const bool reversing = state.direction_known && state.direction_sign != direction;

    if (reversing) {
        const MotorResult cleared = Write(pwm, &zero, sizeof(zero));
        if (!cleared.ok()) {
            return cleared;
        }
        const MotorResult directed =
            Write(gpio, &gpio_level, sizeof(gpio_level));
        if (!directed.ok()) {
            return directed;
        }
        state.direction_sign = direction;
        state.pwm_zero = true;
        state.applied_duty = 0;
        return Ok();
    }
    if (first_direction) {
        const MotorResult directed = Write(gpio, &gpio_level, sizeof(gpio_level));
        if (!directed.ok()) {
            return directed;
        }
    }
    const MotorResult applied = Write(pwm, &duty, sizeof(duty));
    if (!applied.ok()) {
        return applied;
    }

    state.direction_known = true;
    state.direction_sign = direction;
    state.pwm_zero = false;
    state.applied_duty = hardware_duty;
    return Ok();
}

MotorResult MotorDevice::FailSafeAfter(MotorResult failure) noexcept {
    const MotorResult stopped = StopOutputs();
    const MotorResult closed = ClosePersistentDriveOutputs();
    initialized_ = false;
    mode_ = MotorIoMode::kUninitialized;
    if (!stopped.ok()) {
        return Failure(MotorStatus::kSafeStopFailed, stopped.system_errno);
    }
    if (!closed.ok()) {
        return Failure(MotorStatus::kSafeStopFailed, closed.system_errno);
    }
    return failure;
}

MotorResult MotorDevice::Apply(const MotorCommand& command) noexcept {
    if (!initialized_) {
        return Failure(MotorStatus::kNotInitialized, 0);
    }

    // Board wiring is crossed: the logical left wheel owns the physical right
    // PWM/GPIO pair, and the logical right wheel owns the physical left pair.
    MotorResult result = ApplyDrive(
        right_pwm_, right_gpio_, left_, -command.left_drive_pwm);
    if (result.ok()) {
        result = ApplyDrive(
            left_pwm_, left_gpio_, right_, command.right_drive_pwm);
    }

    const std::uint16_t esc_left = static_cast<std::uint16_t>(
        std::clamp(command.left_esc_pwm, 0, kEscDutyLimit));
    const std::uint16_t esc_right = static_cast<std::uint16_t>(
        std::clamp(command.right_esc_pwm, 0, kEscDutyLimit));
    if (result.ok()) {
        result = Write(esc_left_pwm_, &esc_left, sizeof(esc_left));
    }
    if (result.ok()) {
        result = Write(esc_right_pwm_, &esc_right, sizeof(esc_right));
    }
    if (!result.ok()) {
        return FailSafeAfter(result);
    }
    result.applied_command = {
        -left_.applied_duty,
        right_.applied_duty,
        static_cast<int>(esc_left),
        static_cast<int>(esc_right),
    };
    return result;
}

MotorResult MotorDevice::Stop() noexcept {
    initialized_ = false;
    if (mode_ == MotorIoMode::kUninitialized) {
        mode_ = MotorIoMode::kOpenWriteClose;
    }
    const MotorResult stopped = StopOutputs();
    const MotorResult closed = ClosePersistentDriveOutputs();
    mode_ = MotorIoMode::kUninitialized;
    if (!stopped.ok()) {
        return Failure(MotorStatus::kSafeStopFailed, stopped.system_errno);
    }
    if (!closed.ok()) {
        return Failure(MotorStatus::kSafeStopFailed, closed.system_errno);
    }
    return Ok();
}

bool MotorDevice::Initialized() const noexcept {
    return initialized_;
}

MotorIoMode MotorDevice::mode() const noexcept {
    return mode_;
}

const char* MotorIoModeName(MotorIoMode mode) noexcept {
    switch (mode) {
        case MotorIoMode::kUninitialized:
            return "uninitialized";
        case MotorIoMode::kPersistent:
            return "persistent";
        case MotorIoMode::kOpenWriteClose:
            return "open-write-close";
    }
    return "unknown";
}

}  // namespace ls2k::platform::true_ls2k0300
