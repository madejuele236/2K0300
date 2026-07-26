#ifndef LS2K_PLATFORM_TRUE_LS2K0300_MOTOR_DEVICE_HPP
#define LS2K_PLATFORM_TRUE_LS2K0300_MOTOR_DEVICE_HPP

#include <cstddef>
#include <cstdint>

#include "platform/linux/linux_io.hpp"
#include "platform/true_ls2k0300/vendor_paths.hpp"

namespace ls2k::platform::true_ls2k0300 {

enum class MotorStatus : std::uint8_t {
    kOk = 0,
    kNotInitialized,
    kOpenFailed,
    kWriteFailed,
    kCloseFailed,
    kSafeStopFailed,
};

enum class MotorIoMode : std::uint8_t {
    kUninitialized = 0,
    kPersistent,
    kOpenWriteClose,
};

enum class MotorIoPolicy : std::uint8_t {
    kOpenWriteClose = 0,
    kPreferPersistent,
};

struct MotorPaths final {
    const char* left_pwm = kLeftMotorPwmPath;
    const char* right_pwm = kRightMotorPwmPath;
    const char* left_gpio = kLeftMotorGpioPath;
    const char* right_gpio = kRightMotorGpioPath;
    const char* esc_left_pwm = kBrushlessEsc1PwmPath;
    const char* esc_right_pwm = kBrushlessEsc2PwmPath;
};

struct MotorCommand final {
    int left_drive_pwm = 0;
    int right_drive_pwm = 0;
    int left_esc_pwm = 0;
    int right_esc_pwm = 0;
};

struct MotorResult final {
    MotorStatus status{MotorStatus::kNotInitialized};
    int system_errno{0};
    MotorCommand applied_command{};
    [[nodiscard]] constexpr bool ok() const noexcept { return status == MotorStatus::kOk; }
};

class MotorDevice final {
public:
    explicit MotorDevice(
        MotorPaths paths = {},
        const linux_io::SyscallApi& syscalls = linux_io::ProductionSyscalls(),
        MotorIoPolicy io_policy = MotorIoPolicy::kOpenWriteClose) noexcept;
    ~MotorDevice() noexcept;

    MotorDevice(const MotorDevice&) = delete;
    MotorDevice& operator=(const MotorDevice&) = delete;
    MotorDevice(MotorDevice&&) = delete;
    MotorDevice& operator=(MotorDevice&&) = delete;

    [[nodiscard]] MotorResult Initialize() noexcept;
    [[nodiscard]] MotorResult Apply(const MotorCommand& command) noexcept;
    [[nodiscard]] MotorResult Stop() noexcept;
    [[nodiscard]] bool Initialized() const noexcept;
    [[nodiscard]] MotorIoMode mode() const noexcept;

private:
    struct WritableDevice final {
        WritableDevice(const char* device_path,
                       const linux_io::SyscallApi& syscalls) noexcept
            : path(device_path), fd(-1, syscalls) {}

        const char* path;
        linux_io::UniqueFd fd;
    };

    struct DriveState final {
        bool direction_known{false};
        int direction_sign{1};
        bool pwm_zero{true};
        int applied_duty{0};
    };

    [[nodiscard]] MotorResult Probe(const WritableDevice& device) noexcept;
    [[nodiscard]] MotorResult Write(WritableDevice& device,
                                    const void* payload,
                                    std::size_t size) noexcept;
    [[nodiscard]] MotorResult WriteOpenClose(const char* path,
                                             const void* payload,
                                             std::size_t size) noexcept;
    [[nodiscard]] bool OpenPersistentDriveOutputs() noexcept;
    [[nodiscard]] MotorResult ClosePersistentDriveOutputs() noexcept;
    [[nodiscard]] MotorResult StopOutputs() noexcept;
    [[nodiscard]] MotorResult ApplyDrive(WritableDevice& pwm,
                                         WritableDevice& gpio,
                                         DriveState& state,
                                         int logical_duty) noexcept;
    [[nodiscard]] MotorResult FailSafeAfter(MotorResult failure) noexcept;

    const linux_io::SyscallApi* syscalls_;
    WritableDevice left_pwm_;
    WritableDevice right_pwm_;
    WritableDevice left_gpio_;
    WritableDevice right_gpio_;
    WritableDevice esc_left_pwm_;
    WritableDevice esc_right_pwm_;
    DriveState left_{};
    DriveState right_{};
    MotorIoPolicy io_policy_{MotorIoPolicy::kOpenWriteClose};
    MotorIoMode mode_{MotorIoMode::kUninitialized};
    bool initialized_{false};
};

[[nodiscard]] const char* MotorIoModeName(MotorIoMode mode) noexcept;

}  // namespace ls2k::platform::true_ls2k0300

#endif
