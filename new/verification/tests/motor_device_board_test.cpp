#include "platform/true_ls2k0300/adc_device.hpp"
#include "platform/true_ls2k0300/encoder_mapping.hpp"
#include "platform/true_ls2k0300/encoder_pair.hpp"
#include "platform/true_ls2k0300/motor_device.hpp"
#include "platform/true_ls2k0300/vendor_paths.hpp"

#include <chrono>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <thread>

namespace {

using ls2k::platform::true_ls2k0300::MotorCommand;
using ls2k::platform::true_ls2k0300::MotorDevice;
using ls2k::platform::true_ls2k0300::MotorResult;
using ls2k::platform::true_ls2k0300::MotorStatus;
using ls2k::platform::true_ls2k0300::EncoderPair;
using ls2k::platform::true_ls2k0300::EncoderPairResult;
using ls2k::platform::true_ls2k0300::AdcDevice;

constexpr int kLowDrivePwm = 3000;
constexpr int kMotionSampleCount = 250;
constexpr auto kMotionSamplePeriod = std::chrono::milliseconds(20);
constexpr auto kZeroDwell = std::chrono::milliseconds(150);
constexpr const char* kConfirmation = "--confirm-low-duty-motion";
constexpr const char* kSingleForwardConfirmation = "--confirm-dual-forward-5s";
constexpr const char* kPersistentForwardConfirmation =
    "--confirm-persistent-forward-5s";
constexpr const char* kFailureSafeStopConfirmation =
    "--confirm-failure-safe-stop";
constexpr const char* kMappingConfirmation = "--confirm-channel-map";

const ls2k::platform::linux_io::SyscallApi* g_forward_syscalls = nullptr;
int g_fail_write_ordinal = 0;
int g_armed_write_count = 0;

int ForwardOpen(const char* path, int flags, mode_t mode) {
    return g_forward_syscalls->functions().open(path, flags, mode);
}

ssize_t ForwardRead(int fd, void* data, std::size_t size) {
    return g_forward_syscalls->functions().read(fd, data, size);
}

ssize_t InjectingWrite(int fd, const void* data, std::size_t size) {
    if (g_fail_write_ordinal > 0) {
        ++g_armed_write_count;
        if (g_armed_write_count == g_fail_write_ordinal) {
            g_fail_write_ordinal = 0;
            errno = EIO;
            return -1;
        }
    }
    return g_forward_syscalls->functions().write(fd, data, size);
}

off_t ForwardSeek(int fd, off_t offset, int whence) {
    return g_forward_syscalls->functions().lseek(fd, offset, whence);
}

int ForwardClose(int fd) {
    return g_forward_syscalls->functions().close(fd);
}

int ForwardPoll(pollfd* fds, nfds_t count, int timeout_ms) {
    return g_forward_syscalls->functions().poll(fds, count, timeout_ms);
}

const char* StatusName(MotorStatus status) noexcept {
    switch (status) {
        case MotorStatus::kOk:
            return "ok";
        case MotorStatus::kNotInitialized:
            return "not_initialized";
        case MotorStatus::kOpenFailed:
            return "open_failed";
        case MotorStatus::kWriteFailed:
            return "write_failed";
        case MotorStatus::kCloseFailed:
            return "close_failed";
        case MotorStatus::kSafeStopFailed:
            return "safe_stop_failed";
    }
    return "unknown";
}

void PrintResult(const char* step, const MotorResult& result) {
    std::cout << "motor_board_test step=" << step
              << " status=" << StatusName(result.status)
              << " errno=" << result.system_errno << '\n';
}

int FailAndStop(MotorDevice& motor, const char* step, const MotorResult& failure) {
    std::cerr << "motor_board_test FAILURE step=" << step
              << " status=" << StatusName(failure.status)
              << " errno=" << failure.system_errno << '\n';
    const MotorResult stopped = motor.Stop();
    std::cerr << "motor_board_test failure_stop status=" << StatusName(stopped.status)
              << " errno=" << stopped.system_errno << '\n';
    if (!stopped.ok()) {
        std::cerr << "motor_board_test CRITICAL PWM=0 could not be confirmed by MotorDevice\n";
    }
    return 1;
}

int StopAfterEncoderFailure(MotorDevice& motor, const char* step) {
    const MotorResult stopped = motor.Stop();
    std::cerr << "motor_board_test encoder_failure_stop step=" << step
              << " status=" << StatusName(stopped.status)
              << " errno=" << stopped.system_errno << '\n';
    if (!stopped.ok()) {
        std::cerr << "motor_board_test CRITICAL PWM=0 could not be confirmed by MotorDevice\n";
    }
    return 1;
}

void StopAfterMotionValidationFailure(MotorDevice& motor, const char* step) {
    const MotorResult stopped = motor.Stop();
    std::cerr << "motor_board_test validation_failure_stop step=" << step
              << " status=" << StatusName(stopped.status)
              << " errno=" << stopped.system_errno << '\n';
    if (!stopped.ok()) {
        std::cerr << "motor_board_test CRITICAL PWM=0 could not be confirmed by MotorDevice\n";
    }
}

bool ApplyStep(MotorDevice& motor,
               const char* step,
               const MotorCommand& command,
               std::chrono::milliseconds dwell) {
    const MotorResult result = motor.Apply(command);
    PrintResult(step, result);
    if (!result.ok()) {
        static_cast<void>(FailAndStop(motor, step, result));
        return false;
    }
    std::this_thread::sleep_for(dwell);
    return true;
}

bool RunMotionWindow(MotorDevice& motor,
                     EncoderPair& encoder,
                     AdcDevice& battery_adc,
                     const char* step,
                     const MotorCommand& command,
                     int expected_logical_sign) {
    std::int64_t apply_min_us = 0;
    std::int64_t apply_max_us = 0;
    std::int64_t apply_total_us = 0;
    int apply_samples = 0;
    int adc_min = 0;
    int adc_max = 0;
    int adc_samples = 0;
    int adc_errors = 0;

    const auto window_started = std::chrono::steady_clock::now();
    for (int sample_index = 0; sample_index < kMotionSampleCount; ++sample_index) {
        const auto apply_started = std::chrono::steady_clock::now();
        const MotorResult applied = motor.Apply(command);
        const auto apply_finished = std::chrono::steady_clock::now();
        const auto apply_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                  apply_finished - apply_started)
                                  .count();
        if (!applied.ok()) {
            PrintResult(step, applied);
            static_cast<void>(FailAndStop(motor, step, applied));
            return false;
        }
        if (apply_samples == 0 || apply_us < apply_min_us) {
            apply_min_us = apply_us;
        }
        if (apply_samples == 0 || apply_us > apply_max_us) {
            apply_max_us = apply_us;
        }
        apply_total_us += apply_us;
        ++apply_samples;

        const auto battery = battery_adc.ReadRaw();
        if (battery.ok()) {
            if (adc_samples == 0 || battery.raw_value < adc_min) {
                adc_min = battery.raw_value;
            }
            if (adc_samples == 0 || battery.raw_value > adc_max) {
                adc_max = battery.raw_value;
            }
            ++adc_samples;
        } else {
            ++adc_errors;
        }
        std::this_thread::sleep_until(
            window_started + (sample_index + 1) * kMotionSamplePeriod);
    }

    const auto read_started = std::chrono::steady_clock::now();
    const EncoderPairResult sample = encoder.ReadCounts();
    const auto read_finished = std::chrono::steady_clock::now();
    const auto encoder_read_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                     read_finished - read_started)
                                     .count();
    if (!sample.valid()) {
        std::cerr << "motor_board_test FAILURE step=" << step
                  << " left_status="
                  << ls2k::platform::true_ls2k0300::EncoderStatusName(sample.left.status)
                  << " right_status="
                  << ls2k::platform::true_ls2k0300::EncoderStatusName(sample.right.status)
                  << '\n';
        static_cast<void>(StopAfterEncoderFailure(motor, step));
        return false;
    }

    const std::int64_t raw_left_count = sample.left.count;
    const std::int64_t raw_right_count = sample.right.count;
    const auto logical =
        ls2k::platform::true_ls2k0300::NormalizeEncoderCounts(
            sample.left.count, sample.right.count);
    const std::int64_t logical_left_count = logical.left;
    const std::int64_t logical_right_count = logical.right;
    std::cout << "motor_board_test motion step=" << step
              << " samples=" << kMotionSampleCount << '\n'
              << "motor_board_test counts step=" << step
              << " raw_left_count=" << raw_left_count
              << " raw_right_count=" << raw_right_count << '\n'
              << "motor_board_test logical step=" << step
              << " left_count=" << logical_left_count
              << " right_count=" << logical_right_count << '\n'
              << "motor_board_test timing step=" << step
              << " motor_apply_samples=" << apply_samples
              << " motor_apply_min_us=" << apply_min_us
              << " motor_apply_max_us=" << apply_max_us
              << " motor_apply_mean_us="
              << (apply_samples == 0 ? 0 : apply_total_us / apply_samples)
              << " encoder_read_us=" << encoder_read_us << '\n'
              << "motor_board_test battery step=" << step
              << " adc_samples=" << adc_samples
              << " adc_errors=" << adc_errors
              << " adc_min=" << adc_min
              << " adc_max=" << adc_max << '\n';

    if (expected_logical_sign == 0) {
        return true;
    }

    const bool sign_matches = expected_logical_sign > 0
                                  ? logical_left_count > 0 && logical_right_count > 0
                                  : logical_left_count < 0 && logical_right_count < 0;
    if (!sign_matches) {
        std::cerr << "motor_board_test FAILURE step=" << step
                  << " encoder direction does not match logical left=raw_left, "
                     "right=-raw_right contract\n";
        StopAfterMotionValidationFailure(motor, step);
    }
    return sign_matches;
}

}  // namespace

int main(int argc, char** argv) {
    const bool run_acceptance = argc == 2 && std::strcmp(argv[1], kConfirmation) == 0;
    const bool run_single_forward =
        argc == 2 && std::strcmp(argv[1], kSingleForwardConfirmation) == 0;
    const bool run_persistent_forward =
        argc == 2 && std::strcmp(argv[1], kPersistentForwardConfirmation) == 0;
    const bool run_failure_safe_stop =
        argc == 2 && std::strcmp(argv[1], kFailureSafeStopConfirmation) == 0;
    const bool run_channel_map = argc == 2 && std::strcmp(argv[1], kMappingConfirmation) == 0;
    if (!run_acceptance && !run_single_forward &&
        !run_persistent_forward && !run_failure_safe_stop &&
        !run_channel_map) {
        std::cerr
            << "motor_board_test NOT RUN: this helper moves both drive motors.\n"
            << "Lift and secure the chassis, keep clear of wheels, and provide "
            << kConfirmation << " only after explicit approval.\n"
            << "Sequence: PWM=0; forward PWM=" << kLowDrivePwm << " for "
            << (kMotionSampleCount * kMotionSamplePeriod.count())
            << " ms; PWM=0 for " << kZeroDwell.count()
            << " ms; reverse PWM=" << kLowDrivePwm << " for "
            << (kMotionSampleCount * kMotionSamplePeriod.count())
            << " ms; Stop. ESC PWM remains 0.\n"
            << "For a four-window left+/left-/right+/right- mapping diagnostic, use "
            << kMappingConfirmation << ".\n"
            << "For exactly one dual-wheel forward PWM=" << kLowDrivePwm
            << " window lasting "
            << (kMotionSampleCount * kMotionSamplePeriod.count())
            << " ms, use " << kSingleForwardConfirmation << ".\n"
            << "To run that same window with persistent drive PWM/GPIO fds, use "
            << kPersistentForwardConfirmation << ".\n"
            << "To inject one EIO on the second drive write and verify the "
               "fail-safe zero path, use "
            << kFailureSafeStopConfirmation << ".\n";
        return 2;
    }

    std::cerr << "motor_board_test WARNING: real wheels will move at low duty; "
                 "ESC PWM remains 0\n";

    EncoderPair encoder(ls2k::platform::true_ls2k0300::kLeftEncoderPath,
                        ls2k::platform::true_ls2k0300::kRightEncoderPath);
    AdcDevice battery_adc(ls2k::platform::true_ls2k0300::kBatteryAdcPath);
    const auto encoder_initialized = encoder.Initialize();
    if (!encoder_initialized.ready) {
        std::cerr << "motor_board_test FAILURE step=encoder_initialize status="
                  << ls2k::platform::true_ls2k0300::EncoderStatusName(
                         encoder_initialized.status)
                  << '\n';
        return 1;
    }
    std::cout << "motor_board_test encoder_io_mode="
              << ls2k::platform::true_ls2k0300::EncoderIoModeName(
                     encoder_initialized.mode)
              << '\n';

    const auto& production_syscalls =
        ls2k::platform::linux_io::ProductionSyscalls();
    g_forward_syscalls = &production_syscalls;
    const ls2k::platform::linux_io::SyscallApi injecting_syscalls({
        &ForwardOpen,
        &ForwardRead,
        &InjectingWrite,
        &ForwardSeek,
        &ForwardClose,
        &ForwardPoll,
    });
    MotorDevice motor(
        {},
        run_failure_safe_stop ? injecting_syscalls : production_syscalls,
        run_persistent_forward
            ? ls2k::platform::true_ls2k0300::MotorIoPolicy::kPreferPersistent
            : ls2k::platform::true_ls2k0300::MotorIoPolicy::kOpenWriteClose);
    const MotorResult initialized = motor.Initialize();
    PrintResult("initialize", initialized);
    if (!initialized.ok()) {
        return FailAndStop(motor, "initialize", initialized);
    }
    std::cout << "motor_board_test io_mode="
              << ls2k::platform::true_ls2k0300::MotorIoModeName(motor.mode())
              << '\n';

    if (!ApplyStep(motor, "safe_zero", {0, 0, 0, 0}, kZeroDwell)) {
        return 1;
    }
    const EncoderPairResult baseline = encoder.ReadCounts();
    if (!baseline.valid()) {
        std::cerr << "motor_board_test FAILURE step=encoder_baseline left_status="
                  << ls2k::platform::true_ls2k0300::EncoderStatusName(baseline.left.status)
                  << " right_status="
                  << ls2k::platform::true_ls2k0300::EncoderStatusName(baseline.right.status)
                  << '\n';
        return StopAfterEncoderFailure(motor, "encoder_baseline");
    }
    if (run_failure_safe_stop) {
        g_armed_write_count = 0;
        g_fail_write_ordinal = 2;
        const MotorResult injected = motor.Apply({kLowDrivePwm, kLowDrivePwm, 0, 0});
        PrintResult("injected_second_drive_write", injected);
        g_fail_write_ordinal = 0;
        const bool expected_failure =
            injected.status == MotorStatus::kWriteFailed && !motor.Initialized();
        const MotorResult stopped = motor.Stop();
        PrintResult("injected_failure_final_stop", stopped);
        if (!expected_failure || !stopped.ok()) {
            std::cerr << "motor_board_test FAILURE injected write did not "
                         "invalidate the owner and stop all outputs\n";
            return 1;
        }
        std::cout << "motor_board_test FAILURE_SAFE_STOP_COMPLETE final_pwm=0\n";
        return 0;
    }
    if (run_single_forward || run_persistent_forward) {
        if (!RunMotionWindow(motor,
                             encoder,
                             battery_adc,
                             run_persistent_forward
                                 ? "persistent_dual_forward_3000_5s"
                                 : "dual_forward_3000_5s",
                             {kLowDrivePwm, kLowDrivePwm, 0, 0},
                             1)) {
            return 1;
        }
        const MotorResult stopped = motor.Stop();
        PrintResult("single_forward_final_stop", stopped);
        if (!stopped.ok()) {
            return FailAndStop(motor, "single_forward_final_stop", stopped);
        }
        std::cout << "motor_board_test SINGLE_FORWARD_COMPLETE final_pwm=0\n";
        return 0;
    }
    if (run_channel_map) {
        struct MappingWindow final {
            const char* name;
            MotorCommand command;
        };
        const MappingWindow windows[] = {
            {"map_left_positive", {kLowDrivePwm, 0, 0, 0}},
            {"map_left_negative", {-kLowDrivePwm, 0, 0, 0}},
            {"map_right_positive", {0, kLowDrivePwm, 0, 0}},
            {"map_right_negative", {0, -kLowDrivePwm, 0, 0}},
        };
        for (const MappingWindow& window : windows) {
            if (!RunMotionWindow(motor,
                                 encoder,
                                 battery_adc,
                                 window.name,
                                 window.command,
                                 0)) {
                return 1;
            }
            if (!ApplyStep(motor, "map_zero", {0, 0, 0, 0}, kZeroDwell)) {
                return 1;
            }
            const EncoderPairResult cleared = encoder.ReadCounts();
            if (!cleared.valid()) {
                return StopAfterEncoderFailure(motor, "map_zero_encoder");
            }
        }
        const MotorResult stopped = motor.Stop();
        PrintResult("mapping_final_stop", stopped);
        if (!stopped.ok()) {
            return FailAndStop(motor, "mapping_final_stop", stopped);
        }
        std::cout << "motor_board_test MAP_COMPLETE final_pwm=0\n";
        return 0;
    }
    if (!RunMotionWindow(motor,
                         encoder,
                         battery_adc,
                         "forward_low_duty",
                         {kLowDrivePwm, kLowDrivePwm, 0, 0},
                         1)) {
        return 1;
    }
    if (!ApplyStep(motor, "zero_before_reverse", {0, 0, 0, 0}, kZeroDwell)) {
        return 1;
    }
    const EncoderPairResult zero_sample = encoder.ReadCounts();
    if (!zero_sample.valid()) {
        return StopAfterEncoderFailure(motor, "encoder_zero_before_reverse");
    }
    if (!RunMotionWindow(motor,
                         encoder,
                         battery_adc,
                         "reverse_low_duty",
                         {-kLowDrivePwm, -kLowDrivePwm, 0, 0},
                         -1)) {
        return 1;
    }

    const MotorResult stopped = motor.Stop();
    PrintResult("final_stop", stopped);
    if (!stopped.ok()) {
        return FailAndStop(motor, "final_stop", stopped);
    }
    std::cout << "motor_board_test PASS final_pwm=0\n";
    return 0;
}
