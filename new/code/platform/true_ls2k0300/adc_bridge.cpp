#include "platform/true_ls2k0300/adc_device.hpp"

#include <cerrno>
#include <charconv>
#include <cstddef>
#include <fcntl.h>
#include <string_view>
#include <utility>

#include "platform/linux/linux_io.hpp"

namespace ls2k::platform::true_ls2k0300 {

AdcDevice::AdcDevice(std::string path) : path_(std::move(path)) {}

AdcSampleResult AdcDevice::ReadRaw() const noexcept {
    const linux_io::SyscallApi& syscalls = linux_io::ProductionSyscalls();
    const int fd = syscalls.functions().open(path_.c_str(), O_RDONLY | O_CLOEXEC, 0);
    if (fd < 0) {
        return AdcSampleResult{-1, AdcReadError::kOpenFailed, errno};
    }
    linux_io::UniqueFd input(fd, syscalls);

    char buffer[64]{};
    linux_io::IoResult read_result{};
    do {
        read_result = linux_io::ReadOnce(syscalls, input.get(), buffer, sizeof(buffer));
    } while (read_result.error == linux_io::IoError::kInterrupted);
    if (!read_result.ok()) {
        return AdcSampleResult{-1, AdcReadError::kReadFailed, read_result.system_errno};
    }
    if (read_result.bytes_transferred == 0) {
        return AdcSampleResult{-1, AdcReadError::kEmpty, 0};
    }

    std::string_view text(buffer, static_cast<std::size_t>(read_result.bytes_transferred));
    while (!text.empty() && (text.front() == ' ' || text.front() == '\t' || text.front() == '\r' || text.front() == '\n')) {
        text.remove_prefix(1);
    }
    while (!text.empty() && (text.back() == ' ' || text.back() == '\t' || text.back() == '\r' || text.back() == '\n')) {
        text.remove_suffix(1);
    }
    if (text.empty()) {
        return AdcSampleResult{-1, AdcReadError::kEmpty, 0};
    }

    int raw_value = 0;
    const std::from_chars_result parsed = std::from_chars(text.data(), text.data() + text.size(), raw_value);
    if (parsed.ec == std::errc::result_out_of_range) {
        return AdcSampleResult{-1, AdcReadError::kOutOfRange, 0};
    }
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
        return AdcSampleResult{-1, AdcReadError::kParseFailed, 0};
    }
    return AdcSampleResult{raw_value, AdcReadError::kNone, 0};
}

}  // namespace ls2k::platform::true_ls2k0300
