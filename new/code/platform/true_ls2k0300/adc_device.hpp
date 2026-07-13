#pragma once

#include <cstdint>
#include <string>

namespace ls2k::platform::true_ls2k0300 {

enum class AdcReadError : std::uint8_t {
    kNone = 0,
    kOpenFailed,
    kReadFailed,
    kEmpty,
    kParseFailed,
    kOutOfRange,
};

struct AdcSampleResult final {
    int raw_value{-1};
    AdcReadError error{AdcReadError::kNone};
    int system_errno{0};

    [[nodiscard]] constexpr bool ok() const noexcept {
        return error == AdcReadError::kNone;
    }
};

class AdcDevice final {
public:
    explicit AdcDevice(std::string path);

    AdcDevice(const AdcDevice&) = delete;
    AdcDevice& operator=(const AdcDevice&) = delete;
    AdcDevice(AdcDevice&&) noexcept = default;
    AdcDevice& operator=(AdcDevice&&) noexcept = default;

    [[nodiscard]] AdcSampleResult ReadRaw() const noexcept;
    [[nodiscard]] const std::string& path() const noexcept { return path_; }

private:
    std::string path_;
};

}  // namespace ls2k::platform::true_ls2k0300
