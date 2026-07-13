#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "platform/linux/linux_io.hpp"

namespace ls2k::platform::true_ls2k0300 {

inline constexpr std::size_t kImuChannelCount = 9;
inline constexpr std::size_t kMaxImuCandidates = 32;

enum class ImuType : std::uint8_t {
    kNotFound = 0,
    kImu660Ra = 0x10,
    kImu660Rb = 0x11,
    kImu963Ra = 0x12,
};

enum class ImuIoMode : std::uint8_t {
    kUninitialized = 0,
    kPersistent,
    kOpenReadClose,
};

enum class ImuStatus : std::uint8_t {
    kOk = 0,
    kNotInitialized,
    kDiscoveryFailed,
    kUnsupportedModel,
    kOpenFailed,
    kSeekFailed,
    kReadFailed,
    kParseFailed,
    kCloseFailed,
};

struct ImuFilesystemApi final {
    using EnumerateNamePathsFn = std::size_t (*)(void*, std::array<std::string, kMaxImuCandidates>&);

    void* context{nullptr};
    EnumerateNamePathsFn enumerate_name_paths{nullptr};
};

[[nodiscard]] const ImuFilesystemApi& ProductionImuFilesystem() noexcept;

struct ImuInitResult final {
    bool ready{false};
    ImuType type{ImuType::kNotFound};
    ImuIoMode mode{ImuIoMode::kUninitialized};
    ImuStatus status{ImuStatus::kNotInitialized};
    std::uint8_t failed_channel{0xff};
};

struct ImuRawSample final {
    bool valid{false};
    ImuType type{ImuType::kNotFound};
    ImuStatus status{ImuStatus::kNotInitialized};
    std::uint8_t failed_channel{0xff};
    std::int16_t acc_x{0};
    std::int16_t acc_y{0};
    std::int16_t acc_z{0};
    std::int16_t gyro_x{0};
    std::int16_t gyro_y{0};
    std::int16_t gyro_z{0};
    std::int16_t mag_x{0};
    std::int16_t mag_y{0};
    std::int16_t mag_z{0};
};

class ImuDevice final {
public:
    ImuDevice(const linux_io::SyscallApi& syscalls = linux_io::ProductionSyscalls(),
              const ImuFilesystemApi& filesystem = ProductionImuFilesystem());
    ~ImuDevice() noexcept = default;

    ImuDevice(const ImuDevice&) = delete;
    ImuDevice& operator=(const ImuDevice&) = delete;
    ImuDevice(ImuDevice&&) noexcept = default;
    ImuDevice& operator=(ImuDevice&&) noexcept = default;

    [[nodiscard]] ImuInitResult Initialize();
    [[nodiscard]] ImuRawSample ReadRawSample() noexcept;
    void Shutdown() noexcept;

    [[nodiscard]] bool ready() const noexcept { return ready_; }
    [[nodiscard]] ImuType type() const noexcept { return type_; }
    [[nodiscard]] ImuIoMode mode() const noexcept { return mode_; }
    [[nodiscard]] const std::string& source() const noexcept { return source_; }
    [[nodiscard]] const std::string& device_dir() const noexcept { return device_dir_; }

private:
    [[nodiscard]] ImuStatus ReadToken(const std::string& path, char* buffer, std::size_t capacity,
                                      std::size_t& length) noexcept;
    [[nodiscard]] ImuStatus ReadChannelPersistent(std::size_t index, std::int16_t& value) noexcept;
    [[nodiscard]] ImuStatus ReadChannelOpenClose(std::size_t index, std::int16_t& value) noexcept;
    [[nodiscard]] bool OpenPersistentChannels(ImuInitResult& result) noexcept;
    [[nodiscard]] bool ProbeOpenCloseChannels(ImuInitResult& result) noexcept;
    void ResetState() noexcept;

    const linux_io::SyscallApi* syscalls_{nullptr};
    const ImuFilesystemApi* filesystem_{nullptr};
    std::array<std::string, kImuChannelCount> paths_{};
    std::array<linux_io::UniqueFd, kImuChannelCount> fds_{};
    std::string source_;
    std::string device_dir_;
    ImuType type_{ImuType::kNotFound};
    ImuIoMode mode_{ImuIoMode::kUninitialized};
    std::size_t required_channels_{0};
    bool ready_{false};
};

[[nodiscard]] const char* ImuTypeName(ImuType type) noexcept;
[[nodiscard]] const char* ImuStatusName(ImuStatus status) noexcept;
[[nodiscard]] const char* ImuIoModeName(ImuIoMode mode) noexcept;

}  // namespace ls2k::platform::true_ls2k0300
