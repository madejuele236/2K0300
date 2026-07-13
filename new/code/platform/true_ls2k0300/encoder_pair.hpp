#pragma once

#include <cstdint>
#include <string>

#include "platform/linux/linux_io.hpp"

namespace ls2k::platform::true_ls2k0300 {

enum class EncoderIoMode : std::uint8_t {
    kUninitialized = 0,
    kPersistent,
    kOpenReadClose,
};

enum class EncoderIoPolicy : std::uint8_t {
    kOpenReadClose = 0,
    kPreferPersistent,
};

enum class EncoderStatus : std::uint8_t {
    kOk = 0,
    kNotInitialized,
    kOpenFailed,
    kSeekFailed,
    kReadFailed,
    kCloseFailed,
};

struct EncoderInitResult final {
    bool ready{false};
    EncoderIoMode mode{EncoderIoMode::kUninitialized};
    EncoderStatus status{EncoderStatus::kNotInitialized};
};

struct EncoderChannelResult final {
    EncoderStatus status{EncoderStatus::kNotInitialized};
    std::int32_t count{0};

    [[nodiscard]] constexpr bool ok() const noexcept { return status == EncoderStatus::kOk; }
};

struct EncoderPairResult final {
    EncoderChannelResult left{};
    EncoderChannelResult right{};

    [[nodiscard]] constexpr bool valid() const noexcept { return left.ok() && right.ok(); }
};

class EncoderPair final {
public:
    EncoderPair(std::string left_path,
                std::string right_path,
                const linux_io::SyscallApi& syscalls = linux_io::ProductionSyscalls(),
                EncoderIoPolicy io_policy = EncoderIoPolicy::kOpenReadClose);
    ~EncoderPair() noexcept = default;

    EncoderPair(const EncoderPair&) = delete;
    EncoderPair& operator=(const EncoderPair&) = delete;
    EncoderPair(EncoderPair&&) noexcept = default;
    EncoderPair& operator=(EncoderPair&&) noexcept = default;

    [[nodiscard]] EncoderInitResult Initialize() noexcept;
    [[nodiscard]] EncoderPairResult ReadCounts() noexcept;
    void Shutdown() noexcept;

    [[nodiscard]] EncoderIoMode mode() const noexcept { return mode_; }
    [[nodiscard]] bool ready() const noexcept { return ready_; }
    [[nodiscard]] const std::string& left_path() const noexcept { return left_path_; }
    [[nodiscard]] const std::string& right_path() const noexcept { return right_path_; }

private:
    [[nodiscard]] EncoderChannelResult ReadPersistent(linux_io::UniqueFd& fd) noexcept;
    [[nodiscard]] EncoderChannelResult ReadOpenClose(const std::string& path) noexcept;
    [[nodiscard]] bool ProbePersistent(const std::string& path, linux_io::UniqueFd& fd) noexcept;

    std::string left_path_;
    std::string right_path_;
    const linux_io::SyscallApi* syscalls_{nullptr};
    linux_io::UniqueFd left_fd_;
    linux_io::UniqueFd right_fd_;
    EncoderIoPolicy io_policy_{EncoderIoPolicy::kOpenReadClose};
    EncoderIoMode mode_{EncoderIoMode::kUninitialized};
    bool ready_{false};
};

[[nodiscard]] const char* EncoderStatusName(EncoderStatus status) noexcept;
[[nodiscard]] const char* EncoderIoModeName(EncoderIoMode mode) noexcept;

}  // namespace ls2k::platform::true_ls2k0300
