#pragma once

#include <cstddef>
#include <cstdint>
#include <poll.h>
#include <sys/types.h>

namespace ls2k::platform::linux_io {

enum class IoError : std::uint8_t {
    kNone = 0,
    kInterrupted,
    kWouldBlock,
    kInvalidArgument,
    kBadFileDescriptor,
    kPermissionDenied,
    kNotFound,
    kNoSpace,
    kBrokenPipe,
    kIo,
    kOther,
};

struct IoResult final {
    ssize_t bytes_transferred{0};
    IoError error{IoError::kNone};
    int system_errno{0};

    [[nodiscard]] constexpr bool ok() const noexcept {
        return error == IoError::kNone;
    }
};

class SyscallApi final {
public:
    using OpenFn = int (*)(const char*, int, mode_t);
    using ReadFn = ssize_t (*)(int, void*, std::size_t);
    using WriteFn = ssize_t (*)(int, const void*, std::size_t);
    using SeekFn = off_t (*)(int, off_t, int);
    using CloseFn = int (*)(int);
    using PollFn = int (*)(pollfd*, nfds_t, int);

    struct Functions final {
        OpenFn open;
        ReadFn read;
        WriteFn write;
        SeekFn lseek;
        CloseFn close;
        PollFn poll;
    };

    explicit constexpr SyscallApi(Functions functions) noexcept
        : functions_(functions) {}

    SyscallApi(const SyscallApi&) = delete;
    SyscallApi& operator=(const SyscallApi&) = delete;
    SyscallApi(SyscallApi&&) = delete;
    SyscallApi& operator=(SyscallApi&&) = delete;

    [[nodiscard]] constexpr const Functions& functions() const noexcept {
        return functions_;
    }

private:
    const Functions functions_;
};

// Process-lifetime syscall table backed by the Linux C library entry points.
[[nodiscard]] const SyscallApi& ProductionSyscalls() noexcept;

[[nodiscard]] IoError ClassifyErrno(int system_errno) noexcept;
[[nodiscard]] IoResult ReadOnce(
    const SyscallApi& api, int fd, void* buffer, std::size_t size) noexcept;
[[nodiscard]] IoResult WriteOnce(
    const SyscallApi& api, int fd, const void* buffer, std::size_t size) noexcept;

class UniqueFd final {
public:
    UniqueFd() noexcept;
    explicit UniqueFd(int fd) noexcept;
    UniqueFd(int fd, const SyscallApi& api) noexcept;
    ~UniqueFd() noexcept;

    UniqueFd(const UniqueFd&) = delete;
    UniqueFd& operator=(const UniqueFd&) = delete;

    UniqueFd(UniqueFd&& other) noexcept;
    UniqueFd& operator=(UniqueFd&& other) noexcept;

    [[nodiscard]] int get() const noexcept { return fd_; }
    [[nodiscard]] explicit operator bool() const noexcept { return fd_ >= 0; }

    // Closes the currently owned fd exactly once, then takes ownership of fd.
    // A close failure is reported but does not retain ownership of the old fd.
    [[nodiscard]] IoResult Reset(int fd = -1) noexcept;

    // Relinquishes ownership without closing and returns the former fd.
    [[nodiscard]] int Release() noexcept;

private:
    static constexpr int kInvalidFd = -1;

    int fd_{kInvalidFd};
    SyscallApi::CloseFn close_{nullptr};
};

}  // namespace ls2k::platform::linux_io
