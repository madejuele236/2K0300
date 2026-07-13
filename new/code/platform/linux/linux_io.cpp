#include "platform/linux/linux_io.hpp"

#include <cerrno>
#include <fcntl.h>
#include <unistd.h>

namespace ls2k::platform::linux_io {
namespace {

int ProductionOpen(const char* path, int flags, mode_t mode) {
    return ::open(path, flags, mode);
}

ssize_t ProductionRead(int fd, void* buffer, std::size_t size) {
    return ::read(fd, buffer, size);
}

ssize_t ProductionWrite(int fd, const void* buffer, std::size_t size) {
    return ::write(fd, buffer, size);
}

off_t ProductionSeek(int fd, off_t offset, int whence) {
    return ::lseek(fd, offset, whence);
}

int ProductionClose(int fd) {
    return ::close(fd);
}

int ProductionPoll(pollfd* fds, nfds_t count, int timeout_ms) {
    return ::poll(fds, count, timeout_ms);
}

IoResult TransferOnce(
    ssize_t result, int failure_errno) noexcept {
    if (result >= 0) {
        return IoResult{result, IoError::kNone, 0};
    }
    return IoResult{0, ClassifyErrno(failure_errno), failure_errno};
}

}  // namespace

const SyscallApi& ProductionSyscalls() noexcept {
    static const SyscallApi api(SyscallApi::Functions{
        &ProductionOpen,
        &ProductionRead,
        &ProductionWrite,
        &ProductionSeek,
        &ProductionClose,
        &ProductionPoll,
    });
    return api;
}

IoError ClassifyErrno(int system_errno) noexcept {
    switch (system_errno) {
        case 0:
            return IoError::kNone;
        case EINTR:
            return IoError::kInterrupted;
        case EAGAIN:
            return IoError::kWouldBlock;
#if EWOULDBLOCK != EAGAIN
        case EWOULDBLOCK:
            return IoError::kWouldBlock;
#endif
        case EINVAL:
            return IoError::kInvalidArgument;
        case EBADF:
            return IoError::kBadFileDescriptor;
        case EACCES:
        case EPERM:
            return IoError::kPermissionDenied;
        case ENOENT:
            return IoError::kNotFound;
        case ENOSPC:
            return IoError::kNoSpace;
        case EPIPE:
            return IoError::kBrokenPipe;
        case EIO:
            return IoError::kIo;
        default:
            return IoError::kOther;
    }
}

IoResult ReadOnce(
    const SyscallApi& api, int fd, void* buffer, std::size_t size) noexcept {
    const ssize_t result = api.functions().read(fd, buffer, size);
    const int failure_errno = result < 0 ? errno : 0;
    return TransferOnce(result, failure_errno);
}

IoResult WriteOnce(
    const SyscallApi& api, int fd, const void* buffer, std::size_t size) noexcept {
    const ssize_t result = api.functions().write(fd, buffer, size);
    const int failure_errno = result < 0 ? errno : 0;
    return TransferOnce(result, failure_errno);
}

UniqueFd::UniqueFd() noexcept
    : close_(ProductionSyscalls().functions().close) {}

UniqueFd::UniqueFd(int fd) noexcept
    : fd_(fd), close_(ProductionSyscalls().functions().close) {}

UniqueFd::UniqueFd(int fd, const SyscallApi& api) noexcept
    : fd_(fd), close_(api.functions().close) {}

UniqueFd::~UniqueFd() noexcept {
    static_cast<void>(Reset());
}

UniqueFd::UniqueFd(UniqueFd&& other) noexcept
    : fd_(other.Release()), close_(other.close_) {}

UniqueFd& UniqueFd::operator=(UniqueFd&& other) noexcept {
    if (this != &other) {
        static_cast<void>(Reset());
        fd_ = other.Release();
        close_ = other.close_;
    }
    return *this;
}

IoResult UniqueFd::Reset(int fd) noexcept {
    const int old_fd = fd_;
    fd_ = fd;
    if (old_fd < 0) {
        return IoResult{};
    }

    const int result = close_(old_fd);
    const int failure_errno = result < 0 ? errno : 0;
    return TransferOnce(result, failure_errno);
}

int UniqueFd::Release() noexcept {
    const int fd = fd_;
    fd_ = kInvalidFd;
    return fd;
}

}  // namespace ls2k::platform::linux_io
