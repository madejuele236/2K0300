#include "platform/true_ls2k0300/encoder_pair.hpp"
#include "platform/true_ls2k0300/vendor_paths.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdio>
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <unistd.h>

namespace {

using ls2k::platform::linux_io::SyscallApi;
using ls2k::platform::true_ls2k0300::EncoderIoModeName;
using ls2k::platform::true_ls2k0300::EncoderPair;
using ls2k::platform::true_ls2k0300::EncoderStatusName;
using ls2k::platform::true_ls2k0300::kLeftEncoderPath;
using ls2k::platform::true_ls2k0300::kRightEncoderPath;

constexpr int kReadCount = 4;
constexpr std::size_t kTrackedFdCount = 8;

struct TrackedFd final {
    int fd{-1};
    const char* path{nullptr};
};

std::array<TrackedFd, kTrackedFdCount> g_tracked_fds{};

void FlushMarker() noexcept {
    static_cast<void>(std::fflush(stderr));
}

const char* PathForFd(int fd) noexcept {
    for (const TrackedFd& tracked : g_tracked_fds) {
        if (tracked.fd == fd) {
            return tracked.path;
        }
    }
    return "<unknown>";
}

void RememberFd(int fd, const char* path) noexcept {
    for (TrackedFd& tracked : g_tracked_fds) {
        if (tracked.fd < 0) {
            tracked = {fd, path};
            return;
        }
    }
}

void ForgetFd(int fd) noexcept {
    for (TrackedFd& tracked : g_tracked_fds) {
        if (tracked.fd == fd) {
            tracked = {};
            return;
        }
    }
}

int ForwardOpen(const char* path, int flags, mode_t mode) {
    std::fprintf(stderr,
                 "encoder_pair_board_test syscall=open phase=before path=%s fd=-1 "
                 "count=0 return=pending errno=0 flags=%d\n",
                 path,
                 flags);
    FlushMarker();

    errno = 0;
    const int result = ::open(path, flags, mode);
    const int system_errno = result < 0 ? errno : 0;
    if (result >= 0) {
        RememberFd(result, path);
    }
    std::fprintf(stderr,
                 "encoder_pair_board_test syscall=open phase=after path=%s fd=%d "
                 "count=0 return=%d errno=%d flags=%d\n",
                 path,
                 result,
                 result,
                 system_errno,
                 flags);
    FlushMarker();
    errno = system_errno;
    return result;
}

ssize_t ForwardRead(int fd, void* buffer, std::size_t count) {
    const char* const path = PathForFd(fd);
    std::fprintf(stderr,
                 "encoder_pair_board_test syscall=read phase=before path=%s fd=%d "
                 "count=%zu return=pending errno=0\n",
                 path,
                 fd,
                 count);
    FlushMarker();

    errno = 0;
    const ssize_t result = ::read(fd, buffer, count);
    const int system_errno = result < 0 ? errno : 0;
    std::fprintf(stderr,
                 "encoder_pair_board_test syscall=read phase=after path=%s fd=%d "
                 "count=%zu return=%zd errno=%d\n",
                 path,
                 fd,
                 count,
                 result,
                 system_errno);
    FlushMarker();
    errno = system_errno;
    return result;
}

ssize_t ForwardWrite(int fd, const void* buffer, std::size_t count) {
    return ::write(fd, buffer, count);
}

off_t ForwardSeek(int fd, off_t offset, int whence) {
    const char* const path = PathForFd(fd);
    std::fprintf(stderr,
                 "encoder_pair_board_test syscall=lseek phase=before path=%s fd=%d "
                 "count=0 return=pending errno=0 offset=%lld whence=%d\n",
                 path,
                 fd,
                 static_cast<long long>(offset),
                 whence);
    FlushMarker();

    errno = 0;
    const off_t result = ::lseek(fd, offset, whence);
    const int system_errno = result < 0 ? errno : 0;
    std::fprintf(stderr,
                 "encoder_pair_board_test syscall=lseek phase=after path=%s fd=%d "
                 "count=0 return=%lld errno=%d offset=%lld whence=%d\n",
                 path,
                 fd,
                 static_cast<long long>(result),
                 system_errno,
                 static_cast<long long>(offset),
                 whence);
    FlushMarker();
    errno = system_errno;
    return result;
}

int ForwardClose(int fd) {
    const char* const path = PathForFd(fd);
    std::fprintf(stderr,
                 "encoder_pair_board_test syscall=close phase=before path=%s fd=%d "
                 "count=0 return=pending errno=0\n",
                 path,
                 fd);
    FlushMarker();

    errno = 0;
    const int result = ::close(fd);
    const int system_errno = result < 0 ? errno : 0;
    std::fprintf(stderr,
                 "encoder_pair_board_test syscall=close phase=after path=%s fd=%d "
                 "count=0 return=%d errno=%d\n",
                 path,
                 fd,
                 result,
                 system_errno);
    FlushMarker();
    if (result == 0) {
        ForgetFd(fd);
    }
    errno = system_errno;
    return result;
}

int ForwardPoll(pollfd* fds, nfds_t count, int timeout_ms) {
    return ::poll(fds, count, timeout_ms);
}

const SyscallApi& ForwardingSyscalls() noexcept {
    static const SyscallApi api(SyscallApi::Functions{
        &ForwardOpen,
        &ForwardRead,
        &ForwardWrite,
        &ForwardSeek,
        &ForwardClose,
        &ForwardPoll,
    });
    return api;
}

}  // namespace

int main() {
    std::fprintf(stderr,
                 "encoder_pair_board_test start policy=open-read-close reads=%d "
                 "left=%s right=%s\n",
                 kReadCount,
                 kLeftEncoderPath,
                 kRightEncoderPath);
    FlushMarker();

    EncoderPair encoder(kLeftEncoderPath, kRightEncoderPath, ForwardingSyscalls());
    const auto initialized = encoder.Initialize();
    std::fprintf(stderr,
                 "encoder_pair_board_test initialize ready=%d mode=%s status=%s\n",
                 initialized.ready ? 1 : 0,
                 EncoderIoModeName(initialized.mode),
                 EncoderStatusName(initialized.status));
    FlushMarker();
    if (!initialized.ready) {
        encoder.Shutdown();
        return 1;
    }

    for (int index = 0; index < kReadCount; ++index) {
        std::fprintf(stderr, "encoder_pair_board_test sample=%d phase=before\n", index);
        FlushMarker();
        const auto sample = encoder.ReadCounts();
        std::fprintf(stderr,
                     "encoder_pair_board_test sample=%d phase=after valid=%d "
                     "left_status=%s left=%d right_status=%s right=%d\n",
                     index,
                     sample.valid() ? 1 : 0,
                     EncoderStatusName(sample.left.status),
                     sample.left.count,
                     EncoderStatusName(sample.right.status),
                     sample.right.count);
        FlushMarker();
        if (!sample.valid()) {
            encoder.Shutdown();
            return 1;
        }
    }

    encoder.Shutdown();
    std::fprintf(stderr, "encoder_pair_board_test complete\n");
    FlushMarker();
    return 0;
}
