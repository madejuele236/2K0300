#include <cerrno>
#include <cstdlib>
#include <type_traits>
#include <unistd.h>

#include "platform/linux/linux_io.hpp"

namespace {

using ls2k::platform::linux_io::IoError;
using ls2k::platform::linux_io::IoResult;
using ls2k::platform::linux_io::ProductionSyscalls;
using ls2k::platform::linux_io::ReadOnce;
using ls2k::platform::linux_io::SyscallApi;
using ls2k::platform::linux_io::UniqueFd;
using ls2k::platform::linux_io::WriteOnce;

struct FakeState {
    int close_calls{0};
    int last_closed_fd{-1};
    int close_result{0};
    int close_errno{0};
    int read_calls{0};
    ssize_t read_result{0};
    int read_errno{0};
    int write_calls{0};
    ssize_t write_result{0};
    int write_errno{0};
};

FakeState g_fake;

int FakeOpen(const char*, int, mode_t) { return -1; }

ssize_t FakeRead(int, void*, std::size_t) {
    ++g_fake.read_calls;
    errno = g_fake.read_errno;
    return g_fake.read_result;
}

ssize_t FakeWrite(int, const void*, std::size_t) {
    ++g_fake.write_calls;
    errno = g_fake.write_errno;
    return g_fake.write_result;
}

off_t FakeSeek(int, off_t, int) { return -1; }

int FakeClose(int fd) {
    ++g_fake.close_calls;
    g_fake.last_closed_fd = fd;
    errno = g_fake.close_errno;
    return g_fake.close_result;
}

int FakePoll(pollfd*, nfds_t, int) { return -1; }

const SyscallApi& FakeApi() {
    static const SyscallApi api(SyscallApi::Functions{
        &FakeOpen, &FakeRead, &FakeWrite, &FakeSeek, &FakeClose, &FakePoll});
    return api;
}

void ResetFake() {
    g_fake = FakeState{};
}

void Expect(bool condition) {
    if (!condition) {
        std::abort();
    }
}

void TestMoveOnlyAndLifetime() {
    static_assert(std::is_trivially_copyable_v<IoResult>);
    static_assert(!std::is_copy_constructible_v<SyscallApi>);
    static_assert(!std::is_copy_assignable_v<SyscallApi>);
    static_assert(!std::is_copy_constructible_v<UniqueFd>);
    static_assert(!std::is_copy_assignable_v<UniqueFd>);
    static_assert(std::is_nothrow_move_constructible_v<UniqueFd>);
    static_assert(std::is_nothrow_move_assignable_v<UniqueFd>);

    ResetFake();
    UniqueFd invalid;
    Expect(!invalid);
    Expect(invalid.get() == -1);
    {
        UniqueFd source(11, FakeApi());
        UniqueFd moved(static_cast<UniqueFd&&>(source));
        Expect(!source);
        Expect(moved.get() == 11);

        UniqueFd destination(22, FakeApi());
        destination = static_cast<UniqueFd&&>(moved);
        Expect(g_fake.close_calls == 1);
        Expect(g_fake.last_closed_fd == 22);
        Expect(!moved);
        Expect(destination.get() == 11);
    }
    Expect(g_fake.close_calls == 2);
    Expect(g_fake.last_closed_fd == 11);
}

void TestResetReleaseAndCloseError() {
    ResetFake();
    UniqueFd fd(31, FakeApi());
    IoResult reset = fd.Reset(32);
    Expect(reset.ok());
    Expect(g_fake.close_calls == 1);
    Expect(g_fake.last_closed_fd == 31);
    Expect(fd.get() == 32);

    const int released = fd.Release();
    Expect(released == 32);
    Expect(!fd);
    Expect(g_fake.close_calls == 1);

    g_fake.close_result = -1;
    g_fake.close_errno = EINTR;
    UniqueFd interrupted(41, FakeApi());
    const IoResult interrupted_result = interrupted.Reset();
    Expect(!interrupted_result.ok());
    Expect(interrupted_result.error == IoError::kInterrupted);
    Expect(interrupted_result.system_errno == EINTR);
    Expect(g_fake.close_calls == 2);
    Expect(!interrupted);
}

void TestEintrIsReturnedWithoutRetry() {
    ResetFake();
    g_fake.read_result = -1;
    g_fake.read_errno = EINTR;
    char byte{};
    const IoResult result = ReadOnce(FakeApi(), 5, &byte, sizeof(byte));
    Expect(!result.ok());
    Expect(result.bytes_transferred == 0);
    Expect(result.error == IoError::kInterrupted);
    Expect(result.system_errno == EINTR);
    Expect(g_fake.read_calls == 1);
}

void TestShortTransfersAreSuccessful() {
    ResetFake();
    char buffer[8]{};
    g_fake.read_result = 3;
    g_fake.read_errno = EIO;
    const IoResult read_result = ReadOnce(FakeApi(), 6, buffer, sizeof(buffer));
    Expect(read_result.ok());
    Expect(read_result.bytes_transferred == 3);
    Expect(read_result.system_errno == 0);
    Expect(g_fake.read_calls == 1);

    g_fake.write_result = 2;
    g_fake.write_errno = ENOSPC;
    const IoResult write_result = WriteOnce(FakeApi(), 7, buffer, sizeof(buffer));
    Expect(write_result.ok());
    Expect(write_result.bytes_transferred == 2);
    Expect(write_result.system_errno == 0);
    Expect(g_fake.write_calls == 1);
}

void TestFailurePreservesRawErrno() {
    ResetFake();
    g_fake.write_result = -1;
    g_fake.write_errno = EIO;
    const char byte{};
    const IoResult result = WriteOnce(FakeApi(), 8, &byte, sizeof(byte));
    Expect(!result.ok());
    Expect(result.error == IoError::kIo);
    Expect(result.system_errno == EIO);
    Expect(g_fake.write_calls == 1);
}

void TestProductionTableUsesRealSyscalls() {
    const SyscallApi& api = ProductionSyscalls();
    Expect(api.functions().open != nullptr);
    Expect(api.functions().read != nullptr);
    Expect(api.functions().write != nullptr);
    Expect(api.functions().lseek != nullptr);
    Expect(api.functions().close != nullptr);
    Expect(api.functions().poll != nullptr);

    int pipe_fds[2]{};
    Expect(::pipe(pipe_fds) == 0);
    UniqueFd reader(pipe_fds[0]);
    UniqueFd writer(pipe_fds[1]);

    const char sent = 'x';
    const IoResult write_result = WriteOnce(api, writer.get(), &sent, sizeof(sent));
    Expect(write_result.ok());
    Expect(write_result.bytes_transferred == 1);

    char received{};
    const IoResult read_result = ReadOnce(api, reader.get(), &received, sizeof(received));
    Expect(read_result.ok());
    Expect(read_result.bytes_transferred == 1);
    Expect(received == sent);
}

}  // namespace

int main() {
    TestMoveOnlyAndLifetime();
    TestResetReleaseAndCloseError();
    TestEintrIsReturnedWithoutRetry();
    TestShortTransfersAreSuccessful();
    TestFailurePreservesRawErrno();
    TestProductionTableUsesRealSyscalls();
    return EXIT_SUCCESS;
}
