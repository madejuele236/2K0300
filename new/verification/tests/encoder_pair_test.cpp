#include "platform/true_ls2k0300/encoder_mapping.hpp"
#include "platform/true_ls2k0300/encoder_pair.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace {

using ls2k::platform::linux_io::SyscallApi;
using ls2k::platform::true_ls2k0300::EncoderIoMode;
using ls2k::platform::true_ls2k0300::EncoderIoPolicy;
using ls2k::platform::true_ls2k0300::EncoderPair;
using ls2k::platform::true_ls2k0300::EncoderStatus;

struct FakeState {
    std::unordered_map<std::string, std::int16_t> values;
    std::unordered_map<int, std::string> open_fds;
    int next_fd{10};
    int opens{0};
    int closes{0};
    int reads{0};
    int seeks{0};
    bool seek_supported{true};
    bool fail_reads{false};
    bool zero_byte_success{false};
    bool writes_four_bytes{false};
    std::vector<int> closed_fds;
};

FakeState* g_fake = nullptr;

int Open(const char* path, int, mode_t) {
    const auto found = g_fake->values.find(path);
    if (found == g_fake->values.end()) { errno = ENOENT; return -1; }
    const int fd = g_fake->next_fd++;
    g_fake->open_fds.emplace(fd, path);
    ++g_fake->opens;
    return fd;
}

ssize_t Read(int fd, void* output, std::size_t size) {
    ++g_fake->reads;
    if (g_fake->fail_reads) { errno = EIO; return -1; }
    const auto open = g_fake->open_fds.find(fd);
    if (open == g_fake->open_fds.end() || size < sizeof(std::int16_t)) { errno = EBADF; return -1; }
    const std::int16_t value = g_fake->values.at(open->second);
    if (g_fake->writes_four_bytes) {
        const std::uint32_t driver_word = 0x5a5a0000U | static_cast<std::uint16_t>(value);
        // Models the board defect exactly: count remains 2, but the driver
        // writes a complete 32-bit word and reports zero bytes.
        std::memcpy(output, &driver_word, sizeof(driver_word));
    } else {
        std::memcpy(output, &value, sizeof(value));
    }
    return g_fake->zero_byte_success ? 0 : static_cast<ssize_t>(sizeof(value));
}

ssize_t Write(int, const void*, std::size_t) { errno = EIO; return -1; }
off_t Seek(int, off_t, int) {
    ++g_fake->seeks;
    if (!g_fake->seek_supported) { errno = ESPIPE; return -1; }
    return 0;
}
int Close(int fd) {
    if (g_fake->open_fds.erase(fd) == 0) { errno = EBADF; return -1; }
    ++g_fake->closes;
    g_fake->closed_fds.push_back(fd);
    return 0;
}
int Poll(pollfd*, nfds_t, int) { return 0; }

SyscallApi MakeApi() {
    return SyscallApi({&Open, &Read, &Write, &Seek, &Close, &Poll});
}

void Expect(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void TestPersistentAndZeroByteContract() {
    FakeState fake;
    fake.values = {{"/left", 321}, {"/right", -123}};
    fake.zero_byte_success = true;
    g_fake = &fake;
    const SyscallApi api = MakeApi();
    {
        EncoderPair pair("/left", "/right", api, EncoderIoPolicy::kPreferPersistent);
        const auto init = pair.Initialize();
        Expect(init.ready && init.mode == EncoderIoMode::kPersistent, "persistent mode not selected");
        const auto counts = pair.ReadCounts();
        Expect(counts.valid(), "zero-byte board read must be accepted");
        Expect(counts.left.count == 321 && counts.right.count == -123, "left/right raw mapping changed");
        Expect(fake.open_fds.size() == 2, "persistent mode must retain exactly two fds");
    }
    Expect(fake.open_fds.empty() && fake.closes == fake.opens, "destructor did not close persistent fds");
}

void TestDefaultOpenReadCloseSkipsPersistentProbe() {
    FakeState fake;
    fake.values = {{"/left", 7}, {"/right", 9}};
    g_fake = &fake;
    const SyscallApi api = MakeApi();

    EncoderPair pair("/left", "/right", api);
    const auto init = pair.Initialize();
    Expect(init.ready && init.mode == EncoderIoMode::kOpenReadClose,
           "default mode must be open-read-close");
    Expect(fake.seeks == 0, "default initialization attempted a persistent seek probe");
    Expect(fake.opens == 2 && fake.reads == 2 && fake.closes == 2,
           "default initialization must validate each channel exactly once");
}

void TestFallbackModeIsFixedAtInitialization() {
    FakeState fake;
    fake.values = {{"/left", 7}, {"/right", 9}};
    fake.seek_supported = false;
    g_fake = &fake;
    const SyscallApi api = MakeApi();
    EncoderPair pair("/left", "/right", api, EncoderIoPolicy::kPreferPersistent);
    const auto init = pair.Initialize();
    Expect(init.ready && init.mode == EncoderIoMode::kOpenReadClose, "fallback mode not selected");
    Expect(fake.open_fds.empty(), "fallback initialization leaked probe fds");
    const auto counts = pair.ReadCounts();
    Expect(counts.valid() && counts.left.count == 7 && counts.right.count == 9, "fallback read failed");
    Expect(pair.mode() == EncoderIoMode::kOpenReadClose, "periodic read changed fixed mode");
    Expect(fake.open_fds.empty(), "fallback read did not close resources");
}

void TestFailureRemainsVisible() {
    FakeState fake;
    fake.values = {{"/left", 1}, {"/right", 2}};
    g_fake = &fake;
    const SyscallApi api = MakeApi();
    EncoderPair pair("/left", "/right", api, EncoderIoPolicy::kPreferPersistent);
    Expect(pair.Initialize().ready, "setup failed");
    fake.fail_reads = true;
    const auto counts = pair.ReadCounts();
    Expect(!counts.valid(), "read failure was hidden");
    Expect(counts.left.status == EncoderStatus::kReadFailed, "wrong failure status");
    Expect(pair.mode() == EncoderIoMode::kPersistent, "runtime failure silently changed mode");
    pair.Shutdown();
    Expect(fake.open_fds.empty(), "Shutdown did not close persistent fds");
}

void TestFourByteZeroReturnWriteIsIsolated() {
    FakeState fake;
    fake.values = {{"/left", 32767}, {"/right", -32768}};
    fake.open_fds.emplace(0, "/timer");
    fake.next_fd = 12;
    fake.seek_supported = false;
    fake.zero_byte_success = true;
    fake.writes_four_bytes = true;
    g_fake = &fake;
    const SyscallApi api = MakeApi();

    EncoderPair pair("/left", "/right", api);
    const auto init = pair.Initialize();
    Expect(init.ready && init.mode == EncoderIoMode::kOpenReadClose,
           "four-byte driver model did not reach open/read/close mode");
    const auto counts = pair.ReadCounts();
    Expect(counts.valid(), "four-byte zero-return driver write was rejected");
    Expect(counts.left.count == 32767 && counts.right.count == -32768,
           "explicit low-16-bit signed conversion changed positive/negative counts");
    Expect(fake.open_fds.find(0) != fake.open_fds.end(), "neighboring timer fd was closed");
    Expect(std::find(fake.closed_fds.begin(), fake.closed_fds.end(), 0) == fake.closed_fds.end(),
           "encoder owner propagated overwrite into neighboring resource");
}

void TestBoardLogicalDirectionMapping() {
    const auto backward =
        ls2k::platform::true_ls2k0300::NormalizeEncoderCounts(-753, 870);
    Expect(backward.left == -753 && backward.right == -870,
           "board-backed reverse encoder polarity changed");
    const auto forward =
        ls2k::platform::true_ls2k0300::NormalizeEncoderCounts(753, -870);
    Expect(forward.left == 753 && forward.right == 870,
           "board-backed forward encoder polarity changed");
}

}  // namespace

int main() {
    static_assert(!std::is_copy_constructible_v<EncoderPair>);
    static_assert(std::is_nothrow_move_constructible_v<EncoderPair>);
    TestPersistentAndZeroByteContract();
    TestDefaultOpenReadCloseSkipsPersistentProbe();
    TestFallbackModeIsFixedAtInitialization();
    TestFailureRemainsVisible();
    TestFourByteZeroReturnWriteIsIsolated();
    TestBoardLogicalDirectionMapping();
}
