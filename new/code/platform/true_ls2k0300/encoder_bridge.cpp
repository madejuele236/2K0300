#include "platform/true_ls2k0300/encoder_pair.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <utility>

namespace ls2k::platform::true_ls2k0300 {
namespace {

EncoderStatus StatusFromIo(const linux_io::IoResult& result) noexcept {
    return result.ok() ? EncoderStatus::kOk : EncoderStatus::kReadFailed;
}

// The board character driver has been observed writing one 32-bit word even
// when userspace requests the historical 16-bit count. Keep the syscall count
// unchanged, but isolate that device write inside a suitably sized/aligned
// owner-side object before explicitly decoding the contracted low 16 bits.
struct alignas(std::uint32_t) EncoderReadStorage final {
    std::uint32_t word{0xa5a50000U};
};
static_assert(sizeof(EncoderReadStorage) >= sizeof(std::uint32_t),
              "encoder driver isolation storage must accept a 32-bit write");
static_assert(alignof(EncoderReadStorage) >= alignof(std::uint32_t),
              "encoder driver isolation storage must be 32-bit aligned");

std::int32_t DecodeLowInt16(const EncoderReadStorage& storage) noexcept {
    const std::uint16_t low_bits =
        static_cast<std::uint16_t>(storage.word & std::numeric_limits<std::uint16_t>::max());
    std::int16_t signed_count = 0;
    static_assert(sizeof(signed_count) == sizeof(low_bits), "encoder count must remain 16-bit");
    std::memcpy(&signed_count, &low_bits, sizeof(signed_count));
    return static_cast<std::int32_t>(signed_count);
}

}  // namespace

EncoderPair::EncoderPair(std::string left_path,
                         std::string right_path,
                         const linux_io::SyscallApi& syscalls,
                         EncoderIoPolicy io_policy)
    : left_path_(std::move(left_path)),
      right_path_(std::move(right_path)),
      syscalls_(&syscalls),
      left_fd_(-1, syscalls),
      right_fd_(-1, syscalls),
      io_policy_(io_policy) {}

EncoderChannelResult EncoderPair::ReadPersistent(linux_io::UniqueFd& fd) noexcept {
    EncoderChannelResult result{};
    if (!fd) {
        result.status = EncoderStatus::kNotInitialized;
        return result;
    }
    if (syscalls_->functions().lseek(fd.get(), 0, SEEK_SET) < 0) {
        result.status = EncoderStatus::kSeekFailed;
        return result;
    }

    EncoderReadStorage storage{};
    const linux_io::IoResult read =
        linux_io::ReadOnce(*syscalls_, fd.get(), &storage.word, sizeof(std::int16_t));
    // The board encoder driver may update the buffer while returning zero bytes.
    // ReadOnce therefore succeeds for every non-negative syscall result, including zero.
    result.status = StatusFromIo(read);
    if (result.ok()) {
        result.count = DecodeLowInt16(storage);
    }
    return result;
}

EncoderChannelResult EncoderPair::ReadOpenClose(const std::string& path) noexcept {
    EncoderChannelResult result{};
    errno = 0;
    linux_io::UniqueFd fd(syscalls_->functions().open(path.c_str(), O_RDONLY | O_CLOEXEC, 0), *syscalls_);
    if (!fd) {
        result.status = EncoderStatus::kOpenFailed;
        return result;
    }

    EncoderReadStorage storage{};
    const linux_io::IoResult read =
        linux_io::ReadOnce(*syscalls_, fd.get(), &storage.word, sizeof(std::int16_t));
    const linux_io::IoResult close = fd.Reset();
    if (!read.ok()) {
        result.status = EncoderStatus::kReadFailed;
        return result;
    }
    if (!close.ok()) {
        result.status = EncoderStatus::kCloseFailed;
        return result;
    }
    result.status = EncoderStatus::kOk;
    result.count = DecodeLowInt16(storage);
    return result;
}

bool EncoderPair::ProbePersistent(const std::string& path, linux_io::UniqueFd& fd) noexcept {
    static_cast<void>(fd.Reset());
    const int opened = syscalls_->functions().open(path.c_str(), O_RDONLY | O_CLOEXEC, 0);
    if (opened < 0) {
        return false;
    }
    static_cast<void>(fd.Reset(opened));
    if (!ReadPersistent(fd).ok()) {
        return false;
    }
    return ReadPersistent(fd).ok();
}

EncoderInitResult EncoderPair::Initialize() noexcept {
    ready_ = false;
    mode_ = EncoderIoMode::kUninitialized;
    static_cast<void>(left_fd_.Reset());
    static_cast<void>(right_fd_.Reset());

    if (io_policy_ == EncoderIoPolicy::kPreferPersistent &&
        ProbePersistent(left_path_, left_fd_) &&
        ProbePersistent(right_path_, right_fd_)) {
        mode_ = EncoderIoMode::kPersistent;
        ready_ = true;
        return {true, mode_, EncoderStatus::kOk};
    }

    static_cast<void>(left_fd_.Reset());
    static_cast<void>(right_fd_.Reset());
    const EncoderChannelResult left = ReadOpenClose(left_path_);
    if (!left.ok()) {
        return {false, mode_, left.status};
    }
    const EncoderChannelResult right = ReadOpenClose(right_path_);
    if (!right.ok()) {
        return {false, mode_, right.status};
    }

    mode_ = EncoderIoMode::kOpenReadClose;
    ready_ = true;
    return {true, mode_, EncoderStatus::kOk};
}

EncoderPairResult EncoderPair::ReadCounts() noexcept {
    EncoderPairResult result{};
    if (!ready_) {
        return result;
    }
    if (mode_ == EncoderIoMode::kPersistent) {
        result.left = ReadPersistent(left_fd_);
        result.right = ReadPersistent(right_fd_);
        return result;
    }
    if (mode_ == EncoderIoMode::kOpenReadClose) {
        result.left = ReadOpenClose(left_path_);
        result.right = ReadOpenClose(right_path_);
    }
    return result;
}

void EncoderPair::Shutdown() noexcept {
    ready_ = false;
    mode_ = EncoderIoMode::kUninitialized;
    static_cast<void>(left_fd_.Reset());
    static_cast<void>(right_fd_.Reset());
}

const char* EncoderStatusName(EncoderStatus status) noexcept {
    switch (status) {
        case EncoderStatus::kOk: return "ok";
        case EncoderStatus::kNotInitialized: return "not-initialized";
        case EncoderStatus::kOpenFailed: return "open-failed";
        case EncoderStatus::kSeekFailed: return "seek-failed";
        case EncoderStatus::kReadFailed: return "read-failed";
        case EncoderStatus::kCloseFailed: return "close-failed";
    }
    return "unknown";
}

const char* EncoderIoModeName(EncoderIoMode mode) noexcept {
    switch (mode) {
        case EncoderIoMode::kUninitialized: return "uninitialized";
        case EncoderIoMode::kPersistent: return "persistent";
        case EncoderIoMode::kOpenReadClose: return "open-read-close";
    }
    return "unknown";
}

}  // namespace ls2k::platform::true_ls2k0300
