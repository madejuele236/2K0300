#include "platform/true_ls2k0300/imu_device.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <utility>

namespace ls2k::platform::true_ls2k0300 {
namespace {

constexpr char kIioDevicesRoot[] = "/sys/bus/iio/devices";
constexpr std::array<const char*, kImuChannelCount> kChannelFiles{{
    "in_accel_x_raw", "in_accel_y_raw", "in_accel_z_raw",
    "in_anglvel_x_raw", "in_anglvel_y_raw", "in_anglvel_z_raw",
    "in_magn_x_raw", "in_magn_y_raw", "in_magn_z_raw",
}};

bool IsSpace(char value) noexcept {
    return value == ' ' || value == '\t' || value == '\n' || value == '\r' ||
           value == '\f' || value == '\v';
}

bool ParseInt16(const char* data, std::size_t length, std::int16_t& out) noexcept {
    if (data == nullptr || length == 0) {
        return false;
    }
    std::size_t index = 0;
    while (index < length && IsSpace(data[index])) {
        ++index;
    }
    int sign = 1;
    if (index < length && (data[index] == '+' || data[index] == '-')) {
        sign = data[index] == '-' ? -1 : 1;
        ++index;
    }
    if (index >= length || data[index] < '0' || data[index] > '9') {
        return false;
    }
    int magnitude = 0;
    const int maximum = sign < 0 ? 32768 : 32767;
    while (index < length && data[index] >= '0' && data[index] <= '9') {
        magnitude = magnitude * 10 + (data[index] - '0');
        if (magnitude > maximum) {
            return false;
        }
        ++index;
    }
    while (index < length) {
        if (!IsSpace(data[index]) && data[index] != '\0') {
            return false;
        }
        ++index;
    }
    out = static_cast<std::int16_t>(sign * magnitude);
    return true;
}

ImuType ParseModel(const char* data, std::size_t length) noexcept {
    while (length > 0 && (IsSpace(data[length - 1]) || data[length - 1] == '\0')) {
        --length;
    }
    if (length == 8 && std::memcmp(data, "IMU660RA", 8) == 0) return ImuType::kImu660Ra;
    if (length == 8 && std::memcmp(data, "IMU660RB", 8) == 0) return ImuType::kImu660Rb;
    if (length == 8 && std::memcmp(data, "IMU963RA", 8) == 0) return ImuType::kImu963Ra;
    return ImuType::kNotFound;
}

std::size_t EnumerateProductionNamePaths(
    void*, std::array<std::string, kMaxImuCandidates>& paths) {
    if (const char* name_path = std::getenv("LS2K_IMU_NAME_PATH");
        name_path != nullptr && name_path[0] != '\0') {
        paths[0] = name_path;
        return 1;
    }
    if (const char* device_dir = std::getenv("LS2K_IMU_DEVICE_DIR");
        device_dir != nullptr && device_dir[0] != '\0') {
        paths[0] = std::string(device_dir) + "/name";
        return 1;
    }

    struct stat root_stat {};
    if (::stat(kIioDevicesRoot, &root_stat) != 0 || !S_ISDIR(root_stat.st_mode)) {
        return 0;
    }
    DIR* directory = ::opendir(kIioDevicesRoot);
    if (directory == nullptr) {
        return 0;
    }
    std::size_t count = 0;
    while (count < paths.size()) {
        const dirent* entry = ::readdir(directory);
        if (entry == nullptr) break;
        if (std::strncmp(entry->d_name, "iio:device", 10) != 0) continue;
        const std::string device_dir = std::string(kIioDevicesRoot) + "/" + entry->d_name;
        struct stat device_stat {};
        if (::stat(device_dir.c_str(), &device_stat) != 0 || !S_ISDIR(device_stat.st_mode)) continue;
        paths[count++] = device_dir + "/name";
    }
    ::closedir(directory);
    std::sort(paths.begin(), paths.begin() + static_cast<std::ptrdiff_t>(count));
    return count;
}

}  // namespace

const ImuFilesystemApi& ProductionImuFilesystem() noexcept {
    static const ImuFilesystemApi api{nullptr, &EnumerateProductionNamePaths};
    return api;
}

ImuDevice::ImuDevice(const linux_io::SyscallApi& syscalls, const ImuFilesystemApi& filesystem)
    : syscalls_(&syscalls), filesystem_(&filesystem) {
    for (linux_io::UniqueFd& fd : fds_) {
        fd = linux_io::UniqueFd(-1, syscalls);
    }
}

void ImuDevice::ResetState() noexcept {
    for (linux_io::UniqueFd& fd : fds_) {
        static_cast<void>(fd.Reset());
    }
    paths_ = {};
    source_.clear();
    device_dir_.clear();
    type_ = ImuType::kNotFound;
    mode_ = ImuIoMode::kUninitialized;
    required_channels_ = 0;
    ready_ = false;
}

ImuStatus ImuDevice::ReadToken(const std::string& path, char* buffer, std::size_t capacity,
                               std::size_t& length) noexcept {
    length = 0;
    linux_io::UniqueFd fd(syscalls_->functions().open(path.c_str(), O_RDONLY | O_CLOEXEC, 0), *syscalls_);
    if (!fd) return ImuStatus::kOpenFailed;
    const linux_io::IoResult read = linux_io::ReadOnce(*syscalls_, fd.get(), buffer, capacity);
    const linux_io::IoResult close = fd.Reset();
    if (!read.ok() || read.bytes_transferred <= 0) return ImuStatus::kReadFailed;
    if (!close.ok()) return ImuStatus::kCloseFailed;
    length = static_cast<std::size_t>(read.bytes_transferred);
    return ImuStatus::kOk;
}

ImuStatus ImuDevice::ReadChannelPersistent(std::size_t index, std::int16_t& value) noexcept {
    if (index >= required_channels_ || !fds_[index]) return ImuStatus::kNotInitialized;
    if (syscalls_->functions().lseek(fds_[index].get(), 0, SEEK_SET) < 0) return ImuStatus::kSeekFailed;
    char buffer[32]{};
    const linux_io::IoResult read = linux_io::ReadOnce(*syscalls_, fds_[index].get(), buffer, sizeof(buffer));
    if (!read.ok() || read.bytes_transferred <= 0) return ImuStatus::kReadFailed;
    return ParseInt16(buffer, static_cast<std::size_t>(read.bytes_transferred), value)
               ? ImuStatus::kOk : ImuStatus::kParseFailed;
}

ImuStatus ImuDevice::ReadChannelOpenClose(std::size_t index, std::int16_t& value) noexcept {
    if (index >= required_channels_) return ImuStatus::kNotInitialized;
    char buffer[32]{};
    std::size_t length = 0;
    const ImuStatus status = ReadToken(paths_[index], buffer, sizeof(buffer), length);
    if (status != ImuStatus::kOk) return status;
    return ParseInt16(buffer, length, value) ? ImuStatus::kOk : ImuStatus::kParseFailed;
}

bool ImuDevice::OpenPersistentChannels(ImuInitResult& result) noexcept {
    for (std::size_t index = 0; index < required_channels_; ++index) {
        const int opened = syscalls_->functions().open(paths_[index].c_str(), O_RDONLY | O_CLOEXEC, 0);
        if (opened < 0) {
            result.status = ImuStatus::kOpenFailed;
            result.failed_channel = static_cast<std::uint8_t>(index);
            return false;
        }
        static_cast<void>(fds_[index].Reset(opened));
        std::int16_t ignored = 0;
        const ImuStatus status = ReadChannelPersistent(index, ignored);
        if (status != ImuStatus::kOk) {
            result.status = status;
            result.failed_channel = static_cast<std::uint8_t>(index);
            return false;
        }
    }
    return true;
}

bool ImuDevice::ProbeOpenCloseChannels(ImuInitResult& result) noexcept {
    for (std::size_t index = 0; index < required_channels_; ++index) {
        std::int16_t ignored = 0;
        const ImuStatus status = ReadChannelOpenClose(index, ignored);
        if (status != ImuStatus::kOk) {
            result.status = status;
            result.failed_channel = static_cast<std::uint8_t>(index);
            return false;
        }
    }
    return true;
}

ImuInitResult ImuDevice::Initialize() {
    ResetState();
    ImuInitResult result{};
    if (filesystem_ == nullptr || filesystem_->enumerate_name_paths == nullptr) {
        result.status = ImuStatus::kDiscoveryFailed;
        return result;
    }

    std::array<std::string, kMaxImuCandidates> candidates{};
    const std::size_t count = std::min(filesystem_->enumerate_name_paths(filesystem_->context, candidates),
                                       candidates.size());
    bool saw_readable_name = false;
    for (std::size_t index = 0; index < count; ++index) {
        char name[32]{};
        std::size_t length = 0;
        if (ReadToken(candidates[index], name, sizeof(name), length) != ImuStatus::kOk) continue;
        saw_readable_name = true;
        const ImuType candidate_type = ParseModel(name, length);
        if (candidate_type == ImuType::kNotFound) continue;
        type_ = candidate_type;
        source_ = candidates[index];
        const std::size_t slash = source_.find_last_of('/');
        device_dir_ = slash == std::string::npos ? source_ : source_.substr(0, slash);
        for (std::size_t channel = 0; channel < paths_.size(); ++channel) {
            paths_[channel] = device_dir_ + "/" + kChannelFiles[channel];
        }
        break;
    }
    if (type_ == ImuType::kNotFound) {
        result.status = saw_readable_name ? ImuStatus::kUnsupportedModel : ImuStatus::kDiscoveryFailed;
        return result;
    }

    required_channels_ = type_ == ImuType::kImu963Ra ? 9U : 6U;
    result.type = type_;
    if (OpenPersistentChannels(result)) {
        mode_ = ImuIoMode::kPersistent;
    } else {
        for (linux_io::UniqueFd& fd : fds_) static_cast<void>(fd.Reset());
        if (!ProbeOpenCloseChannels(result)) return result;
        mode_ = ImuIoMode::kOpenReadClose;
    }
    ready_ = true;
    result.ready = true;
    result.mode = mode_;
    result.status = ImuStatus::kOk;
    result.failed_channel = 0xff;
    return result;
}

ImuRawSample ImuDevice::ReadRawSample() noexcept {
    ImuRawSample sample{};
    sample.type = type_;
    if (!ready_) return sample;

    std::array<std::int16_t, kImuChannelCount> raw{};
    for (std::size_t index = 0; index < required_channels_; ++index) {
        const ImuStatus status = mode_ == ImuIoMode::kPersistent
                                     ? ReadChannelPersistent(index, raw[index])
                                     : ReadChannelOpenClose(index, raw[index]);
        if (status != ImuStatus::kOk) {
            sample.status = status;
            sample.failed_channel = static_cast<std::uint8_t>(index);
            return sample;
        }
    }
    sample.acc_x = raw[0]; sample.acc_y = raw[1]; sample.acc_z = raw[2];
    sample.gyro_x = raw[3]; sample.gyro_y = raw[4]; sample.gyro_z = raw[5];
    if (required_channels_ == 9U) {
        sample.mag_x = raw[6]; sample.mag_y = raw[7]; sample.mag_z = raw[8];
    }
    sample.valid = true;
    sample.status = ImuStatus::kOk;
    sample.failed_channel = 0xff;
    return sample;
}

void ImuDevice::Shutdown() noexcept {
    ResetState();
}

const char* ImuTypeName(ImuType type) noexcept {
    switch (type) {
        case ImuType::kNotFound: return "unknown";
        case ImuType::kImu660Ra: return "imu660ra";
        case ImuType::kImu660Rb: return "imu660rb";
        case ImuType::kImu963Ra: return "imu963ra";
    }
    return "unknown";
}

const char* ImuStatusName(ImuStatus status) noexcept {
    switch (status) {
        case ImuStatus::kOk: return "ok";
        case ImuStatus::kNotInitialized: return "not-initialized";
        case ImuStatus::kDiscoveryFailed: return "discovery-failed";
        case ImuStatus::kUnsupportedModel: return "unsupported-model";
        case ImuStatus::kOpenFailed: return "open-failed";
        case ImuStatus::kSeekFailed: return "seek-failed";
        case ImuStatus::kReadFailed: return "read-failed";
        case ImuStatus::kParseFailed: return "parse-failed";
        case ImuStatus::kCloseFailed: return "close-failed";
    }
    return "unknown";
}

const char* ImuIoModeName(ImuIoMode mode) noexcept {
    switch (mode) {
        case ImuIoMode::kUninitialized: return "uninitialized";
        case ImuIoMode::kPersistent: return "persistent";
        case ImuIoMode::kOpenReadClose: return "open-read-close";
    }
    return "unknown";
}

}  // namespace ls2k::platform::true_ls2k0300
