#include "platform/true_ls2k0300/imu_device.hpp"

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>

namespace {

using ls2k::platform::linux_io::SyscallApi;
using namespace ls2k::platform::true_ls2k0300;

struct FakeState {
    std::unordered_map<std::string, std::string> files;
    std::unordered_map<int, std::string> open_fds;
    int next_fd{20};
    int opens{0};
    int closes{0};
    bool seek_supported{true};
    std::string fail_path;
};

FakeState* g_fake = nullptr;

int Open(const char* path, int, mode_t) {
    if (g_fake->files.find(path) == g_fake->files.end()) { errno = ENOENT; return -1; }
    const int fd = g_fake->next_fd++;
    g_fake->open_fds.emplace(fd, path);
    ++g_fake->opens;
    return fd;
}
ssize_t Read(int fd, void* output, std::size_t size) {
    const auto open = g_fake->open_fds.find(fd);
    if (open == g_fake->open_fds.end()) { errno = EBADF; return -1; }
    if (open->second == g_fake->fail_path) { errno = EIO; return -1; }
    const std::string& value = g_fake->files.at(open->second);
    const std::size_t count = value.size() < size ? value.size() : size;
    std::memcpy(output, value.data(), count);
    return static_cast<ssize_t>(count);
}
ssize_t Write(int, const void*, std::size_t) { errno = EIO; return -1; }
off_t Seek(int, off_t, int) { if (!g_fake->seek_supported) { errno = ESPIPE; return -1; } return 0; }
int Close(int fd) {
    if (g_fake->open_fds.erase(fd) == 0) { errno = EBADF; return -1; }
    ++g_fake->closes;
    return 0;
}
int Poll(pollfd*, nfds_t, int) { return 0; }

std::size_t Enumerate(void* context, std::array<std::string, kMaxImuCandidates>& paths) {
    paths[0] = *static_cast<std::string*>(context);
    return 1;
}

SyscallApi MakeApi() { return SyscallApi({&Open, &Read, &Write, &Seek, &Close, &Poll}); }

void Populate(FakeState& fake, const std::string& model) {
    fake.files["/iio/name"] = model + "\n";
    const char* names[] = {"in_accel_x_raw", "in_accel_y_raw", "in_accel_z_raw",
                           "in_anglvel_x_raw", "in_anglvel_y_raw", "in_anglvel_z_raw",
                           "in_magn_x_raw", "in_magn_y_raw", "in_magn_z_raw"};
    for (int index = 0; index < 9; ++index) fake.files[std::string("/iio/") + names[index]] = std::to_string(index + 1);
}

void Expect(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }

void TestModelAxesAndResourceOwnership() {
    FakeState fake;
    Populate(fake, "IMU963RA");
    g_fake = &fake;
    std::string name_path = "/iio/name";
    const ImuFilesystemApi filesystem{&name_path, &Enumerate};
    const SyscallApi api = MakeApi();
    {
        ImuDevice imu(api, filesystem);
        const auto init = imu.Initialize();
        Expect(init.ready && init.type == ImuType::kImu963Ra, "IMU963RA discovery failed");
        Expect(init.mode == ImuIoMode::kPersistent, "persistent mode not selected");
        const auto sample = imu.ReadRawSample();
        Expect(sample.valid, "valid 9-channel sample rejected");
        Expect(sample.acc_x == 1 && sample.acc_y == 2 && sample.acc_z == 3, "accelerometer axis order changed");
        Expect(sample.gyro_x == 4 && sample.gyro_y == 5 && sample.gyro_z == 6, "gyro axis order changed");
        Expect(sample.mag_x == 7 && sample.mag_y == 8 && sample.mag_z == 9, "magnetometer axis order changed");
        Expect(imu.source() == "/iio/name" && imu.device_dir() == "/iio", "resolved path contract changed");
    }
    Expect(fake.open_fds.empty() && fake.opens == fake.closes, "IMU owner leaked fds");
}

void TestRequiredChannelFailureInvalidatesWholeSample() {
    FakeState fake;
    Populate(fake, "IMU660RB");
    g_fake = &fake;
    std::string name_path = "/iio/name";
    const ImuFilesystemApi filesystem{&name_path, &Enumerate};
    const SyscallApi api = MakeApi();
    ImuDevice imu(api, filesystem);
    Expect(imu.Initialize().ready, "setup failed");
    fake.fail_path = "/iio/in_anglvel_y_raw";
    const auto sample = imu.ReadRawSample();
    Expect(!sample.valid, "required-channel failure was hidden");
    Expect(sample.status == ImuStatus::kReadFailed && sample.failed_channel == 4, "failure owner/channel lost");
    Expect(sample.acc_x == 0 && sample.gyro_x == 0, "partial sample escaped as compensated data");
}

void TestFallbackAndSixChannelContract() {
    FakeState fake;
    Populate(fake, "IMU660RA");
    fake.seek_supported = false;
    fake.files.erase("/iio/in_magn_x_raw");
    fake.files.erase("/iio/in_magn_y_raw");
    fake.files.erase("/iio/in_magn_z_raw");
    g_fake = &fake;
    std::string name_path = "/iio/name";
    const ImuFilesystemApi filesystem{&name_path, &Enumerate};
    const SyscallApi api = MakeApi();
    ImuDevice imu(api, filesystem);
    const auto init = imu.Initialize();
    Expect(init.ready && init.mode == ImuIoMode::kOpenReadClose, "fallback mode not selected");
    const auto sample = imu.ReadRawSample();
    Expect(sample.valid && sample.gyro_z == 6, "six-channel model contract changed");
    Expect(sample.mag_x == 0 && fake.open_fds.empty(), "optional magn channels were accessed or fds leaked");
}

void TestUnsupportedModelFailsDiscovery() {
    FakeState fake;
    Populate(fake, "SOMETHING_ELSE");
    g_fake = &fake;
    std::string name_path = "/iio/name";
    const ImuFilesystemApi filesystem{&name_path, &Enumerate};
    const SyscallApi api = MakeApi();
    ImuDevice imu(api, filesystem);
    const auto init = imu.Initialize();
    Expect(!init.ready && init.status == ImuStatus::kUnsupportedModel, "unsupported model accepted");
}

}  // namespace

int main() {
    static_assert(!std::is_copy_constructible_v<ImuDevice>);
    static_assert(std::is_nothrow_move_constructible_v<ImuDevice>);
    TestModelAxesAndResourceOwnership();
    TestRequiredChannelFailureInvalidatesWholeSample();
    TestFallbackAndSixChannelContract();
    TestUnsupportedModelFailsDiscovery();
}
