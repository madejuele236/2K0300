#include "platform/true_ls2k0300/encoder_pair.hpp"
#include "platform/true_ls2k0300/imu_device.hpp"
#include "platform/true_ls2k0300/motor_device.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <type_traits>
#include <utility>

namespace allocation_probe {

bool active = false;
std::size_t count = 0;

void Record() noexcept {
    if (active) {
        ++count;
    }
}

class Scope final {
public:
    Scope() noexcept {
        count = 0;
        active = true;
    }

    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;

    ~Scope() noexcept { active = false; }

    [[nodiscard]] std::size_t Stop() noexcept {
        active = false;
        return count;
    }
};

}  // namespace allocation_probe

void* operator new(std::size_t size) {
    allocation_probe::Record();
    if (void* const memory = std::malloc(size == 0 ? 1 : size)) {
        return memory;
    }
    throw std::bad_alloc();
}

void* operator new[](std::size_t size) {
    allocation_probe::Record();
    if (void* const memory = std::malloc(size == 0 ? 1 : size)) {
        return memory;
    }
    throw std::bad_alloc();
}

void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
    allocation_probe::Record();
    return std::malloc(size == 0 ? 1 : size);
}

void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
    allocation_probe::Record();
    return std::malloc(size == 0 ? 1 : size);
}

void* operator new(std::size_t size, std::align_val_t alignment) {
    allocation_probe::Record();
    void* memory = nullptr;
    const std::size_t requested = size == 0 ? 1 : size;
    if (::posix_memalign(&memory, static_cast<std::size_t>(alignment), requested) == 0) {
        return memory;
    }
    throw std::bad_alloc();
}

void* operator new[](std::size_t size, std::align_val_t alignment) {
    return ::operator new(size, alignment);
}

void* operator new(std::size_t size, std::align_val_t alignment,
                   const std::nothrow_t&) noexcept {
    allocation_probe::Record();
    void* memory = nullptr;
    const std::size_t requested = size == 0 ? 1 : size;
    return ::posix_memalign(&memory, static_cast<std::size_t>(alignment), requested) == 0
               ? memory
               : nullptr;
}

void* operator new[](std::size_t size, std::align_val_t alignment,
                     const std::nothrow_t& tag) noexcept {
    return ::operator new(size, alignment, tag);
}

void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete[](void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void* memory, std::size_t) noexcept { std::free(memory); }
void operator delete(void* memory, const std::nothrow_t&) noexcept { std::free(memory); }
void operator delete[](void* memory, const std::nothrow_t&) noexcept { std::free(memory); }
void operator delete(void* memory, std::align_val_t) noexcept { std::free(memory); }
void operator delete[](void* memory, std::align_val_t) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t, std::align_val_t) noexcept { std::free(memory); }
void operator delete[](void* memory, std::size_t, std::align_val_t) noexcept { std::free(memory); }
void operator delete(void* memory, std::align_val_t, const std::nothrow_t&) noexcept {
    std::free(memory);
}
void operator delete[](void* memory, std::align_val_t, const std::nothrow_t&) noexcept {
    std::free(memory);
}

namespace {

using ls2k::platform::linux_io::SyscallApi;
using namespace ls2k::platform::true_ls2k0300;

struct File final {
    const char* path{nullptr};
    const void* data{nullptr};
    std::size_t size{0};
};

struct OpenFd final {
    int fd{-1};
    const File* file{nullptr};
};

struct FakeSyscalls final {
    std::array<File, 16> files{};
    std::size_t file_count{0};
    std::array<OpenFd, 32> open_fds{};
    int next_fd{20};
    bool accept_any_path{false};
    bool seek_supported{true};
    std::size_t opens{0};
    std::size_t reads{0};
    std::size_t writes{0};
    std::size_t seeks{0};
    std::size_t closes{0};

    void Add(const char* path, const void* data, std::size_t size) noexcept {
        files[file_count++] = File{path, data, size};
    }

    [[nodiscard]] const File* FindFile(const char* path) const noexcept {
        for (std::size_t index = 0; index < file_count; ++index) {
            if (std::strcmp(files[index].path, path) == 0) {
                return &files[index];
            }
        }
        return nullptr;
    }

    [[nodiscard]] OpenFd* FindFd(int fd) noexcept {
        for (OpenFd& open : open_fds) {
            if (open.fd == fd) {
                return &open;
            }
        }
        return nullptr;
    }
};

FakeSyscalls* g_fake = nullptr;

int Open(const char* path, int, mode_t) {
    const File* const file = g_fake->FindFile(path);
    if (file == nullptr && !g_fake->accept_any_path) {
        errno = ENOENT;
        return -1;
    }
    for (OpenFd& slot : g_fake->open_fds) {
        if (slot.fd < 0) {
            const int fd = g_fake->next_fd++;
            slot = OpenFd{fd, file};
            ++g_fake->opens;
            return fd;
        }
    }
    errno = EMFILE;
    return -1;
}

ssize_t Read(int fd, void* output, std::size_t capacity) {
    OpenFd* const open = g_fake->FindFd(fd);
    if (open == nullptr || open->file == nullptr) {
        errno = EBADF;
        return -1;
    }
    const std::size_t size = std::min(capacity, open->file->size);
    std::memcpy(output, open->file->data, size);
    ++g_fake->reads;
    return static_cast<ssize_t>(size);
}

ssize_t Write(int fd, const void*, std::size_t size) {
    if (g_fake->FindFd(fd) == nullptr) {
        errno = EBADF;
        return -1;
    }
    ++g_fake->writes;
    return static_cast<ssize_t>(size);
}

off_t Seek(int fd, off_t, int) {
    if (g_fake->FindFd(fd) == nullptr) {
        errno = EBADF;
        return -1;
    }
    if (!g_fake->seek_supported) {
        errno = ESPIPE;
        return -1;
    }
    ++g_fake->seeks;
    return 0;
}

int Close(int fd) {
    OpenFd* const open = g_fake->FindFd(fd);
    if (open == nullptr) {
        errno = EBADF;
        return -1;
    }
    *open = OpenFd{};
    ++g_fake->closes;
    return 0;
}

int Poll(pollfd*, nfds_t, int) { return 0; }

SyscallApi MakeApi() {
    return SyscallApi({&Open, &Read, &Write, &Seek, &Close, &Poll});
}

void Expect(bool condition, const char* message) noexcept {
    if (!condition) {
        std::fprintf(stderr, "hot_path_allocation_contract_test: %s\n", message);
        std::abort();
    }
}

std::size_t EnumerateImu(void* context,
                         std::array<std::string, kMaxImuCandidates>& paths) {
    paths[0] = static_cast<const char*>(context);
    return 1;
}

void TestEncoderPersistentReadHasNoAllocation() {
    const std::int16_t left_value = 321;
    const std::int16_t right_value = -123;
    FakeSyscalls fake;
    fake.Add("/encoder/left", &left_value, sizeof(left_value));
    fake.Add("/encoder/right", &right_value, sizeof(right_value));
    g_fake = &fake;
    const SyscallApi api = MakeApi();
    EncoderPair encoder("/encoder/left",
                        "/encoder/right",
                        api,
                        EncoderIoPolicy::kPreferPersistent);
    const EncoderInitResult init = encoder.Initialize();
    Expect(init.ready && init.mode == EncoderIoMode::kPersistent,
           "encoder setup did not select the explicitly requested persistent path");

    const std::size_t seeks_before = fake.seeks;
    const std::size_t reads_before = fake.reads;
    EncoderPairResult result{};
    std::size_t allocations = 0;
    {
        allocation_probe::Scope measurement;
        result = encoder.ReadCounts();
        allocations = measurement.Stop();
    }

    Expect(result.valid() && result.left.count == left_value && result.right.count == right_value,
           "encoder hot read did not succeed with the prepared fixed state");
    Expect(fake.seeks - seeks_before == 2 && fake.reads - reads_before == 2,
           "encoder hot read did not use exactly two persistent seek/read operations");
    Expect(allocations == 0, "EncoderPair::ReadCounts allocated dynamically");
}

void TestImuPersistentSampleHasNoAllocation() {
    constexpr char kName[] = "IMU963RA\n";
    constexpr std::array<const char*, kImuChannelCount> kPaths{{
        "/iio/in_accel_x_raw", "/iio/in_accel_y_raw", "/iio/in_accel_z_raw",
        "/iio/in_anglvel_x_raw", "/iio/in_anglvel_y_raw", "/iio/in_anglvel_z_raw",
        "/iio/in_magn_x_raw", "/iio/in_magn_y_raw", "/iio/in_magn_z_raw",
    }};
    constexpr std::array<const char*, kImuChannelCount> kValues{{
        "1", "2", "3", "4", "5", "6", "7", "8", "9",
    }};

    FakeSyscalls fake;
    fake.Add("/iio/name", kName, sizeof(kName) - 1);
    for (std::size_t index = 0; index < kPaths.size(); ++index) {
        fake.Add(kPaths[index], kValues[index], std::strlen(kValues[index]));
    }
    g_fake = &fake;
    const SyscallApi api = MakeApi();
    const ImuFilesystemApi filesystem{
        const_cast<char*>("/iio/name"), &EnumerateImu};
    ImuDevice imu(api, filesystem);
    const ImuInitResult init = imu.Initialize();
    Expect(init.ready && init.type == ImuType::kImu963Ra && init.mode == ImuIoMode::kPersistent,
           "IMU setup did not establish the nine-channel persistent path");

    const std::size_t seeks_before = fake.seeks;
    const std::size_t reads_before = fake.reads;
    ImuRawSample sample{};
    std::size_t allocations = 0;
    {
        allocation_probe::Scope measurement;
        sample = imu.ReadRawSample();
        allocations = measurement.Stop();
    }

    Expect(sample.valid && sample.status == ImuStatus::kOk &&
               sample.acc_x == 1 && sample.gyro_z == 6 && sample.mag_z == 9,
           "IMU hot sample did not succeed with the prepared fixed state");
    Expect(fake.seeks - seeks_before == kImuChannelCount &&
               fake.reads - reads_before == kImuChannelCount,
           "IMU hot sample did not use exactly nine persistent seek/read operations");
    Expect(allocations == 0, "ImuDevice::ReadRawSample allocated dynamically");
}

void TestMotorDefaultApplyHasNoAllocation() {
    FakeSyscalls fake;
    fake.accept_any_path = true;
    g_fake = &fake;
    const SyscallApi api = MakeApi();
    const MotorPaths paths{"lpwm", "rpwm", "lgpio", "rgpio", "lesc", "resc"};
    MotorDevice motor(paths, api);
    Expect(motor.Initialize().ok(),
           "motor setup did not establish the open/write/close device owner");

    const std::size_t opens_before = fake.opens;
    const std::size_t writes_before = fake.writes;
    const std::size_t closes_before = fake.closes;
    MotorResult result{};
    std::size_t allocations = 0;
    {
        allocation_probe::Scope measurement;
        result = motor.Apply(MotorCommand{100, 200, 300, 400});
        allocations = measurement.Stop();
    }

    Expect(result.ok(), "motor hot Apply did not succeed with the prepared fixed state");
    Expect(fake.opens - opens_before == 6 && fake.writes - writes_before == 6 &&
               fake.closes - closes_before == 6,
           "motor hot Apply did not perform exactly six open/write/close operations");
    Expect(allocations == 0, "MotorDevice::Apply allocated dynamically");
}

}  // namespace

int main() {
    static_assert(noexcept(std::declval<EncoderPair&>().ReadCounts()));
    static_assert(noexcept(std::declval<ImuDevice&>().ReadRawSample()));
    static_assert(noexcept(std::declval<MotorDevice&>().Apply(std::declval<const MotorCommand&>())));

    TestEncoderPersistentReadHasNoAllocation();
    TestImuPersistentSampleHasNoAllocation();
    TestMotorDefaultApplyHasNoAllocation();
}
