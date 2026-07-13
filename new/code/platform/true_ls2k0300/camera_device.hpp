#ifndef LS2K_PLATFORM_TRUE_LS2K0300_CAMERA_DEVICE_HPP
#define LS2K_PLATFORM_TRUE_LS2K0300_CAMERA_DEVICE_HPP

#include <cstddef>
#include <cstdint>
#include <memory>

#include "platform/linux/linux_io.hpp"

namespace ls2k::platform::true_ls2k0300 {

struct CameraConfig final {
    const char* device = "/dev/video0";
    int width = 320;
    int height = 240;
    int fps = 60;
    int buffer_count = 3;
    int timeout_ms = 50;
};

struct CameraFrameView final {
    const std::uint8_t* data = nullptr;
    std::size_t bytes = 0;
    int width = 0;
    int height = 0;
    int stride = 0;

    [[nodiscard]] bool valid() const noexcept {
        return data != nullptr && bytes != 0 && width > 0 && height > 0 && stride >= width * 2;
    }
};

// V4L2 operations not covered by the generic Linux syscall mechanism. Tests
// inject this table together with SyscallApi; production binds it once.
struct CameraSystemApi final {
    using IoctlFn = int (*)(int, unsigned long, void*);
    using MmapFn = void* (*)(void*, std::size_t, int, int, int, std::int64_t);
    using MunmapFn = int (*)(void*, std::size_t);

    IoctlFn ioctl = nullptr;
    MmapFn mmap = nullptr;
    MunmapFn munmap = nullptr;
};

[[nodiscard]] const CameraSystemApi& ProductionCameraSystemApi() noexcept;

class CameraDevice final {
public:
    explicit CameraDevice(
        const linux_io::SyscallApi& syscalls = linux_io::ProductionSyscalls(),
        const CameraSystemApi& system = ProductionCameraSystemApi()) noexcept;
    ~CameraDevice() noexcept;

    CameraDevice(const CameraDevice&) = delete;
    CameraDevice& operator=(const CameraDevice&) = delete;
    CameraDevice(CameraDevice&&) noexcept;
    CameraDevice& operator=(CameraDevice&&) noexcept;

    [[nodiscard]] bool Start(const CameraConfig& config) noexcept;
    // The returned view is non-owning and remains valid only until the next
    // Capture(), Stop(), move-assignment, or destruction of this object.
    [[nodiscard]] CameraFrameView Capture() noexcept;
    void Stop() noexcept;
    [[nodiscard]] bool Running() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace ls2k::platform::true_ls2k0300

#endif
