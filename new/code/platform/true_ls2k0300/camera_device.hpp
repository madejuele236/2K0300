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
    bool drain_ready_buffers = true;
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

enum class CameraLifecycleStage {
    kNone,
    kConfig,
    kOpen,
    kQueryCapability,
    kSetFormat,
    kRequestBuffers,
    kQueryBuffer,
    kMapBuffer,
    kInitialQueue,
    kStreamOn,
    kRequeueHeld,
    kPoll,
    kDequeue,
    kValidateBuffer,
    kValidateMapping,
    kRequeueRejected,
};

enum class CameraStartStatus {
    kStarted,
    kDeviceStateUnavailable,
    kInvalidConfig,
    kOpenFailed,
    kQueryCapabilityFailed,
    kCaptureUnsupported,
    kStreamingUnsupported,
    kSetFormatFailed,
    kNegotiatedFormatMismatch,
    kRequestBuffersFailed,
    kInsufficientBuffers,
    kMappingAllocationFailed,
    kQueryBufferFailed,
    kMapBufferFailed,
    kInitialQueueFailed,
    kStreamOnFailed,
};

enum class CameraCaptureStatus {
    kFrameReady,
    kNotRunning,
    kRequeueHeldFailed,
    kPollTimeout,
    kPollFailed,
    kPollUnexpectedEvents,
    kDequeueFailed,
    kDrainDequeueFailed,
    kSupersededBufferRequeueFailed,
    kInvalidBufferIndex,
    kInvalidBufferIndexRequeueFailed,
    kInvalidMapping,
    kInvalidMappingRequeueFailed,
};

struct CameraNegotiatedFormat final {
    int width = 0;
    int height = 0;
    int stride = 0;
    std::uint32_t pixel_format = 0;
};

struct CameraStartResult final {
    CameraStartStatus status = CameraStartStatus::kInvalidConfig;
    CameraLifecycleStage stage = CameraLifecycleStage::kConfig;
    CameraNegotiatedFormat negotiated{};
    std::size_t mapped_buffer_count = 0;

    [[nodiscard]] bool ok() const noexcept { return status == CameraStartStatus::kStarted; }
};

struct CameraCaptureResult final {
    CameraCaptureStatus status = CameraCaptureStatus::kNotRunning;
    CameraLifecycleStage stage = CameraLifecycleStage::kNone;
    CameraFrameView frame{};
    std::uint64_t capture_time_ms = 0;
    std::uint64_t dequeue_time_ms = 0;
    std::uint32_t v4l2_sequence = 0;
    bool v4l2_timestamp_valid = false;
    std::uint64_t poll_wait_us = 0;
    std::uint64_t dequeue_us = 0;
    int drained_buffer_count = 0;

    [[nodiscard]] bool ok() const noexcept {
        return status == CameraCaptureStatus::kFrameReady && frame.valid();
    }
};

[[nodiscard]] const char* CameraLifecycleStageCode(CameraLifecycleStage stage) noexcept;
[[nodiscard]] const char* CameraStartStatusCode(CameraStartStatus status) noexcept;
[[nodiscard]] const char* CameraCaptureStatusCode(CameraCaptureStatus status) noexcept;

// V4L2 operations not covered by the generic Linux syscall mechanism. Tests
// inject this table together with SyscallApi; production binds it once.
struct CameraSystemApi final {
    using IoctlFn = int (*)(int, unsigned long, void*);
    using MmapFn = void* (*)(void*, std::size_t, int, int, int, std::int64_t);
    using MunmapFn = int (*)(void*, std::size_t);
    using NowUsFn = std::uint64_t (*)();

    IoctlFn ioctl = nullptr;
    MmapFn mmap = nullptr;
    MunmapFn munmap = nullptr;
    NowUsFn now_us = nullptr;
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

    [[nodiscard]] CameraStartResult Start(const CameraConfig& config) noexcept;
    // The returned view is non-owning and remains valid only until the next
    // Capture(), Stop(), move-assignment, or destruction of this object.
    [[nodiscard]] CameraCaptureResult Capture() noexcept;
    void Stop() noexcept;
    [[nodiscard]] bool Running() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace ls2k::platform::true_ls2k0300

#endif
