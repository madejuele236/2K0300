#include "platform/true_ls2k0300/camera_device.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <new>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <utility>
#include <vector>

#include "platform/camera_v4l2_timestamp.hpp"

namespace ls2k::platform::true_ls2k0300 {
namespace {

int ProductionIoctl(int fd, unsigned long request, void* argument) {
    return ::ioctl(fd, request, argument);
}

void* ProductionMmap(
    void* address, std::size_t length, int protection, int flags, int fd, std::int64_t offset) {
    return ::mmap(address, length, protection, flags, fd, static_cast<off_t>(offset));
}

int ProductionMunmap(void* address, std::size_t length) {
    return ::munmap(address, length);
}

std::uint64_t ProductionNowUs() {
    using namespace std::chrono;
    return static_cast<std::uint64_t>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

}  // namespace

const CameraSystemApi& ProductionCameraSystemApi() noexcept {
    static const CameraSystemApi api{
        &ProductionIoctl, &ProductionMmap, &ProductionMunmap, &ProductionNowUs};
    return api;
}

const char* CameraLifecycleStageCode(CameraLifecycleStage stage) noexcept {
    switch (stage) {
    case CameraLifecycleStage::kNone: return "none";
    case CameraLifecycleStage::kConfig: return "config";
    case CameraLifecycleStage::kOpen: return "open";
    case CameraLifecycleStage::kQueryCapability: return "querycap";
    case CameraLifecycleStage::kSetFormat: return "set_format";
    case CameraLifecycleStage::kRequestBuffers: return "request_buffers";
    case CameraLifecycleStage::kQueryBuffer: return "query_buffer";
    case CameraLifecycleStage::kMapBuffer: return "map_buffer";
    case CameraLifecycleStage::kInitialQueue: return "initial_queue";
    case CameraLifecycleStage::kStreamOn: return "stream_on";
    case CameraLifecycleStage::kRequeueHeld: return "requeue_held";
    case CameraLifecycleStage::kPoll: return "poll";
    case CameraLifecycleStage::kDequeue: return "dequeue";
    case CameraLifecycleStage::kValidateBuffer: return "validate_buffer";
    case CameraLifecycleStage::kValidateMapping: return "validate_mapping";
    case CameraLifecycleStage::kRequeueRejected: return "requeue_rejected";
    }
    return "unknown";
}

const char* CameraStartStatusCode(CameraStartStatus status) noexcept {
    switch (status) {
    case CameraStartStatus::kStarted: return "started";
    case CameraStartStatus::kDeviceStateUnavailable: return "device_state_unavailable";
    case CameraStartStatus::kInvalidConfig: return "invalid_config";
    case CameraStartStatus::kOpenFailed: return "open_failed";
    case CameraStartStatus::kQueryCapabilityFailed: return "querycap_failed";
    case CameraStartStatus::kCaptureUnsupported: return "capture_unsupported";
    case CameraStartStatus::kStreamingUnsupported: return "streaming_unsupported";
    case CameraStartStatus::kSetFormatFailed: return "set_format_failed";
    case CameraStartStatus::kNegotiatedFormatMismatch: return "negotiated_format_mismatch";
    case CameraStartStatus::kRequestBuffersFailed: return "request_buffers_failed";
    case CameraStartStatus::kInsufficientBuffers: return "insufficient_buffers";
    case CameraStartStatus::kMappingAllocationFailed: return "mapping_allocation_failed";
    case CameraStartStatus::kQueryBufferFailed: return "query_buffer_failed";
    case CameraStartStatus::kMapBufferFailed: return "map_buffer_failed";
    case CameraStartStatus::kInitialQueueFailed: return "initial_queue_failed";
    case CameraStartStatus::kStreamOnFailed: return "stream_on_failed";
    }
    return "unknown";
}

const char* CameraCaptureStatusCode(CameraCaptureStatus status) noexcept {
    switch (status) {
    case CameraCaptureStatus::kFrameReady: return "frame_ready";
    case CameraCaptureStatus::kNotRunning: return "not_running";
    case CameraCaptureStatus::kRequeueHeldFailed: return "requeue_held_failed";
    case CameraCaptureStatus::kPollTimeout: return "poll_timeout";
    case CameraCaptureStatus::kPollFailed: return "poll_failed";
    case CameraCaptureStatus::kPollUnexpectedEvents: return "poll_unexpected_events";
    case CameraCaptureStatus::kDequeueFailed: return "dequeue_failed";
    case CameraCaptureStatus::kDrainDequeueFailed: return "drain_dequeue_failed";
    case CameraCaptureStatus::kSupersededBufferRequeueFailed:
        return "superseded_buffer_requeue_failed";
    case CameraCaptureStatus::kInvalidBufferIndex: return "invalid_buffer_index";
    case CameraCaptureStatus::kInvalidBufferIndexRequeueFailed:
        return "invalid_buffer_index_requeue_failed";
    case CameraCaptureStatus::kInvalidMapping: return "invalid_mapping";
    case CameraCaptureStatus::kInvalidMappingRequeueFailed:
        return "invalid_mapping_requeue_failed";
    }
    return "unknown";
}

struct CameraDevice::Impl final {
    struct Mapping final {
        void* address = nullptr;
        std::size_t length = 0;
    };

    struct IoctlOutcome final {
        bool ok = false;
        int error = 0;
    };

    Impl(const linux_io::SyscallApi& syscall_api, const CameraSystemApi& system_api) noexcept
        : syscalls(syscall_api), system(system_api), fd(-1, syscall_api) {}

    IoctlOutcome IoctlWithError(unsigned long request, void* argument) noexcept {
        int rc;
        do {
            errno = 0;
            rc = system.ioctl(fd.get(), request, argument);
        } while (rc < 0 && errno == EINTR);
        return {rc == 0, rc == 0 ? 0 : errno};
    }

    bool Ioctl(unsigned long request, void* argument) noexcept {
        return IoctlWithError(request, argument).ok;
    }

    bool RequeueHeld() noexcept {
        if (!held) {
            return true;
        }
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = held_index;
        if (!Ioctl(VIDIOC_QBUF, &buffer)) {
            return false;
        }
        held = false;
        return true;
    }

    std::uint64_t NowUs() const noexcept {
        return system.now_us == nullptr ? ProductionNowUs() : system.now_us();
    }

    void Stop() noexcept {
        held = false;
        if (fd) {
            if (streaming) {
                v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                static_cast<void>(Ioctl(VIDIOC_STREAMOFF, &type));
            }
            streaming = false;
            for (Mapping& mapping : mappings) {
                if (mapping.address != nullptr && mapping.address != MAP_FAILED) {
                    static_cast<void>(system.munmap(mapping.address, mapping.length));
                }
            }
            mappings.clear();
            v4l2_requestbuffers release{};
            release.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            release.memory = V4L2_MEMORY_MMAP;
            release.count = 0;
            static_cast<void>(Ioctl(VIDIOC_REQBUFS, &release));
            static_cast<void>(fd.Reset());
        } else {
            mappings.clear();
        }
        width = 0;
        height = 0;
        stride = 0;
        timeout_ms = 0;
        drain_ready_buffers = false;
    }

    CameraStartResult Start(const CameraConfig& config) noexcept {
        Stop();
        if (config.device == nullptr || config.width <= 0 || config.height <= 0 ||
            config.fps <= 0 || config.buffer_count < 2 || config.timeout_ms < 0 ||
            system.ioctl == nullptr || system.mmap == nullptr || system.munmap == nullptr) {
            return {CameraStartStatus::kInvalidConfig, CameraLifecycleStage::kConfig, {}};
        }

        const int opened = syscalls.functions().open(config.device, O_RDWR | O_NONBLOCK, 0);
        if (opened < 0) {
            return {CameraStartStatus::kOpenFailed, CameraLifecycleStage::kOpen, {}};
        }
        static_cast<void>(fd.Reset(opened));

        v4l2_capability capability{};
        if (!Ioctl(VIDIOC_QUERYCAP, &capability)) {
            Stop();
            return {CameraStartStatus::kQueryCapabilityFailed,
                    CameraLifecycleStage::kQueryCapability,
                    {}};
        }
        const __u32 effective_capabilities =
            (capability.capabilities & V4L2_CAP_DEVICE_CAPS) != 0
                ? capability.device_caps
                : capability.capabilities;
        if ((effective_capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0) {
            Stop();
            return {CameraStartStatus::kCaptureUnsupported,
                    CameraLifecycleStage::kQueryCapability,
                    {}};
        }
        if ((effective_capabilities & V4L2_CAP_STREAMING) == 0) {
            Stop();
            return {CameraStartStatus::kStreamingUnsupported,
                    CameraLifecycleStage::kQueryCapability,
                    {}};
        }

        v4l2_format format{};
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = static_cast<__u32>(config.width);
        format.fmt.pix.height = static_cast<__u32>(config.height);
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        format.fmt.pix.field = V4L2_FIELD_ANY;
        if (!Ioctl(VIDIOC_S_FMT, &format)) {
            Stop();
            return {CameraStartStatus::kSetFormatFailed, CameraLifecycleStage::kSetFormat, {}};
        }
        const CameraNegotiatedFormat negotiated{
            static_cast<int>(format.fmt.pix.width),
            static_cast<int>(format.fmt.pix.height),
            static_cast<int>(format.fmt.pix.bytesperline),
            format.fmt.pix.pixelformat};
        if (format.fmt.pix.width != static_cast<__u32>(config.width) ||
            format.fmt.pix.height != static_cast<__u32>(config.height) ||
            format.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV ||
            format.fmt.pix.bytesperline < static_cast<__u32>(config.width * 2)) {
            Stop();
            return {CameraStartStatus::kNegotiatedFormatMismatch,
                    CameraLifecycleStage::kSetFormat,
                    negotiated};
        }
        width = config.width;
        height = config.height;
        stride = static_cast<int>(format.fmt.pix.bytesperline);
        timeout_ms = config.timeout_ms;
        drain_ready_buffers = config.drain_ready_buffers;

        v4l2_streamparm parameters{};
        parameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parameters.parm.capture.timeperframe.numerator = 1;
        parameters.parm.capture.timeperframe.denominator = static_cast<__u32>(config.fps);
        static_cast<void>(Ioctl(VIDIOC_S_PARM, &parameters));

        v4l2_requestbuffers request{};
        request.count = static_cast<__u32>(config.buffer_count);
        request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        request.memory = V4L2_MEMORY_MMAP;
        if (!Ioctl(VIDIOC_REQBUFS, &request)) {
            Stop();
            return {CameraStartStatus::kRequestBuffersFailed,
                    CameraLifecycleStage::kRequestBuffers,
                    negotiated};
        }
        if (request.count < 2) {
            Stop();
            return {CameraStartStatus::kInsufficientBuffers,
                    CameraLifecycleStage::kRequestBuffers,
                    negotiated};
        }
        try {
            mappings.resize(request.count);
        } catch (...) {
            Stop();
            return {CameraStartStatus::kMappingAllocationFailed,
                    CameraLifecycleStage::kRequestBuffers,
                    negotiated};
        }

        for (std::size_t index = 0; index < mappings.size(); ++index) {
            v4l2_buffer buffer{};
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = static_cast<__u32>(index);
            if (!Ioctl(VIDIOC_QUERYBUF, &buffer)) {
                Stop();
                return {CameraStartStatus::kQueryBufferFailed,
                        CameraLifecycleStage::kQueryBuffer,
                        negotiated};
            }
            void* address = system.mmap(nullptr,
                                        buffer.length,
                                        PROT_READ | PROT_WRITE,
                                        MAP_SHARED,
                                        fd.get(),
                                        buffer.m.offset);
            if (address == MAP_FAILED) {
                Stop();
                return {CameraStartStatus::kMapBufferFailed,
                        CameraLifecycleStage::kMapBuffer,
                        negotiated};
            }
            mappings[index] = {address, buffer.length};
            if (!Ioctl(VIDIOC_QBUF, &buffer)) {
                Stop();
                return {CameraStartStatus::kInitialQueueFailed,
                        CameraLifecycleStage::kInitialQueue,
                        negotiated};
            }
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (!Ioctl(VIDIOC_STREAMON, &type)) {
            Stop();
            return {CameraStartStatus::kStreamOnFailed,
                    CameraLifecycleStage::kStreamOn,
                    negotiated};
        }
        streaming = true;
        return {CameraStartStatus::kStarted,
                CameraLifecycleStage::kStreamOn,
                negotiated,
                mappings.size()};
    }

    CameraCaptureResult Capture() noexcept {
        if (!streaming) {
            return {CameraCaptureStatus::kNotRunning, CameraLifecycleStage::kNone};
        }
        if (!RequeueHeld()) {
            return {CameraCaptureStatus::kRequeueHeldFailed,
                    CameraLifecycleStage::kRequeueHeld};
        }
        pollfd descriptor{};
        descriptor.fd = fd.get();
        descriptor.events = POLLIN;
        const std::uint64_t poll_begin_us = NowUs();
        const int poll_rc = syscalls.functions().poll(&descriptor, 1, timeout_ms);
        const std::uint64_t poll_wait_us = NowUs() - poll_begin_us;
        if (poll_rc == 0) {
            CameraCaptureResult result{};
            result.status = CameraCaptureStatus::kPollTimeout;
            result.stage = CameraLifecycleStage::kPoll;
            result.poll_wait_us = poll_wait_us;
            return result;
        }
        if (poll_rc < 0) {
            CameraCaptureResult result{};
            result.status = CameraCaptureStatus::kPollFailed;
            result.stage = CameraLifecycleStage::kPoll;
            result.poll_wait_us = poll_wait_us;
            return result;
        }
        constexpr short kPollErrorEvents = POLLERR | POLLHUP | POLLNVAL;
        if ((descriptor.revents & kPollErrorEvents) != 0 ||
            (descriptor.revents & POLLIN) == 0) {
            CameraCaptureResult result{};
            result.status = CameraCaptureStatus::kPollUnexpectedEvents;
            result.stage = CameraLifecycleStage::kPoll;
            result.poll_wait_us = poll_wait_us;
            return result;
        }

        std::uint64_t dequeue_begin_us = 0;
        std::uint64_t dequeue_end_us = 0;
        struct SelectedBuffer final {
            v4l2_buffer buffer{};
            const Mapping* mapping = nullptr;
            std::size_t available = 0;
            std::uint64_t dequeue_time_ms = 0;
            bool active = false;
        } selected;
        int drained_buffer_count = 0;
        const std::size_t dequeue_bound = drain_ready_buffers ? mappings.size() : 1U;
        for (std::size_t attempt = 0; attempt < dequeue_bound; ++attempt) {
            v4l2_buffer buffer{};
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = V4L2_MEMORY_MMAP;
            if (attempt == 0) {
                dequeue_begin_us = NowUs();
            }
            const IoctlOutcome dequeue = IoctlWithError(VIDIOC_DQBUF, &buffer);
            dequeue_end_us = NowUs();
            const std::uint64_t current_dequeue_end_us = dequeue_end_us;
            if (!dequeue.ok) {
                if (selected.active && drain_ready_buffers && dequeue.error == EAGAIN) {
                    break;
                }
                if (selected.active) {
                    held = true;
                    held_index = selected.buffer.index;
                }
                CameraCaptureResult result{};
                result.status = selected.active ? CameraCaptureStatus::kDrainDequeueFailed
                                                : CameraCaptureStatus::kDequeueFailed;
                result.stage = CameraLifecycleStage::kDequeue;
                result.poll_wait_us = poll_wait_us;
                result.dequeue_us = dequeue_end_us - dequeue_begin_us;
                result.drained_buffer_count = drained_buffer_count;
                return result;
            }
            ++drained_buffer_count;

            if (buffer.index >= mappings.size()) {
                const bool requeued = Ioctl(VIDIOC_QBUF, &buffer);
                dequeue_end_us = NowUs();
                if (selected.active) {
                    held = true;
                    held_index = selected.buffer.index;
                }
                if (!requeued) {
                    Stop();
                }
                CameraCaptureResult result{};
                result.status = requeued ? CameraCaptureStatus::kInvalidBufferIndex
                                         : CameraCaptureStatus::kInvalidBufferIndexRequeueFailed;
                result.stage = requeued ? CameraLifecycleStage::kValidateBuffer
                                        : CameraLifecycleStage::kRequeueRejected;
                result.poll_wait_us = poll_wait_us;
                result.dequeue_us = dequeue_end_us - dequeue_begin_us;
                result.drained_buffer_count = drained_buffer_count;
                return result;
            }

            const Mapping& mapping = mappings[buffer.index];
            const std::size_t minimum =
                static_cast<std::size_t>(height) * static_cast<std::size_t>(stride);
            const std::size_t available = buffer.bytesused == 0 ? mapping.length : buffer.bytesused;
            if (mapping.address == nullptr || mapping.length < minimum || available < minimum) {
                const bool requeued = Ioctl(VIDIOC_QBUF, &buffer);
                dequeue_end_us = NowUs();
                if (selected.active) {
                    held = true;
                    held_index = selected.buffer.index;
                }
                if (!requeued) {
                    Stop();
                }
                CameraCaptureResult result{};
                result.status = requeued ? CameraCaptureStatus::kInvalidMapping
                                         : CameraCaptureStatus::kInvalidMappingRequeueFailed;
                result.stage = requeued ? CameraLifecycleStage::kValidateMapping
                                        : CameraLifecycleStage::kRequeueRejected;
                result.poll_wait_us = poll_wait_us;
                result.dequeue_us = dequeue_end_us - dequeue_begin_us;
                result.drained_buffer_count = drained_buffer_count;
                return result;
            }

            if (selected.active) {
                const bool superseded_requeued = Ioctl(VIDIOC_QBUF, &selected.buffer);
                dequeue_end_us = NowUs();
                if (!superseded_requeued) {
                    const bool current_requeued = Ioctl(VIDIOC_QBUF, &buffer);
                    dequeue_end_us = NowUs();
                    if (current_requeued) {
                        held = true;
                        held_index = selected.buffer.index;
                    } else {
                        Stop();
                    }
                    CameraCaptureResult result{};
                    result.status = CameraCaptureStatus::kSupersededBufferRequeueFailed;
                    result.stage = CameraLifecycleStage::kRequeueRejected;
                    result.poll_wait_us = poll_wait_us;
                    result.dequeue_us = dequeue_end_us - dequeue_begin_us;
                    result.drained_buffer_count = drained_buffer_count;
                    return result;
                }
            }

            selected.buffer = buffer;
            selected.mapping = &mapping;
            selected.available = available;
            selected.dequeue_time_ms = current_dequeue_end_us / 1000U;
            selected.active = true;
        }

        held = true;
        held_index = selected.buffer.index;
        const V4l2CaptureTimestampSelection timestamp =
            SelectV4l2CaptureTimestamp(selected.buffer.timestamp,
                                       selected.buffer.flags,
                                       selected.dequeue_time_ms);
        CameraCaptureResult result{};
        result.status = CameraCaptureStatus::kFrameReady;
        result.stage = CameraLifecycleStage::kDequeue;
        result.frame = {static_cast<const std::uint8_t*>(selected.mapping->address),
                        selected.available,
                        width,
                        height,
                        stride};
        result.capture_time_ms = timestamp.capture_time_ms;
        result.dequeue_time_ms = selected.dequeue_time_ms;
        result.v4l2_sequence = selected.buffer.sequence;
        result.v4l2_timestamp_valid = timestamp.v4l2_timestamp_valid;
        result.poll_wait_us = poll_wait_us;
        result.dequeue_us = dequeue_end_us - dequeue_begin_us;
        result.drained_buffer_count = drained_buffer_count;
        return result;
    }

    const linux_io::SyscallApi& syscalls;
    const CameraSystemApi& system;
    linux_io::UniqueFd fd;
    std::vector<Mapping> mappings;
    int width = 0;
    int height = 0;
    int stride = 0;
    int timeout_ms = 0;
    bool drain_ready_buffers = false;
    bool streaming = false;
    bool held = false;
    __u32 held_index = 0;
};

CameraDevice::CameraDevice(
    const linux_io::SyscallApi& syscalls, const CameraSystemApi& system) noexcept
    : impl_(new (std::nothrow) Impl(syscalls, system)) {}

CameraDevice::~CameraDevice() noexcept {
    Stop();
}

CameraDevice::CameraDevice(CameraDevice&&) noexcept = default;
CameraDevice& CameraDevice::operator=(CameraDevice&& other) noexcept {
    if (this != &other) {
        Stop();
        impl_ = std::move(other.impl_);
    }
    return *this;
}

CameraStartResult CameraDevice::Start(const CameraConfig& config) noexcept {
    return impl_ == nullptr
               ? CameraStartResult{CameraStartStatus::kDeviceStateUnavailable,
                                   CameraLifecycleStage::kNone,
                                   {}}
               : impl_->Start(config);
}

CameraCaptureResult CameraDevice::Capture() noexcept {
    return impl_ == nullptr
               ? CameraCaptureResult{CameraCaptureStatus::kNotRunning,
                                     CameraLifecycleStage::kNone}
               : impl_->Capture();
}

void CameraDevice::Stop() noexcept {
    if (impl_ != nullptr) {
        impl_->Stop();
    }
}

bool CameraDevice::Running() const noexcept {
    return impl_ != nullptr && impl_->streaming;
}

}  // namespace ls2k::platform::true_ls2k0300
