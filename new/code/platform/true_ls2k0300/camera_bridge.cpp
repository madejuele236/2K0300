#include "platform/true_ls2k0300/camera_device.hpp"

#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <new>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <utility>
#include <vector>

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

}  // namespace

const CameraSystemApi& ProductionCameraSystemApi() noexcept {
    static const CameraSystemApi api{&ProductionIoctl, &ProductionMmap, &ProductionMunmap};
    return api;
}

struct CameraDevice::Impl final {
    struct Mapping final {
        void* address = nullptr;
        std::size_t length = 0;
    };

    Impl(const linux_io::SyscallApi& syscall_api, const CameraSystemApi& system_api) noexcept
        : syscalls(syscall_api), system(system_api), fd(-1, syscall_api) {}

    bool Ioctl(unsigned long request, void* argument) noexcept {
        int rc;
        do {
            rc = system.ioctl(fd.get(), request, argument);
        } while (rc < 0 && errno == EINTR);
        return rc == 0;
    }

    bool RequeueHeld() noexcept {
        if (!held) {
            return true;
        }
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = held_index;
        held = false;
        return Ioctl(VIDIOC_QBUF, &buffer);
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
    }

    bool Start(const CameraConfig& config) noexcept {
        Stop();
        if (config.device == nullptr || config.width <= 0 || config.height <= 0 ||
            config.fps <= 0 || config.buffer_count < 2 || config.timeout_ms < 0 ||
            system.ioctl == nullptr || system.mmap == nullptr || system.munmap == nullptr) {
            return false;
        }

        const int opened = syscalls.functions().open(config.device, O_RDWR | O_NONBLOCK, 0);
        if (opened < 0) {
            return false;
        }
        static_cast<void>(fd.Reset(opened));

        v4l2_capability capability{};
        if (!Ioctl(VIDIOC_QUERYCAP, &capability) ||
            (capability.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0 ||
            (capability.capabilities & V4L2_CAP_STREAMING) == 0) {
            Stop();
            return false;
        }

        v4l2_format format{};
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = static_cast<__u32>(config.width);
        format.fmt.pix.height = static_cast<__u32>(config.height);
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        format.fmt.pix.field = V4L2_FIELD_ANY;
        if (!Ioctl(VIDIOC_S_FMT, &format) || format.fmt.pix.width != static_cast<__u32>(config.width) ||
            format.fmt.pix.height != static_cast<__u32>(config.height) ||
            format.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV ||
            format.fmt.pix.bytesperline < static_cast<__u32>(config.width * 2)) {
            Stop();
            return false;
        }
        width = config.width;
        height = config.height;
        stride = static_cast<int>(format.fmt.pix.bytesperline);
        timeout_ms = config.timeout_ms;

        v4l2_streamparm parameters{};
        parameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parameters.parm.capture.timeperframe.numerator = 1;
        parameters.parm.capture.timeperframe.denominator = static_cast<__u32>(config.fps);
        static_cast<void>(Ioctl(VIDIOC_S_PARM, &parameters));

        v4l2_requestbuffers request{};
        request.count = static_cast<__u32>(config.buffer_count);
        request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        request.memory = V4L2_MEMORY_MMAP;
        if (!Ioctl(VIDIOC_REQBUFS, &request) || request.count < 2) {
            Stop();
            return false;
        }
        try {
            mappings.resize(request.count);
        } catch (...) {
            Stop();
            return false;
        }

        for (std::size_t index = 0; index < mappings.size(); ++index) {
            v4l2_buffer buffer{};
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = static_cast<__u32>(index);
            if (!Ioctl(VIDIOC_QUERYBUF, &buffer)) {
                Stop();
                return false;
            }
            void* address = system.mmap(nullptr,
                                        buffer.length,
                                        PROT_READ | PROT_WRITE,
                                        MAP_SHARED,
                                        fd.get(),
                                        buffer.m.offset);
            if (address == MAP_FAILED) {
                Stop();
                return false;
            }
            mappings[index] = {address, buffer.length};
            if (!Ioctl(VIDIOC_QBUF, &buffer)) {
                Stop();
                return false;
            }
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (!Ioctl(VIDIOC_STREAMON, &type)) {
            Stop();
            return false;
        }
        streaming = true;
        return true;
    }

    CameraFrameView Capture() noexcept {
        if (!streaming || !RequeueHeld()) {
            return {};
        }
        pollfd descriptor{};
        descriptor.fd = fd.get();
        descriptor.events = POLLIN;
        const int poll_rc = syscalls.functions().poll(&descriptor, 1, timeout_ms);
        if (poll_rc <= 0 || (descriptor.revents & POLLIN) == 0) {
            return {};
        }

        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        if (!Ioctl(VIDIOC_DQBUF, &buffer)) {
            return {};
        }
        if (buffer.index >= mappings.size()) {
            static_cast<void>(Ioctl(VIDIOC_QBUF, &buffer));
            return {};
        }
        held = true;
        held_index = buffer.index;
        const Mapping& mapping = mappings[buffer.index];
        const std::size_t minimum = static_cast<std::size_t>(height) * static_cast<std::size_t>(stride);
        const std::size_t available = buffer.bytesused == 0 ? mapping.length : buffer.bytesused;
        if (mapping.address == nullptr || mapping.length < minimum || available < minimum) {
            return {};
        }
        return {static_cast<const std::uint8_t*>(mapping.address), available, width, height, stride};
    }

    const linux_io::SyscallApi& syscalls;
    const CameraSystemApi& system;
    linux_io::UniqueFd fd;
    std::vector<Mapping> mappings;
    int width = 0;
    int height = 0;
    int stride = 0;
    int timeout_ms = 0;
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

bool CameraDevice::Start(const CameraConfig& config) noexcept {
    return impl_ != nullptr && impl_->Start(config);
}

CameraFrameView CameraDevice::Capture() noexcept {
    return impl_ == nullptr ? CameraFrameView{} : impl_->Capture();
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
