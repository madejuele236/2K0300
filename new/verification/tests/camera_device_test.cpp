#include "platform/true_ls2k0300/camera_device.hpp"

#include <cassert>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <linux/videodev2.h>
#include <sys/mman.h>

namespace {

struct FakeState {
    int opens = 0;
    int closes = 0;
    int maps = 0;
    int unmaps = 0;
    int stream_on = 0;
    int stream_off = 0;
    int release_buffers = 0;
    unsigned dq_index = 0;
    std::uint8_t storage[2][64]{};
} g;

int Open(const char*, int, mode_t) { ++g.opens; return 20 + g.opens; }
ssize_t Read(int, void*, std::size_t) { return -1; }
ssize_t Write(int, const void*, std::size_t) { return -1; }
off_t Seek(int, off_t, int) { return -1; }
int Close(int) { ++g.closes; return 0; }
int Poll(pollfd* descriptor, nfds_t, int) {
    descriptor->revents = POLLIN;
    return 1;
}

int Ioctl(int, unsigned long request, void* argument) {
    if (request == VIDIOC_QUERYCAP) {
        auto* cap = static_cast<v4l2_capability*>(argument);
        cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
    } else if (request == VIDIOC_S_FMT) {
        auto* format = static_cast<v4l2_format*>(argument);
        format->fmt.pix.bytesperline = format->fmt.pix.width * 2;
    } else if (request == VIDIOC_REQBUFS) {
        auto* buffers = static_cast<v4l2_requestbuffers*>(argument);
        if (buffers->count == 0) ++g.release_buffers;
        else buffers->count = 2;
    } else if (request == VIDIOC_QUERYBUF) {
        auto* buffer = static_cast<v4l2_buffer*>(argument);
        buffer->length = 64;
        buffer->m.offset = buffer->index * 64;
    } else if (request == VIDIOC_DQBUF) {
        auto* buffer = static_cast<v4l2_buffer*>(argument);
        buffer->index = g.dq_index++ % 2;
        buffer->bytesused = 64;
    } else if (request == VIDIOC_STREAMON) {
        ++g.stream_on;
    } else if (request == VIDIOC_STREAMOFF) {
        ++g.stream_off;
    }
    return 0;
}

void* Mmap(void*, std::size_t, int, int, int, std::int64_t offset) {
    ++g.maps;
    return g.storage[offset / 64];
}
int Munmap(void*, std::size_t) { ++g.unmaps; return 0; }

}  // namespace

int main() {
    const ls2k::platform::linux_io::SyscallApi syscalls(
        {&Open, &Read, &Write, &Seek, &Close, &Poll});
    const ls2k::platform::true_ls2k0300::CameraSystemApi system{&Ioctl, &Mmap, &Munmap};
    ls2k::platform::true_ls2k0300::CameraDevice camera(syscalls, system);
    const ls2k::platform::true_ls2k0300::CameraConfig config{"fake", 4, 8, 30, 2, 5};

    assert(camera.Start(config));
    assert(camera.Running());
    const auto first = camera.Capture();
    assert(first.valid() && first.data == g.storage[0]);
    const auto second = camera.Capture();
    assert(second.valid() && second.data == g.storage[1]);
    camera.Stop();
    assert(!camera.Running());
    assert(g.opens == 1 && g.closes == 1);
    assert(g.maps == 2 && g.unmaps == 2);
    assert(g.stream_on == 1 && g.stream_off == 1 && g.release_buffers == 1);

    assert(camera.Start(config));
    assert(camera.Capture().valid());
    camera.Stop();
    assert(g.opens == 2 && g.closes == 2);
    assert(g.maps == 4 && g.unmaps == 4);
    assert(g.stream_on == 2 && g.stream_off == 2 && g.release_buffers == 2);
}
