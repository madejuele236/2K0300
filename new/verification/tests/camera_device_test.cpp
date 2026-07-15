#include "platform/true_ls2k0300/camera_device.hpp"

#include <cassert>
#include <cerrno>
#include <cstdint>
#include <initializer_list>
#include <linux/videodev2.h>
#include <sys/mman.h>

namespace {

struct DqReply {
    int rc = 0;
    int error = 0;
    unsigned index = 0;
    unsigned bytesused = 64;
    std::uint32_t sequence = 0;
    timeval timestamp{};
    std::uint32_t flags = 0;
};

struct FakeState {
    int opens = 0;
    int closes = 0;
    int maps = 0;
    int unmaps = 0;
    int stream_on = 0;
    int stream_off = 0;
    int release_buffers = 0;
    bool open_fails = false;
    unsigned long failing_ioctl = 0;
    std::uint32_t capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
    bool format_mismatch = false;
    unsigned requested_buffer_count = 2;
    bool mmap_fails = false;
    int poll_result = 1;
    short poll_revents = POLLIN;
    unsigned dq_index = 0;
    int dq_calls = 0;
    DqReply dq_replies[8]{};
    int dq_reply_count = 0;
    int dq_reply_position = 0;
    int qbuf_calls = 0;
    int fail_qbuf_call = 0;
    unsigned qbuf_indices[16]{};
    int qbuf_index_count = 0;
    bool short_buffer = false;
    std::uint64_t now_us = 5000000;
    std::uint8_t storage[2][64]{};
} g;

void Reset() { g = {}; }

void ScriptDq(std::initializer_list<DqReply> replies) {
    g.dq_reply_count = 0;
    g.dq_reply_position = 0;
    for (const DqReply& reply : replies) {
        g.dq_replies[g.dq_reply_count++] = reply;
    }
}

DqReply Ready(unsigned index,
              std::uint32_t sequence = 0,
              std::uint64_t timestamp_ms = 0,
              std::uint32_t flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
    DqReply reply{};
    reply.index = index;
    reply.sequence = sequence;
    reply.timestamp.tv_sec = static_cast<time_t>(timestamp_ms / 1000U);
    reply.timestamp.tv_usec = static_cast<suseconds_t>((timestamp_ms % 1000U) * 1000U);
    reply.flags = flags;
    return reply;
}

DqReply Error(int error) {
    DqReply reply{};
    reply.rc = -1;
    reply.error = error;
    return reply;
}

int Open(const char*, int, mode_t) {
    ++g.opens;
    return g.open_fails ? -1 : 20 + g.opens;
}
ssize_t Read(int, void*, std::size_t) { return -1; }
ssize_t Write(int, const void*, std::size_t) { return -1; }
off_t Seek(int, off_t, int) { return -1; }
int Close(int) { ++g.closes; return 0; }
int Poll(pollfd* descriptor, nfds_t, int) {
    descriptor->revents = g.poll_revents;
    return g.poll_result;
}

int Ioctl(int, unsigned long request, void* argument) {
    if (request == VIDIOC_QBUF) {
        ++g.qbuf_calls;
        auto* buffer = static_cast<v4l2_buffer*>(argument);
        if (g.qbuf_index_count < 16) {
            g.qbuf_indices[g.qbuf_index_count++] = buffer->index;
        }
        if (request == g.failing_ioctl || g.qbuf_calls == g.fail_qbuf_call) {
            errno = EIO;
            return -1;
        }
    } else if (request == g.failing_ioctl) {
        errno = EIO;
        return -1;
    }

    if (request == VIDIOC_QUERYCAP) {
        static_cast<v4l2_capability*>(argument)->capabilities = g.capabilities;
    } else if (request == VIDIOC_S_FMT) {
        auto* format = static_cast<v4l2_format*>(argument);
        format->fmt.pix.bytesperline = format->fmt.pix.width * 2;
        if (g.format_mismatch) {
            ++format->fmt.pix.width;
        }
    } else if (request == VIDIOC_REQBUFS) {
        auto* buffers = static_cast<v4l2_requestbuffers*>(argument);
        if (buffers->count == 0) {
            ++g.release_buffers;
        } else {
            buffers->count = g.requested_buffer_count;
        }
    } else if (request == VIDIOC_QUERYBUF) {
        auto* buffer = static_cast<v4l2_buffer*>(argument);
        buffer->length = 64;
        buffer->m.offset = buffer->index * 64;
    } else if (request == VIDIOC_DQBUF) {
        ++g.dq_calls;
        auto* buffer = static_cast<v4l2_buffer*>(argument);
        if (g.dq_reply_position < g.dq_reply_count) {
            const DqReply& reply = g.dq_replies[g.dq_reply_position++];
            if (reply.rc != 0) {
                errno = reply.error;
                return reply.rc;
            }
            buffer->index = reply.index;
            buffer->bytesused = reply.bytesused;
            buffer->sequence = reply.sequence;
            buffer->timestamp = reply.timestamp;
            buffer->flags = reply.flags;
        } else {
            buffer->index = g.dq_index++;
            buffer->bytesused = g.short_buffer ? 8 : 64;
        }
    } else if (request == VIDIOC_STREAMON) {
        ++g.stream_on;
    } else if (request == VIDIOC_STREAMOFF) {
        ++g.stream_off;
    }
    return 0;
}

void* Mmap(void*, std::size_t, int, int, int, std::int64_t offset) {
    ++g.maps;
    return g.mmap_fails ? MAP_FAILED : g.storage[offset / 64];
}
int Munmap(void*, std::size_t) { ++g.unmaps; return 0; }
std::uint64_t NowUs() {
    const std::uint64_t value = g.now_us;
    g.now_us += 1000;
    return value;
}

const ls2k::platform::linux_io::SyscallApi& Syscalls() {
    static const ls2k::platform::linux_io::SyscallApi api(
        {&Open, &Read, &Write, &Seek, &Close, &Poll});
    return api;
}

const ls2k::platform::true_ls2k0300::CameraSystemApi& System() {
    static const ls2k::platform::true_ls2k0300::CameraSystemApi api{
        &Ioctl, &Mmap, &Munmap, &NowUs};
    return api;
}

using ls2k::platform::true_ls2k0300::CameraCaptureStatus;
using ls2k::platform::true_ls2k0300::CameraConfig;
using ls2k::platform::true_ls2k0300::CameraDevice;
using ls2k::platform::true_ls2k0300::CameraLifecycleStage;
using ls2k::platform::true_ls2k0300::CameraStartStatus;

const CameraConfig kSingleConfig{"fake", 4, 8, 30, 2, 5, false};
const CameraConfig kDrainConfig{"fake", 4, 8, 30, 2, 5, true};

void ClearCaptureLogs() {
    g.dq_calls = 0;
    g.qbuf_index_count = 0;
}

void ExpectStartFailure(CameraStartStatus status, CameraLifecycleStage stage) {
    CameraDevice camera(Syscalls(), System());
    const auto result = camera.Start(kSingleConfig);
    assert(!result.ok());
    assert(result.status == status);
    assert(result.stage == stage);
    assert(!camera.Running());
}

void TestStartReportsActualMappedBufferCount() {
    Reset();
    CameraConfig requested_three = kSingleConfig;
    requested_three.buffer_count = 3;
    g.requested_buffer_count = 2;
    CameraDevice camera(Syscalls(), System());
    const auto start = camera.Start(requested_three);
    assert(start.ok());
    assert(start.mapped_buffer_count == 2);
    camera.Stop();
}

void TestTypedStartupFailures() {
    Reset();
    CameraDevice invalid(Syscalls(), System());
    CameraConfig invalid_config = kSingleConfig;
    invalid_config.buffer_count = 1;
    assert(invalid.Start(invalid_config).status == CameraStartStatus::kInvalidConfig);

    Reset(); g.open_fails = true;
    ExpectStartFailure(CameraStartStatus::kOpenFailed, CameraLifecycleStage::kOpen);
    Reset(); g.failing_ioctl = VIDIOC_QUERYCAP;
    ExpectStartFailure(CameraStartStatus::kQueryCapabilityFailed,
                       CameraLifecycleStage::kQueryCapability);
    Reset(); g.capabilities = V4L2_CAP_STREAMING;
    ExpectStartFailure(CameraStartStatus::kCaptureUnsupported,
                       CameraLifecycleStage::kQueryCapability);
    Reset(); g.capabilities = V4L2_CAP_VIDEO_CAPTURE;
    ExpectStartFailure(CameraStartStatus::kStreamingUnsupported,
                       CameraLifecycleStage::kQueryCapability);
    Reset(); g.failing_ioctl = VIDIOC_S_FMT;
    ExpectStartFailure(CameraStartStatus::kSetFormatFailed, CameraLifecycleStage::kSetFormat);

    Reset(); g.format_mismatch = true;
    CameraDevice mismatch(Syscalls(), System());
    const auto mismatch_result = mismatch.Start(kSingleConfig);
    assert(mismatch_result.status == CameraStartStatus::kNegotiatedFormatMismatch);
    assert(mismatch_result.negotiated.width == 5);

    Reset(); g.failing_ioctl = VIDIOC_REQBUFS;
    ExpectStartFailure(CameraStartStatus::kRequestBuffersFailed,
                       CameraLifecycleStage::kRequestBuffers);
    Reset(); g.requested_buffer_count = 1;
    ExpectStartFailure(CameraStartStatus::kInsufficientBuffers,
                       CameraLifecycleStage::kRequestBuffers);
    Reset(); g.failing_ioctl = VIDIOC_QUERYBUF;
    ExpectStartFailure(CameraStartStatus::kQueryBufferFailed,
                       CameraLifecycleStage::kQueryBuffer);
    Reset(); g.mmap_fails = true;
    ExpectStartFailure(CameraStartStatus::kMapBufferFailed, CameraLifecycleStage::kMapBuffer);
    Reset(); g.failing_ioctl = VIDIOC_QBUF;
    ExpectStartFailure(CameraStartStatus::kInitialQueueFailed,
                       CameraLifecycleStage::kInitialQueue);
    Reset(); g.failing_ioctl = VIDIOC_STREAMON;
    ExpectStartFailure(CameraStartStatus::kStreamOnFailed,
                       CameraLifecycleStage::kStreamOn);
}

void TestDrainDisabledDequeuesExactlyOnce() {
    Reset();
    CameraDevice camera(Syscalls(), System());
    assert(camera.Start(kSingleConfig).ok());
    ClearCaptureLogs();
    ScriptDq({Ready(0, 11, 5001), Error(EAGAIN)});
    const auto capture = camera.Capture();
    assert(capture.ok() && capture.frame.data == g.storage[0]);
    assert(g.dq_calls == 1 && capture.drained_buffer_count == 1);
    assert(capture.v4l2_sequence == 11);
    assert(capture.v4l2_timestamp_valid && capture.capture_time_ms == 5001);
    assert(capture.dequeue_time_ms == 5003);
    assert(capture.poll_wait_us == 1000 && capture.dequeue_us == 1000);
    assert(g.qbuf_index_count == 0);
    camera.Stop();
}

void TestDrainOneReadyThenEagain() {
    Reset();
    CameraDevice camera(Syscalls(), System());
    assert(camera.Start(kDrainConfig).ok());
    ClearCaptureLogs();
    ScriptDq({Ready(0, 21, 5001), Error(EAGAIN)});
    const auto capture = camera.Capture();
    assert(capture.ok() && capture.frame.data == g.storage[0]);
    assert(g.dq_calls == 2 && capture.drained_buffer_count == 1);
    assert(capture.v4l2_sequence == 21 && capture.v4l2_timestamp_valid);
    assert(capture.dequeue_us == 2000);
    assert(g.qbuf_index_count == 0);
    camera.Stop();
}

void TestDrainMultipleKeepsNewestAndRequeuesStale() {
    Reset();
    CameraDevice camera(Syscalls(), System());
    assert(camera.Start(kDrainConfig).ok());
    ClearCaptureLogs();
    ScriptDq({Ready(0, 31, 5001), Ready(1, 32, 0), Error(EIO)});
    const auto capture = camera.Capture();
    assert(capture.ok() && capture.frame.data == g.storage[1]);
    assert(g.dq_calls == 2);  // mapped-buffer bound prevents the scripted third dequeue.
    assert(capture.drained_buffer_count == 2);
    assert(g.qbuf_index_count == 1 && g.qbuf_indices[0] == 0);
    assert(capture.dequeue_us == 3000);
    assert(capture.v4l2_sequence == 32);
    assert(!capture.v4l2_timestamp_valid);
    assert(capture.dequeue_time_ms == 5004);
    assert(capture.capture_time_ms == capture.dequeue_time_ms);

    ScriptDq({Ready(0, 33, 5005), Error(EAGAIN)});
    const auto next = camera.Capture();
    assert(next.ok());
    assert(g.qbuf_indices[1] == 1);  // prior callback-held newest buffer requeued next.
    camera.Stop();
}

void TestDrainFollowupFailureDoesNotPublishSelectedFrame() {
    Reset();
    CameraDevice camera(Syscalls(), System());
    assert(camera.Start(kDrainConfig).ok());
    ClearCaptureLogs();
    ScriptDq({Ready(0, 41, 5001), Error(EIO)});
    const auto failed = camera.Capture();
    assert(!failed.ok() && !failed.frame.valid());
    assert(failed.status == CameraCaptureStatus::kDrainDequeueFailed);
    assert(failed.drained_buffer_count == 1 && g.dq_calls == 2);
    assert(failed.dequeue_us == 2000);
    assert(g.qbuf_index_count == 0);

    ScriptDq({Ready(1, 42, 5002), Error(EAGAIN)});
    const auto recovered = camera.Capture();
    assert(recovered.ok() && recovered.frame.data == g.storage[1]);
    assert(g.qbuf_indices[0] == 0);  // failed capture's selected buffer requeued first.
    camera.Stop();
}

void TestDrainRequeueFailuresStayVisible() {
    Reset();
    CameraDevice camera(Syscalls(), System());
    assert(camera.Start(kDrainConfig).ok());
    ClearCaptureLogs();
    ScriptDq({Ready(0), Ready(1)});
    g.fail_qbuf_call = g.qbuf_calls + 1;
    const auto stale_failure = camera.Capture();
    assert(!stale_failure.ok() && !stale_failure.frame.valid());
    assert(stale_failure.status == CameraCaptureStatus::kSupersededBufferRequeueFailed);
    assert(stale_failure.drained_buffer_count == 2);
    assert(stale_failure.dequeue_us == 4000);
    assert(g.qbuf_index_count == 2 && g.qbuf_indices[0] == 0 && g.qbuf_indices[1] == 1);
    assert(camera.Running());

    g.fail_qbuf_call = 0;
    ScriptDq({Ready(1), Error(EAGAIN)});
    assert(camera.Capture().ok());
    camera.Stop();

    Reset();
    CameraDevice stale_stop(Syscalls(), System());
    assert(stale_stop.Start(kDrainConfig).ok());
    ClearCaptureLogs();
    ScriptDq({Ready(0), Ready(1)});
    g.failing_ioctl = VIDIOC_QBUF;
    const auto stale_stop_failure = stale_stop.Capture();
    assert(!stale_stop_failure.ok() && !stale_stop_failure.frame.valid());
    assert(stale_stop_failure.status ==
           CameraCaptureStatus::kSupersededBufferRequeueFailed);
    assert(stale_stop_failure.drained_buffer_count == 2);
    assert(stale_stop_failure.dequeue_us == 4000);
    assert(g.qbuf_index_count == 2 && g.qbuf_indices[0] == 0 && g.qbuf_indices[1] == 1);
    assert(!stale_stop.Running() && g.stream_off == 1 && g.unmaps == 2);

    Reset();
    CameraDevice invalid(Syscalls(), System());
    assert(invalid.Start(kDrainConfig).ok());
    ClearCaptureLogs();
    ScriptDq({Ready(9)});
    g.fail_qbuf_call = g.qbuf_calls + 1;
    const auto invalid_failure = invalid.Capture();
    assert(!invalid_failure.ok() && !invalid_failure.frame.valid());
    assert(invalid_failure.status == CameraCaptureStatus::kInvalidBufferIndexRequeueFailed);
    assert(invalid_failure.drained_buffer_count == 1 && invalid_failure.dequeue_us == 2000);
    assert(g.qbuf_index_count == 1 && g.qbuf_indices[0] == 9);
    assert(!invalid.Running() && g.stream_off == 1 && g.unmaps == 2);

    Reset();
    CameraDevice invalid_mapping(Syscalls(), System());
    assert(invalid_mapping.Start(kDrainConfig).ok());
    ClearCaptureLogs();
    DqReply short_mapping = Ready(0);
    short_mapping.bytesused = 8;
    ScriptDq({short_mapping});
    const auto mapping_failure = invalid_mapping.Capture();
    assert(!mapping_failure.ok() && !mapping_failure.frame.valid());
    assert(mapping_failure.status == CameraCaptureStatus::kInvalidMapping);
    assert(mapping_failure.drained_buffer_count == 1 && mapping_failure.dequeue_us == 2000);
    assert(g.qbuf_index_count == 1 && g.qbuf_indices[0] == 0);
    assert(invalid_mapping.Running());
    invalid_mapping.Stop();
}

void TestTimestampTrustAndFallbackAtOwner() {
    struct Case {
        std::uint64_t timestamp_ms;
        std::uint32_t flags;
        bool trusted;
    };
    const Case cases[] = {
        {5001, V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC, true},
        {0, V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC, false},
        {5001, V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN, false},
        {6000, V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC, false},
        {3000, V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC, false},
    };
    for (const Case& item : cases) {
        Reset();
        CameraDevice camera(Syscalls(), System());
        assert(camera.Start(kSingleConfig).ok());
        ScriptDq({Ready(0, 77, item.timestamp_ms, item.flags)});
        const auto capture = camera.Capture();
        assert(capture.ok());
        assert(capture.v4l2_sequence == 77);
        assert(capture.v4l2_timestamp_valid == item.trusted);
        assert(capture.capture_time_ms ==
               (item.trusted ? item.timestamp_ms : capture.dequeue_time_ms));
        camera.Stop();
    }
}

void TestCaptureStatusesAndHeldLifecycle() {
    Reset();
    CameraDevice camera(Syscalls(), System());
    assert(camera.Start(kSingleConfig).ok());

    g.poll_result = 0;
    assert(camera.Capture().status == CameraCaptureStatus::kPollTimeout);
    g.poll_result = -1;
    assert(camera.Capture().status == CameraCaptureStatus::kPollFailed);
    g.poll_result = 1;
    g.poll_revents = POLLERR;
    assert(camera.Capture().status == CameraCaptureStatus::kPollUnexpectedEvents);

    g.poll_revents = POLLIN;
    g.failing_ioctl = VIDIOC_DQBUF;
    assert(camera.Capture().status == CameraCaptureStatus::kDequeueFailed);
    g.failing_ioctl = 0;
    ScriptDq({Ready(0)});
    assert(camera.Capture().ok());
    g.fail_qbuf_call = g.qbuf_calls + 1;
    assert(camera.Capture().status == CameraCaptureStatus::kRequeueHeldFailed);
    g.fail_qbuf_call = 0;
    ScriptDq({Ready(1)});
    assert(camera.Capture().ok());
    camera.Stop();
}

void TestPollErrorEventsTakePrecedenceOverReadable() {
    constexpr short kErrorEvents[] = {POLLERR, POLLHUP, POLLNVAL};
    for (const short error_event : kErrorEvents) {
        Reset();
        CameraDevice camera(Syscalls(), System());
        assert(camera.Start(kSingleConfig).ok());
        ClearCaptureLogs();
        g.poll_revents = static_cast<short>(POLLIN | error_event);
        const auto capture = camera.Capture();
        assert(capture.status == CameraCaptureStatus::kPollUnexpectedEvents);
        assert(g.dq_calls == 0);
        camera.Stop();
    }
}

}  // namespace

int main() {
    TestTypedStartupFailures();
    TestStartReportsActualMappedBufferCount();
    TestDrainDisabledDequeuesExactlyOnce();
    TestDrainOneReadyThenEagain();
    TestDrainMultipleKeepsNewestAndRequeuesStale();
    TestDrainFollowupFailureDoesNotPublishSelectedFrame();
    TestDrainRequeueFailuresStayVisible();
    TestTimestampTrustAndFallbackAtOwner();
    TestCaptureStatusesAndHeldLifecycle();
    TestPollErrorEventsTakePrecedenceOverReadable();
}
