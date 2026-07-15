#ifndef LS2K_PLATFORM_CAMERA_V4L2_TIMESTAMP_HPP
#define LS2K_PLATFORM_CAMERA_V4L2_TIMESTAMP_HPP

#include <cstdint>
#include <linux/videodev2.h>
#include <sys/time.h>

namespace ls2k::platform {

struct V4l2CaptureTimestampSelection {
    std::uint64_t capture_time_ms = 0;
    bool v4l2_timestamp_valid = false;
};

inline constexpr std::uint64_t kV4l2CaptureTimestampFutureSlackMs = 5;
inline constexpr std::uint64_t kV4l2CaptureTimestampMaxAgeMs = 1000;

inline bool IsV4l2CaptureTimestampPlausible(
    std::uint64_t capture_time_ms,
    std::uint64_t dequeue_time_ms) noexcept {
    if (capture_time_ms == 0 || dequeue_time_ms == 0) {
        return false;
    }
    if (capture_time_ms > dequeue_time_ms) {
        return capture_time_ms - dequeue_time_ms <=
               kV4l2CaptureTimestampFutureSlackMs;
    }
    return dequeue_time_ms - capture_time_ms <=
           kV4l2CaptureTimestampMaxAgeMs;
}

inline std::uint64_t V4l2TimevalToMs(const timeval& value) {
    if (value.tv_sec <= 0 && value.tv_usec <= 0) {
        return 0;
    }
    return static_cast<std::uint64_t>(value.tv_sec) * 1000U +
           static_cast<std::uint64_t>(value.tv_usec) / 1000U;
}

inline V4l2CaptureTimestampSelection SelectV4l2CaptureTimestamp(
    const timeval& timestamp,
    std::uint32_t flags,
    std::uint64_t dequeue_time_ms) {
    V4l2CaptureTimestampSelection selection{};
    selection.capture_time_ms = dequeue_time_ms;

    const std::uint64_t timestamp_ms = V4l2TimevalToMs(timestamp);
    const std::uint32_t timestamp_kind = flags & V4L2_BUF_FLAG_TIMESTAMP_MASK;
    if (timestamp_kind != V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC ||
        !IsV4l2CaptureTimestampPlausible(timestamp_ms, dequeue_time_ms)) {
        return selection;
    }

    selection.capture_time_ms = timestamp_ms;
    selection.v4l2_timestamp_valid = true;
    return selection;
}

}  // namespace ls2k::platform

#endif  // LS2K_PLATFORM_CAMERA_V4L2_TIMESTAMP_HPP
