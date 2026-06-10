#ifndef LS2K_PLATFORM_CAMERA_FRAME_SOURCE_V4L2_SELECTION_HPP
#define LS2K_PLATFORM_CAMERA_FRAME_SOURCE_V4L2_SELECTION_HPP

#include <cstddef>
#include <linux/videodev2.h>

namespace ls2k::platform::detail {

class IV4l2BufferRequeue {
public:
    virtual ~IV4l2BufferRequeue() = default;
    virtual void Requeue(v4l2_buffer buffer) = 0;
};

class DequeuedBufferGuard {
public:
    explicit DequeuedBufferGuard(IV4l2BufferRequeue& requeue) : requeue_(&requeue) {}
    DequeuedBufferGuard(const DequeuedBufferGuard&) = delete;
    DequeuedBufferGuard& operator=(const DequeuedBufferGuard&) = delete;
    ~DequeuedBufferGuard() { Requeue(); }

    bool Active() const { return active_; }
    const v4l2_buffer& Buffer() const { return buffer_; }

    void Reset(const v4l2_buffer& buffer) {
        Requeue();
        buffer_ = buffer;
        active_ = true;
    }

    void Requeue() {
        if (!active_) {
            return;
        }
        requeue_->Requeue(buffer_);
        active_ = false;
    }

private:
    IV4l2BufferRequeue* requeue_ = nullptr;
    v4l2_buffer buffer_{};
    bool active_ = false;
};

struct V4l2DequeuedSelection {
    bool has_selected = false;
    v4l2_buffer selected{};
};

inline V4l2DequeuedSelection SelectNewestDequeuedBufferForProcessing(
    const v4l2_buffer& buffer,
    std::size_t buffer_count,
    DequeuedBufferGuard& selected) {
    V4l2DequeuedSelection result{};
    if (buffer.index >= buffer_count) {
        return result;
    }
    selected.Reset(buffer);
    result.has_selected = true;
    result.selected = selected.Buffer();
    return result;
}

}  // namespace ls2k::platform::detail

#endif  // LS2K_PLATFORM_CAMERA_FRAME_SOURCE_V4L2_SELECTION_HPP
