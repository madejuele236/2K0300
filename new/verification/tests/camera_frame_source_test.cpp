#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "platform/camera_frame_source_v4l2_selection.hpp"
#include "port/camera_frame_source.hpp"

namespace {

void Expect(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class OwningFallbackSource final : public ls2k::port::ICameraFrameSource {
public:
    bool Start(const ls2k::port::CameraSourceParameters&,
               ls2k::port::DiagnosticSink&) override {
        ready_ = true;
        return true;
    }

    void Stop(ls2k::port::DiagnosticSink&) override { ready_ = false; }

    ls2k::port::CameraRawFrame WaitRawFrame(int,
                                            ls2k::port::DiagnosticSink&) override {
        ls2k::port::CameraRawFrame frame{};
        frame.valid = true;
        frame.format = ls2k::port::CameraFrameFormat::kGray;
        frame.width = 4;
        frame.height = 2;
        frame.stride = 4;
        frame.metadata.source = Name();
        frame.metadata.frame_id = 17;
        frame.metadata.capture_time_ms = 1700;
        frame.data = {1, 2, 3, 4, 5, 6, 7, 8};
        return frame;
    }

    bool Ready() const override { return ready_; }
    const char* Name() const override { return "owning_fallback"; }

private:
    bool ready_ = false;
};

class ScopedDirectSource final : public ls2k::port::ICameraFrameSource {
public:
    bool Start(const ls2k::port::CameraSourceParameters&,
               ls2k::port::DiagnosticSink&) override {
        ready_ = true;
        return true;
    }

    void Stop(ls2k::port::DiagnosticSink&) override { ready_ = false; }

    ls2k::port::CameraRawFrame WaitRawFrame(int,
                                            ls2k::port::DiagnosticSink&) override {
        return {};
    }

    bool CaptureRawFrame(int,
                         ls2k::port::DiagnosticSink&,
                         const ls2k::port::CameraRawFrameConsumer& consumer) override {
        callback_active_ = true;
        ls2k::port::CameraRawFrameView view{};
        view.valid = true;
        view.format = ls2k::port::CameraFrameFormat::kYuyv;
        view.data = data_.data();
        view.width = 2;
        view.height = 1;
        view.stride = 4;
        view.metadata.source = Name();
        view.metadata.frame_id = 99;
        const bool ok = consumer(view);
        callback_active_ = false;
        released_ = true;
        return ok;
    }

    bool Ready() const override { return ready_; }
    const char* Name() const override { return "scoped_direct"; }
    bool callback_active() const { return callback_active_; }
    bool released() const { return released_; }

private:
    bool ready_ = false;
    bool callback_active_ = false;
    bool released_ = false;
    std::vector<std::uint8_t> data_{10, 1, 20, 2};
};

class NullDiagnostics final : public ls2k::port::DiagnosticSink {
public:
    void Emit(const ls2k::port::DiagnosticEvent&) override {}
};

class RecordingRequeue final : public ls2k::platform::detail::IV4l2BufferRequeue {
public:
    void Requeue(v4l2_buffer buffer) override {
        requeued_indices.push_back(buffer.index);
    }

    std::vector<unsigned int> requeued_indices{};
};

void TestDefaultCaptureRawFrameWrapsOwningFrame() {
    OwningFallbackSource source;
    NullDiagnostics diagnostics;
    bool called = false;
    const bool ok = source.CaptureRawFrame(1,
                                           diagnostics,
                                           [&](const ls2k::port::CameraRawFrameView& view) {
                                               called = true;
                                               Expect(view.Valid(), "default callback view invalid");
                                               Expect(view.format == ls2k::port::CameraFrameFormat::kGray,
                                                      "default callback format mismatch");
                                               Expect(view.data[0] == 1 && view.data[7] == 8,
                                                      "default callback data mismatch");
                                               Expect(view.metadata.frame_id == 17,
                                                      "default callback metadata mismatch");
                                               const ls2k::port::CameraPixelFrameView pixel =
                                                   view.PixelView();
                                               Expect(pixel.Valid(),
                                                      "default callback pixel view invalid");
                                               Expect(pixel.format == ls2k::port::CameraFrameFormat::kGray,
                                                      "default callback pixel format mismatch");
                                               Expect(pixel.data == view.data,
                                                      "default callback pixel view must share source bytes");
                                               return true;
                                           });
    Expect(ok, "default callback should return consumer result");
    Expect(called, "default callback was not invoked");
}

void TestDirectCaptureRawFrameScope() {
    ScopedDirectSource source;
    NullDiagnostics diagnostics;
    bool called = false;
    const bool ok = source.CaptureRawFrame(1,
                                           diagnostics,
                                           [&](const ls2k::port::CameraRawFrameView& view) {
                                               called = true;
                                               Expect(source.callback_active(),
                                                      "direct view must be consumed inside source scope");
                                               Expect(view.Valid(), "direct callback view invalid");
                                               Expect(view.format == ls2k::port::CameraFrameFormat::kYuyv,
                                                      "direct callback format mismatch");
                                               Expect(view.data[0] == 10 && view.data[2] == 20,
                                                      "direct callback data mismatch");
                                               const ls2k::port::CameraPixelFrameView pixel =
                                                   view.PixelView();
                                               Expect(pixel.Valid(),
                                                      "direct callback pixel view invalid");
                                               Expect(pixel.format == ls2k::port::CameraFrameFormat::kYuyv,
                                                      "direct callback pixel format mismatch");
                                               Expect(pixel.stride == 4,
                                                      "direct callback pixel stride mismatch");
                                               Expect(pixel.data == view.data,
                                                      "direct callback pixel view must share source bytes");
                                               return true;
                                           });
    Expect(ok, "direct callback should return consumer result");
    Expect(called, "direct callback was not invoked");
    Expect(source.released(), "direct source should release after callback");
}

void TestRawFrameViewValidRequiresFormatStride() {
    std::vector<std::uint8_t> bytes(8, 0);
    ls2k::port::CameraRawFrameView view{};
    view.valid = true;
    view.format = ls2k::port::CameraFrameFormat::kYuyv;
    view.data = bytes.data();
    view.width = 4;
    view.height = 1;
    view.stride = 4;
    Expect(!view.Valid(), "YUYV raw view with short stride must fail closed");
    Expect(!view.PixelView().Valid(), "short-stride YUYV pixel view must be invalid");
    view.stride = 8;
    Expect(view.Valid(), "YUYV raw view with bytesperline stride should be valid");
    Expect(view.PixelView().Valid(), "YUYV pixel view with bytesperline stride should be valid");
}

void TestV4l2NewestSelectionRequeuesStaleAndReleasesSelectedAfterScope() {
    RecordingRequeue requeue;
    {
        ls2k::platform::detail::DequeuedBufferGuard selected(requeue);
        v4l2_buffer first{};
        first.index = 0;
        v4l2_buffer second{};
        second.index = 1;
        v4l2_buffer invalid{};
        invalid.index = 5;

        const ls2k::platform::detail::V4l2DequeuedSelection first_selection =
            ls2k::platform::detail::SelectNewestDequeuedBufferForProcessing(first, 3, selected);
        Expect(first_selection.has_selected, "first valid V4L2 buffer should be selected");
        Expect(selected.Active(), "selected guard should be active after first selection");
        Expect(requeue.requeued_indices.empty(),
               "first selected buffer must not be requeued before replacement or scope exit");

        const ls2k::platform::detail::V4l2DequeuedSelection second_selection =
            ls2k::platform::detail::SelectNewestDequeuedBufferForProcessing(second, 3, selected);
        Expect(second_selection.has_selected, "second valid V4L2 buffer should be selected");
        Expect(requeue.requeued_indices.size() == 1U &&
                   requeue.requeued_indices[0] == 0U,
               "replacing selected buffer must requeue stale first buffer");
        Expect(selected.Active() && selected.Buffer().index == 1U,
               "newest valid V4L2 buffer should remain selected");

        const ls2k::platform::detail::V4l2DequeuedSelection invalid_selection =
            ls2k::platform::detail::SelectNewestDequeuedBufferForProcessing(invalid, 3, selected);
        Expect(!invalid_selection.has_selected, "invalid V4L2 buffer index must not be selected");
        Expect(requeue.requeued_indices.size() == 1U,
               "invalid buffer must not disturb current selected buffer");
    }
    Expect(requeue.requeued_indices.size() == 2U &&
               requeue.requeued_indices[1] == 1U,
           "selected newest V4L2 buffer must be requeued when callback scope exits");
}

}  // namespace

int main() {
    try {
        TestDefaultCaptureRawFrameWrapsOwningFrame();
        TestDirectCaptureRawFrameScope();
        TestRawFrameViewValidRequiresFormatStride();
        TestV4l2NewestSelectionRequeuesStaleAndReleasesSelectedAfterScope();
    } catch (const std::exception& error) {
        std::cerr << "camera_frame_source_test failed: " << error.what() << "\n";
        return 1;
    }
    std::cout << "camera_frame_source_test passed\n";
    return 0;
}
