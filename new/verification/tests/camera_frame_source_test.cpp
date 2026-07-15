#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "platform/camera_frame_source.hpp"
#include "port/camera_frame_source.hpp"

namespace {

void Expect(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class OwningFrameSource final : public ls2k::port::ICameraFrameSource {
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

class RecordingDiagnostics final : public ls2k::port::DiagnosticSink {
public:
    void Emit(const ls2k::port::DiagnosticEvent& event) override { events.push_back(event); }

    bool Saw(const std::string& code, const std::string& message_fragment) const {
        for (const auto& event : events) {
            if (event.code == code && event.message.find(message_fragment) != std::string::npos) {
                return true;
            }
        }
        return false;
    }

    std::vector<ls2k::port::DiagnosticEvent> events{};
};

void TestDefaultCaptureRawFrameWrapsOwningFrame() {
    OwningFrameSource source;
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

void TestConfiguredSourceSelectionAndStartupDiagnostics() {
    ls2k::port::RuntimeParameters params{};
    RecordingDiagnostics diagnostics;
    params.camera_source.backend = "unsupported_backend";
    auto unsupported = ls2k::platform::MakeStartedCameraFrameSource(params, diagnostics);
    Expect(unsupported == nullptr, "unsupported backend must not create a frame source");

    params.camera_source.backend = "v4l2_yuyv";
    params.camera_source.device = "/definitely/not/a/video/device";
    auto failed = ls2k::platform::MakeStartedCameraFrameSource(params, diagnostics);
    Expect(failed == nullptr, "failed V4L2 startup must not invent a fallback");
    Expect(diagnostics.Saw("camera_source.start.open_failed",
                           "stage=open status=open_failed"),
           "source boundary must expose typed owner startup stage/status");
}

void TestActiveConfigAndMetadataMapping() {
    ls2k::port::CameraSourceParameters params{};
    params.device = "/dev/test-video";
    params.width = 160;
    params.height = 120;
    params.fps = 55;
    params.buffer_count = 4;
    params.poll_timeout_ms = 37;
    params.drain_ready_buffers = false;
    const auto single = ls2k::platform::BuildCameraDeviceConfig(params);
    Expect(std::string(single.device) == params.device, "device mapping mismatch");
    Expect(single.width == 160 && single.height == 120 && single.fps == 55,
           "geometry/fps mapping mismatch");
    Expect(single.buffer_count == 4 && single.timeout_ms == 37,
           "buffer/timeout mapping mismatch");
    Expect(!single.drain_ready_buffers, "disabled drain mapping mismatch");
    params.drain_ready_buffers = true;
    Expect(ls2k::platform::BuildCameraDeviceConfig(params).drain_ready_buffers,
           "enabled drain mapping mismatch");

    ls2k::platform::true_ls2k0300::CameraCaptureResult capture{};
    capture.capture_time_ms = 4100;
    capture.dequeue_time_ms = 4123;
    capture.v4l2_sequence = 0xFEDCBA98U;
    capture.v4l2_timestamp_valid = true;
    capture.drained_buffer_count = 3;
    capture.poll_wait_us = 101;
    capture.dequeue_us = 202;
    const auto metadata =
        ls2k::platform::BuildCameraRawFrameMetadata("v4l2_yuyv", 77, capture);
    Expect(metadata.source == "v4l2_yuyv" && metadata.frame_id == 77,
           "source/frame-id metadata mapping mismatch");
    Expect(metadata.capture_time_ms == 4100 && metadata.dequeue_time_ms == 4123,
           "capture/dequeue time metadata mapping mismatch");
    Expect(metadata.v4l2_sequence == 0xFEDCBA98U && metadata.v4l2_timestamp_valid,
           "V4L2 sequence/timestamp metadata mapping mismatch");
    Expect(metadata.drained_buffer_count == 3 && metadata.poll_wait_us == 101 &&
               metadata.dequeue_us == 202,
           "drain timing metadata mapping mismatch");
}

}  // namespace

int main() {
    try {
        TestDefaultCaptureRawFrameWrapsOwningFrame();
        TestDirectCaptureRawFrameScope();
        TestRawFrameViewValidRequiresFormatStride();
        TestConfiguredSourceSelectionAndStartupDiagnostics();
        TestActiveConfigAndMetadataMapping();
    } catch (const std::exception& error) {
        std::cerr << "camera_frame_source_test failed: " << error.what() << "\n";
        return 1;
    }
    std::cout << "camera_frame_source_test passed\n";
    return 0;
}
