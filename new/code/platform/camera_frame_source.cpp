#include "platform/camera_frame_source.hpp"

#include <memory>
#include <sstream>
#include <string>

namespace ls2k::platform {
namespace {

std::string StartDiagnosticMessage(const true_ls2k0300::CameraStartResult& result,
                                   const port::CameraSourceParameters& config) {
    std::ostringstream message;
    message << "stage=" << true_ls2k0300::CameraLifecycleStageCode(result.stage)
            << " status=" << true_ls2k0300::CameraStartStatusCode(result.status)
            << " device=" << config.device;
    if (result.negotiated.width > 0 || result.negotiated.height > 0) {
        message << " negotiated=" << result.negotiated.width << 'x'
                << result.negotiated.height << " stride=" << result.negotiated.stride
                << " pixel_format=" << result.negotiated.pixel_format
                << " mapped_buffer_count=" << result.mapped_buffer_count;
    }
    return message.str();
}

std::string CaptureDiagnosticMessage(const true_ls2k0300::CameraCaptureResult& result) {
    std::ostringstream message;
    message << "stage=" << true_ls2k0300::CameraLifecycleStageCode(result.stage)
            << " status=" << true_ls2k0300::CameraCaptureStatusCode(result.status)
            << " poll_wait_us=" << result.poll_wait_us
            << " dequeue_us=" << result.dequeue_us
            << " drained_buffer_count=" << result.drained_buffer_count;
    return message.str();
}

class CameraDeviceFrameSource final : public port::ICameraFrameSource {
public:
    explicit CameraDeviceFrameSource(const char* name) noexcept : name_(name) {}

    bool Start(const port::CameraSourceParameters& config,
               port::DiagnosticSink& diagnostics) override {
        Stop(diagnostics);
        const true_ls2k0300::CameraConfig device_config = BuildCameraDeviceConfig(config);
        const true_ls2k0300::CameraStartResult result = device_.Start(device_config);
        ready_ = result.ok();
        diagnostics.Emit({ready_ ? port::DiagnosticLevel::kInfo : port::DiagnosticLevel::kWarning,
                          std::string("camera_source.start.") +
                              true_ls2k0300::CameraStartStatusCode(result.status),
                          StartDiagnosticMessage(result, config),
                          port::NowMs()});
        return ready_;
    }

    void Stop(port::DiagnosticSink& diagnostics) override {
        const bool was_ready = ready_;
        device_.Stop();
        ready_ = false;
        if (was_ready) {
            diagnostics.Emit({port::DiagnosticLevel::kInfo,
                              "camera_source.device.stop",
                              "CameraDevice released stream, mappings, buffers, and fd",
                              port::NowMs()});
        }
    }

    port::CameraRawFrame WaitRawFrame(int timeout_ms,
                                      port::DiagnosticSink& diagnostics) override {
        (void)timeout_ms;  // timeout is bound into CameraDevice at Start().
        port::CameraRawFrame out{};
        static_cast<void>(CaptureRawFrame(
            timeout_ms,
            diagnostics,
            [&out](const port::CameraRawFrameView& view) {
                if (!view.Valid()) {
                    return false;
                }
                const std::size_t size =
                    static_cast<std::size_t>(view.height) * static_cast<std::size_t>(view.stride);
                out.valid = true;
                out.format = view.format;
                out.width = view.width;
                out.height = view.height;
                out.stride = view.stride;
                out.metadata = view.metadata;
                out.data.assign(view.data, view.data + size);
                return true;
            }));
        return out;
    }

    bool CaptureRawFrame(int timeout_ms,
                         port::DiagnosticSink& diagnostics,
                         const port::CameraRawFrameConsumer& consumer) override {
        (void)timeout_ms;
        if (!ready_) {
            return false;
        }
        const true_ls2k0300::CameraCaptureResult capture = device_.Capture();
        const std::uint64_t dequeue_time_ms = port::NowMs();
        if (!capture.ok()) {
            if (capture.status == true_ls2k0300::CameraCaptureStatus::kPollTimeout) {
                return false;
            }
            port::EmitRateLimited(diagnostics,
                                  {port::DiagnosticLevel::kWarning,
                                   std::string("camera_source.capture.") +
                                       true_ls2k0300::CameraCaptureStatusCode(capture.status),
                                   CaptureDiagnosticMessage(capture),
                                   dequeue_time_ms},
                                  1000);
            return false;
        }
        const true_ls2k0300::CameraFrameView& frame = capture.frame;
        port::CameraRawFrameView view{};
        view.valid = true;
        view.format = port::CameraFrameFormat::kYuyv;
        view.data = frame.data;
        view.width = frame.width;
        view.height = frame.height;
        view.stride = frame.stride;
        view.metadata = BuildCameraRawFrameMetadata(Name(), ++frame_id_, capture);
        return consumer(view);
    }

    bool Ready() const override { return ready_ && device_.Running(); }
    const char* Name() const override { return name_; }

private:
    const char* name_;
    true_ls2k0300::CameraDevice device_{};
    bool ready_ = false;
    std::uint64_t frame_id_ = 0;
};

std::unique_ptr<port::ICameraFrameSource> MakeSourceByName(const std::string& backend) {
    if (backend == "v4l2_yuyv") {
        return std::make_unique<CameraDeviceFrameSource>("v4l2_yuyv");
    }
    return nullptr;
}

}  // namespace

true_ls2k0300::CameraConfig BuildCameraDeviceConfig(
    const port::CameraSourceParameters& params) noexcept {
    return {params.device.c_str(),
            params.width,
            params.height,
            params.fps,
            params.buffer_count,
            params.poll_timeout_ms,
            params.drain_ready_buffers};
}

port::CameraRawFrameMetadata BuildCameraRawFrameMetadata(
    const char* source,
    std::uint64_t frame_id,
    const true_ls2k0300::CameraCaptureResult& capture) {
    port::CameraRawFrameMetadata metadata{};
    metadata.source = source == nullptr ? "none" : source;
    metadata.frame_id = frame_id;
    metadata.capture_time_ms = capture.capture_time_ms;
    metadata.dequeue_time_ms = capture.dequeue_time_ms;
    metadata.v4l2_sequence = capture.v4l2_sequence;
    metadata.v4l2_timestamp_valid = capture.v4l2_timestamp_valid;
    metadata.drained_buffer_count = capture.drained_buffer_count;
    metadata.poll_wait_us = capture.poll_wait_us;
    metadata.dequeue_us = capture.dequeue_us;
    return metadata;
}

std::unique_ptr<port::ICameraFrameSource> MakeStartedCameraFrameSource(
    const port::RuntimeParameters& params,
    port::DiagnosticSink& diagnostics) {
    auto source = MakeSourceByName(params.camera_source.backend);
    if (source && source->Start(params.camera_source, diagnostics)) {
        return source;
    }
    return nullptr;
}

}  // namespace ls2k::platform
