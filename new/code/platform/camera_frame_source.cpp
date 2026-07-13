#include "platform/camera_frame_source.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "platform/true_ls2k0300/camera_device.hpp"

namespace ls2k::platform {
namespace {

std::uint64_t NowUs() noexcept {
    using namespace std::chrono;
    return static_cast<std::uint64_t>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

class CameraDeviceFrameSource final : public port::ICameraFrameSource {
public:
    explicit CameraDeviceFrameSource(const char* name) noexcept : name_(name) {}

    bool Start(const port::CameraSourceParameters& config,
               port::DiagnosticSink& diagnostics) override {
        Stop(diagnostics);
        const true_ls2k0300::CameraConfig device_config{
            config.device.c_str(),
            config.width,
            config.height,
            config.fps,
            config.buffer_count,
            config.poll_timeout_ms,
        };
        ready_ = device_.Start(device_config);
        diagnostics.Emit({ready_ ? port::DiagnosticLevel::kInfo : port::DiagnosticLevel::kWarning,
                          ready_ ? "camera_source.device.start"
                                 : "camera_source.device.start_failed",
                          ready_ ? std::string("CameraDevice started on ") + config.device
                                 : std::string("CameraDevice failed to start on ") + config.device,
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
        const std::uint64_t begin_us = NowUs();
        const true_ls2k0300::CameraFrameView frame = device_.Capture();
        const std::uint64_t dequeue_time_ms = port::NowMs();
        if (!frame.valid()) {
            port::EmitRateLimited(diagnostics,
                                  {port::DiagnosticLevel::kWarning,
                                   "camera_source.device.empty",
                                   "CameraDevice returned no complete YUYV frame",
                                   dequeue_time_ms},
                                  1000);
            return false;
        }
        port::CameraRawFrameView view{};
        view.valid = true;
        view.format = port::CameraFrameFormat::kYuyv;
        view.data = frame.data;
        view.width = frame.width;
        view.height = frame.height;
        view.stride = frame.stride;
        view.metadata.source = Name();
        view.metadata.frame_id = ++frame_id_;
        view.metadata.capture_time_ms = dequeue_time_ms;
        view.metadata.dequeue_time_ms = dequeue_time_ms;
        view.metadata.dequeue_us = NowUs() - begin_us;
        view.metadata.drained_buffer_count = 1;
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

class NullCameraFrameSource final : public port::ICameraFrameSource {
public:
    bool Start(const port::CameraSourceParameters&, port::DiagnosticSink&) override {
        ready_ = true;
        return true;
    }
    void Stop(port::DiagnosticSink&) override { ready_ = false; }
    port::CameraRawFrame WaitRawFrame(int, port::DiagnosticSink&) override { return {}; }
    bool Ready() const override { return ready_; }
    const char* Name() const override { return "null"; }

private:
    bool ready_ = false;
};

std::unique_ptr<port::ICameraFrameSource> MakeSourceByName(const std::string& backend) {
    if (backend == "v4l2_yuyv") {
        return std::make_unique<CameraDeviceFrameSource>("v4l2_yuyv");
    }
    // Keep the configuration spelling during the atomic migration, but route
    // it to the same new owner. No vendor globals or vendor UVC code are used.
    if (backend == "vendor_uvc") {
        return std::make_unique<CameraDeviceFrameSource>("camera_device_yuyv");
    }
    if (backend == "null" || backend == "disabled") {
        return std::make_unique<NullCameraFrameSource>();
    }
    return nullptr;
}

}  // namespace

std::unique_ptr<port::ICameraFrameSource> MakeStartedCameraFrameSource(
    const port::RuntimeParameters& params,
    port::DiagnosticSink& diagnostics) {
    auto source = MakeSourceByName(params.camera_source.backend);
    if (source && source->Start(params.camera_source, diagnostics)) {
        return source;
    }
    if (params.camera_source.fallback_backend.empty() ||
        params.camera_source.fallback_backend == params.camera_source.backend) {
        return nullptr;
    }

    port::CameraSourceParameters fallback_config = params.camera_source;
    fallback_config.backend = params.camera_source.fallback_backend;
    auto fallback = MakeSourceByName(fallback_config.backend);
    if (fallback && fallback->Start(fallback_config, diagnostics)) {
        diagnostics.Emit({port::DiagnosticLevel::kWarning,
                          "camera_source.fallback",
                          "camera source primary failed; using fallback backend " +
                              fallback_config.backend,
                          port::NowMs()});
        return fallback;
    }
    return nullptr;
}

}  // namespace ls2k::platform
