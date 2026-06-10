#include "runtime/capture/camera_capture_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <optional>

#include "platform/camera_frame_source.hpp"
#include "port/perf_counter.hpp"
#include "port/thread_scheduling.hpp"

namespace ls2k::runtime {
namespace {

uint64_t NowUs() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

bool CopyRawToPixelView(const port::CameraRawFrameView& raw,
                        port::MutableCameraPixelFrameView out) {
    if (!raw.Valid() ||
        !out.Valid() ||
        raw.format != out.format ||
        raw.width != out.width ||
        raw.height != out.height ||
        raw.stride < port::MinimumStrideBytes(raw.format, raw.width) ||
        out.stride < raw.stride) {
        return false;
    }
    for (int row = 0; row < raw.height; ++row) {
        const std::uint8_t* src =
            raw.data + static_cast<std::size_t>(row) * static_cast<std::size_t>(raw.stride);
        std::uint8_t* dst =
            out.data + static_cast<std::size_t>(row) * static_cast<std::size_t>(out.stride);
        std::copy(src, src + raw.stride, dst);
    }
    return true;
}

}  // namespace

CameraCaptureWorker::CameraCaptureWorker(CameraFrameStore& frame_store,
                                         port::DiagnosticSink& diagnostics)
    : frame_store_(frame_store), diagnostics_(diagnostics) {}

CameraCaptureWorker::~CameraCaptureWorker() {
    Stop();
}

bool CameraCaptureWorker::Start(const port::RuntimeParameters& params) {
    if (running_) {
        return true;
    }
    params_ = params;
    source_ = platform::MakeStartedCameraFrameSource(params_, diagnostics_);
    if (!source_) {
        diagnostics_.Emit({port::DiagnosticLevel::kFailSafe,
                           "camera_capture_worker.source_unavailable",
                           "camera capture worker could not start any frame source",
                           port::NowMs()});
        return false;
    }
    stop_requested_.store(false);
    running_.store(true);
    thread_ = std::thread([this]() { ThreadMain(); });
    diagnostics_.Emit({port::DiagnosticLevel::kInfo,
                       "camera_capture_worker.start",
                       "camera capture worker started",
                       port::NowMs()});
    return true;
}

void CameraCaptureWorker::Stop() {
    stop_requested_.store(true);
    if (thread_.joinable()) {
        thread_.join();
    }
    if (source_) {
        source_->Stop(diagnostics_);
        source_.reset();
    }
    running_.store(false);
}

bool CameraCaptureWorker::Running() const {
    return running_.load();
}

bool CameraCaptureWorker::ConvertRawFrame(const port::CameraRawFrame& raw,
                                          port::LegacyCameraFrame& out,
                                          port::CameraRawFrameMetadata& metadata) {
    metadata = raw.metadata;
    if (!raw.valid ||
        raw.width <= 0 ||
        raw.height <= 0 ||
        raw.width > port::kCompiledCameraFrameWidth ||
        raw.height > port::kCompiledCameraFrameHeight) {
        return false;
    }

    const uint64_t begin_us = NowUs();
    LS2K_PERF_SCOPE(port::PerfStage::kCameraYuyvToGray);
    bool ok = false;
    if (raw.format == port::CameraFrameFormat::kGray) {
        if (raw.stride >= raw.width &&
            raw.data.size() >= static_cast<std::size_t>(raw.stride) *
                                   static_cast<std::size_t>(raw.height)) {
            out = {};
            out.width = raw.width;
            out.height = raw.height;
            for (int row = 0; row < raw.height; ++row) {
                const std::uint8_t* src =
                    raw.data.data() +
                    static_cast<std::size_t>(row) * static_cast<std::size_t>(raw.stride);
                std::uint8_t* dst =
                    out.gray.data() +
                    static_cast<std::size_t>(row) * static_cast<std::size_t>(raw.width);
                std::copy(src, src + raw.width, dst);
            }
            ok = true;
        }
    } else if (raw.format == port::CameraFrameFormat::kYuyv) {
        ok = YuyvToGray(raw.data.data(), raw.width, raw.height, raw.stride, out);
    }
    metadata.yuyv_to_gray_us = NowUs() - begin_us;
    return ok;
}

bool CameraCaptureWorker::ConvertRawFrameViewToStore(const port::CameraRawFrameView& raw) {
    port::CameraRawFrameMetadata metadata = raw.metadata;
    if (!raw.Valid() ||
        raw.width <= 0 ||
        raw.height <= 0 ||
        raw.width > port::kCompiledCameraFrameWidth ||
        raw.height > port::kCompiledCameraFrameHeight) {
        return false;
    }

    std::optional<CameraFrameStore::WriteLease> lease =
        frame_store_.ReserveWritable(raw.format, raw.width, raw.height, raw.stride, metadata);
    if (!lease.has_value()) {
        return false;
    }
    port::MutableCameraPixelFrameView dst = lease->MutablePixelView();
    if (!dst.Valid()) {
        lease->Abort();
        return false;
    }

    const bool ok = CopyRawToPixelView(raw, dst);
    metadata.yuyv_to_gray_us = 0;
    if (!ok) {
        lease->Abort();
        return false;
    }

    const uint64_t frame_id =
        metadata.frame_id == 0 ? raw.metadata.frame_id : metadata.frame_id;
    const uint64_t capture_time_ms =
        metadata.capture_time_ms == 0 ? port::NowMs() : metadata.capture_time_ms;
    const uint64_t submit_begin_us = NowUs();
    const CameraFrameHandle handle =
        lease->Commit(frame_id, capture_time_ms, metadata, submit_begin_us);
    return handle.valid;
}

void CameraCaptureWorker::ThreadMain() {
    port::ApplyThreadSchedulingProfile(port::ThreadSchedulingRole::kCameraCapture,
                                       &diagnostics_);
    while (!stop_requested_.load()) {
        bool saw_raw_frame = false;
        bool materialized = false;
        (void)source_->CaptureRawFrame(
            std::max(1, params_.camera_source.poll_timeout_ms),
            diagnostics_,
            [&](const port::CameraRawFrameView& raw) {
                saw_raw_frame = raw.valid;
                LS2K_PERF_SCOPE(port::PerfStage::kCameraFrameMaterialize);
                {
                    LS2K_PERF_SCOPE(port::PerfStage::kCameraStoreSubmit);
                    materialized = ConvertRawFrameViewToStore(raw);
                }
                return materialized;
            });
        if (!saw_raw_frame) {
            continue;
        }
        if (!materialized) {
            port::EmitRateLimited(diagnostics_,
                                  {port::DiagnosticLevel::kWarning,
                                   "camera_capture_worker.convert_failed",
                                   "camera capture worker dropped frame because conversion failed",
                                   port::NowMs()},
                                  1000);
            continue;
        }
    }
    running_.store(false);
}

}  // namespace ls2k::runtime
