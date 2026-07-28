#include "runtime/perception_frontend.hpp"

/// 感知前端实现 —— 运行时感知管线调度。
/// 负责故障注入、诊断发布、感知结果缓存和前端线程生命周期管理。

#include <cstdlib>
#include <optional>
#include <string>
#include <utility>

#include "port/numeric_parse.hpp"
#include "port/perf_counter.hpp"

namespace ls2k::runtime {
namespace {

/// 从环境变量读取正整数值（用于故障注入间隔）
/// @param key         环境变量名
/// @param diagnostics 诊断输出接口
/// @param now_ms      当前时间戳
/// @return            正整数值，无效或不存在时返回 0
int ReadPositiveIntervalEnv(const char* key, port::DiagnosticSink& diagnostics, uint64_t now_ms) {
    const char* value = std::getenv(key);
    if (value == nullptr || value[0] == '\0') {
        return 0;
    }
    const std::optional<int> parsed = port::ParsePositiveIntStrict(value);
    if (parsed.has_value()) {
        return *parsed;
    }
    port::EmitRateLimited(diagnostics,
                          {port::DiagnosticLevel::kWarning,
                           "perception.inject.invalid_env",
                           std::string("ignoring invalid fault-injection interval for ") + key + "=" + value,
                           now_ms},
                          1000);
    return 0;
}

/// 构建丢帧回退感知结果（用于故障注入场景 —— 模拟帧丢失，保持帧 ID 但标记为不新鲜）
/// @param capture  相机捕获信息
/// @return         构建的降级感知结果
port::PerceptionResult BuildDroppedFrameFallback(const port::CameraCapture& capture) {
    port::PerceptionResult fallback{};
    fallback.published = true;
    fallback.fresh = false;
    fallback.frame_id = capture.frame_id;
    fallback.capture_time_ms = capture.capture_time_ms;
    fallback.publish_time_ms = capture.capture_time_ms;
    fallback.perception_tag = "injected-drop-frame";
    return fallback;
}

}  // namespace

/// 构造感知前端：保存相机适配器、运行时状态和诊断接口引用
/// @param camera      相机适配器
/// @param state       运行时状态
/// @param diagnostics 诊断输出接口
PerceptionFrontend::PerceptionFrontend(CameraFrameStore& frame_store,
                                       RuntimeState& state,
                                       port::DiagnosticSink& diagnostics)
    : frame_store_(frame_store), state_(state), diagnostics_(diagnostics) {}

/// 配置感知前端：委派到帧感知管线的配置
/// @param params  运行时参数
/// @return        配置是否成功
bool PerceptionFrontend::Configure(const port::RuntimeParameters& params) {
    return frame_pipeline_.Configure(params, diagnostics_);
}

/// 消费感知复位请求：清空 reference hold 与 ML scene/tracker；CircleV2 contract 不变。
void PerceptionFrontend::ConsumeMemoryResetRequest() {
    const uint64_t generation = state_.perception_memory_reset_generation.load();
    if (generation == consumed_perception_memory_reset_generation_) {
        return;
    }
    frame_pipeline_.ResetReferenceMemory();
    consumed_perception_memory_reset_generation_ = generation;
}

/// 处理一帧图像：故障注入 → 空帧处理 → 二值模型 → sparse BEV 感知 → 结果缓存。
/// 支持通过环境变量 LS2K_FAULT_INJECT_DROP_FRAME_EVERY_N 模拟帧丢失。
/// @param params  运行时参数
bool PerceptionFrontend::ProcessOneFrame(const port::RuntimeParameters& params) {
    ConsumeMemoryResetRequest();

    std::optional<CameraFrameStore::ReadLease> latest =
        frame_store_.AcquireLatestAfter(last_processed_frame_id_);
    if (!latest.has_value()) {
        return false;
    }
    LS2K_PERF_SCOPE(port::PerfStage::kPerceptionFrame);
    const CameraFrameHandle& handle = latest->Handle();

    ++processed_frames_;
    last_processed_frame_id_ = handle.frame_id;
    port::CameraCapture capture{};
    capture.has_frame = true;
    capture.marker = port::CameraGeometryMarker::kPhase1Adapted;
    capture.frame_id = handle.frame_id;
    capture.capture_time_ms = handle.capture_time_ms;
    capture.source_width = handle.width;
    capture.source_height = handle.height;
    capture.view = latest->View();
    capture.pixel_view = latest->PixelView();
    {
        LS2K_PERF_SCOPE(port::PerfStage::kCameraFrameAge);
        (void)capture.capture_time_ms;
    }
    const int drop_frame_every_n =
        ReadPositiveIntervalEnv("LS2K_FAULT_INJECT_DROP_FRAME_EVERY_N", diagnostics_, port::NowMs());
    if (drop_frame_every_n > 0 && processed_frames_ % static_cast<uint64_t>(drop_frame_every_n) == 0) {
        port::EmitRateLimited(diagnostics_,
                              {port::DiagnosticLevel::kWarning,
                               "perception.inject.drop_frame",
                               "injecting bounded Phase B dropped-frame fault on the accepted runtime entrypoint",
                               port::NowMs()},
                              1000);
        port::PerceptionResult fallback = BuildDroppedFrameFallback(capture);
        fallback.publish_time_ms = port::NowMs();
        {
            std::lock_guard<std::mutex> lock(state_.shared_mutex);
            state_.perception = std::move(fallback);
            ++state_.perception_publish_count;
        }
        return true;
    }

    port::MotionHistory motion_history{};
    bool motion_session_active = false;
    {
        std::lock_guard<std::mutex> lock(state_.shared_mutex);
        motion_history = state_.motion_history;
        motion_session_active =
            state_.motion_state.phase == control::MotionPhase::kSpinup ||
            state_.motion_state.phase == control::MotionPhase::kRunning;
    }
    port::PerceptionResult perception =
        frame_pipeline_.ProcessFrame(capture,
                                     params,
                                     motion_history,
                                     motion_session_active);

    {
        std::lock_guard<std::mutex> lock(state_.shared_mutex);
        state_.perception = std::move(perception);
        ++state_.perception_publish_count;
    }
    return true;
}

}  // namespace ls2k::runtime
