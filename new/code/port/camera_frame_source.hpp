#ifndef LS2K_PORT_CAMERA_FRAME_SOURCE_HPP
#define LS2K_PORT_CAMERA_FRAME_SOURCE_HPP

#include <functional>
#include <string>

#include "port/camera_frame_types.hpp"
#include "port/diagnostics.hpp"
#include "port/runtime_parameter_types.hpp"

namespace ls2k::port {

/// ICameraFrameSource 暴露的短生命周期 raw/gray frame view。
struct CameraRawFrameView {
    bool valid = false;                  ///< 是否有可用帧
    CameraFrameFormat format = CameraFrameFormat::kGray;  ///< 数据格式
    const uint8_t* data = nullptr;       ///< 原始帧数据，只在 source 回调期间有效
    int width = 0;                       ///< 宽度
    int height = 0;                      ///< 高度
    int stride = 0;                      ///< 每行字节数
    CameraRawFrameMetadata metadata{};   ///< 帧元数据

    bool Valid() const {
        return valid && data != nullptr && width > 0 && height > 0 &&
               stride >= MinimumStrideBytes(format, width);
    }

    CameraPixelFrameView PixelView() const {
        CameraPixelFrameView view{};
        view.valid = Valid();
        view.format = format;
        view.data = data;
        view.width = width;
        view.height = height;
        view.stride = stride;
        view.frame_id = metadata.frame_id;
        view.capture_time_ms = metadata.capture_time_ms;
        return view;
    }
};

using CameraRawFrameConsumer = std::function<bool(const CameraRawFrameView& view)>;

/// 相机帧源抽象，只隐藏 backend，不拥有 latest/history 语义
class ICameraFrameSource {
public:
    virtual ~ICameraFrameSource() = default;

    /// 启动 frame source
    virtual bool Start(const CameraSourceParameters& config,
                       DiagnosticSink& diagnostics) = 0;
    /// 停止 frame source
    virtual void Stop(DiagnosticSink& diagnostics) = 0;
    /// 等待一个 raw/gray frame，timeout 只属于 source/worker 线程
    virtual CameraRawFrame WaitRawFrame(int timeout_ms,
                                        DiagnosticSink& diagnostics) = 0;
    /// 在 source 拥有 backend buffer 生命周期期间同步消费一个 raw/gray frame。
    virtual bool CaptureRawFrame(int timeout_ms,
                                 DiagnosticSink& diagnostics,
                                 const CameraRawFrameConsumer& consumer) {
        CameraRawFrame frame = WaitRawFrame(timeout_ms, diagnostics);
        if (!frame.valid) {
            return false;
        }
        CameraRawFrameView view{};
        view.valid = true;
        view.format = frame.format;
        view.data = frame.data.data();
        view.width = frame.width;
        view.height = frame.height;
        view.stride = frame.stride;
        view.metadata = frame.metadata;
        return consumer(view);
    }
    /// 后端是否可用
    virtual bool Ready() const = 0;
    /// 后端名称
    virtual const char* Name() const = 0;
};

}  // namespace ls2k::port

#endif  // LS2K_PORT_CAMERA_FRAME_SOURCE_HPP
