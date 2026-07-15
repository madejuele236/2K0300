#ifndef LS2K_PLATFORM_CAMERA_FRAME_SOURCE_HPP
#define LS2K_PLATFORM_CAMERA_FRAME_SOURCE_HPP

#include <memory>

#include "platform/true_ls2k0300/camera_device.hpp"
#include "port/camera_frame_source.hpp"
#include "port/runtime_parameter_types.hpp"

namespace ls2k::platform {

/// 将活动 runtime camera 参数一次映射到 V4L2 owner 配置。
true_ls2k0300::CameraConfig BuildCameraDeviceConfig(
    const port::CameraSourceParameters& params) noexcept;

/// 将 V4L2 owner 的最终 selected-buffer 事实一次映射为 raw-frame metadata。
port::CameraRawFrameMetadata BuildCameraRawFrameMetadata(
    const char* source,
    std::uint64_t frame_id,
    const true_ls2k0300::CameraCaptureResult& capture);

/// 创建并启动配置指定的 camera frame source。
std::unique_ptr<port::ICameraFrameSource> MakeStartedCameraFrameSource(
    const port::RuntimeParameters& params,
    port::DiagnosticSink& diagnostics);

}  // namespace ls2k::platform

#endif  // LS2K_PLATFORM_CAMERA_FRAME_SOURCE_HPP
