#ifndef LS2K_VISION_BEV_SAMPLE_PROJECTION_LUT_HPP
#define LS2K_VISION_BEV_SAMPLE_PROJECTION_LUT_HPP

#include <array>
#include <cstddef>
#include <vector>

#include "port/bev_reference_types.hpp"
#include "port/camera_frame_types.hpp"
#include "port/runtime_parameter_types.hpp"
#include "vision/bev/bev_projector.hpp"

namespace ls2k::vision {

/// 采样投影状态（用于查找表条目）
enum class BEVSampleProjectionState {
    kSampleable,         ///< 可采样（投影在图像范围内）
    kOutsideFrame,       ///< 投影点超出图像范围
    kProjectionFailed,   ///< 投影失败
};

/// 采样投影查找表条目（缓存一个BEV点投影到图像的结果）
struct BEVSampleProjectionEntry {
    BEVSampleProjectionState state = BEVSampleProjectionState::kProjectionFailed; ///< 投影状态
    float forward_m = 0.0F;  ///< BEV前向距离（米）
    float lateral_m = 0.0F;  ///< BEV横向偏移（米）
    float image_row_px = 0.0F; ///< 投影到图像的行坐标（像素）
    float image_col_px = 0.0F; ///< 投影到图像的列坐标（像素）
};

/// BEV采样投影查找表，缓存所有采样点从BEV到图像的投影结果
struct BEVSampleProjectionLut {
    bool valid = false;                                             ///< 查找表是否有效
    port::BEVProjectorCalibration calibration{};                    ///< 生成此表时的标定参数
    int frame_width = 0;                                            ///< 源图像宽度
    int frame_height = 0;                                           ///< 源图像高度
    int frame_stride = 0;                                           ///< 源图像行跨度
    std::array<float, port::kBevReferenceSampleCount> forward_samples_m{}; ///< 前向采样距离数组
    std::size_t sparse_row_count = port::kBevReferenceSampleCount;          ///< 启用的前向采样前缀长度
    float lateral_limit_m = 0.0F;                                   ///< 横向采样限制（米）
    float lateral_step_m = 0.0F;                                    ///< 横向采样步长（米）
    std::size_t lateral_sample_count = 0;                           ///< 横向采样点数
    std::vector<BEVSampleProjectionEntry> entries{};                ///< 所有采样点的投影条目
};

/// 将采样投影状态枚举转换为可读字符串
const char* ToString(BEVSampleProjectionState state);

/// 确保采样投影查找表有效且匹配当前帧/参数/投影器，否则重建
/// @param lut 查找表
/// @param frame 当前相机帧
/// @param params 运行时参数
/// @param projector BEV投影器
/// @return 查找表是否有效
bool EnsureBEVSampleProjectionLut(BEVSampleProjectionLut& lut,
                                  const port::CameraPixelFrameView& frame,
                                  const port::RuntimeParameters& params,
                                  const BEVProjector& projector);

}  // namespace ls2k::vision

#endif  // LS2K_VISION_BEV_SAMPLE_PROJECTION_LUT_HPP
