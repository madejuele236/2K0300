#ifndef LS2K_LEGACY_STEERING_BEV_SIMPLE_PERCEPTION_HPP
#define LS2K_LEGACY_STEERING_BEV_SIMPLE_PERCEPTION_HPP

#include <string>
#include <vector>

#include "vision/bev/bev_boundary_row.hpp"
#include "vision/bev/bev_projector.hpp"
#include "vision/bev/bev_reference_path_builder.hpp"
#include "vision/bev/bev_row_facts.hpp"
#include "vision/bev/bev_sample_projection_lut.hpp"
#include "vision/bev/bev_segment_connectivity.hpp"
#include "port/bev_reference_types.hpp"
#include "port/camera_frame_types.hpp"
#include "port/runtime_parameter_types.hpp"

namespace ls2k::vision {

/// 简单BEV感知结果，包含行扫描结果和构建的参考路径
struct BEVSimplePerceptionResult {
    int threshold = 0;                           ///< 使用的二值化阈值
    std::vector<BEVSimpleRowScan> rows{};        ///< 各行的扫描结果
    std::size_t boundary_jump_count = 0;         ///< V9 局部 Y 边界跳变数量
    std::size_t boundary_span_count = 0;         ///< V9 同行边界 span 数量
    BEVSegmentConnectivityResult origin_to_last_row_midpoint_connectivity{};
    BEVRoadPathFacts road_path_facts{};           ///< 普通路径选中候选的对齐中心/实际边界事实
    port::BEVReferencePath reference_path{};      ///< 构建的参考路径
    std::string reference_mode = "none";          ///< 参考路径模式字符串
    std::string reference_source = "none";        ///< 参考路径来源字符串
};

/// 将采样投影状态枚举转换为可读字符串
const char* ToString(BEVSampleProjectionState state);
/// 将参考路径模式枚举转换为可读字符串
const char* ToString(port::ReferenceMode mode);
/// 将BEV路径点来源枚举转换为可读字符串
const char* ToString(port::BEVPathPointSource source);

/// 运行完整的BEV简单感知管线
/// @param frame 原始相机帧
/// @param params 运行时参数
/// @param projector BEV投影器
/// @param lut 可选的外部查找表指针（为nullptr时自动创建临时表）
/// @return 感知结果（行扫描、参考路径等）
BEVSimplePerceptionResult RunBEVSimplePerception(const port::CameraPixelFrameView& frame,
                                                 const port::RuntimeParameters& params,
                                                 const BEVProjector& projector,
                                                 BEVSampleProjectionLut* lut);

}  // namespace ls2k::vision

#endif  // LS2K_LEGACY_STEERING_BEV_SIMPLE_PERCEPTION_HPP
