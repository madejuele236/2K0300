#ifndef LS2K_LEGACY_STEERING_VISUAL_ELEMENT_PIPELINE_HPP
#define LS2K_LEGACY_STEERING_VISUAL_ELEMENT_PIPELINE_HPP

#include <vector>

#include "port/camera_frame_types.hpp"
#include "port/runtime_parameter_types.hpp"
#include "port/visual_element_evidence_types.hpp"
#include "vision/bev/bev_row_facts.hpp"
#include "vision/bev/bev_segment_connectivity.hpp"

namespace ls2k::vision {

/// 视觉元素管线输入，runtime V1 只接受稀疏行事实和按需 ROI 上下文。
struct VisualElementPipelineInput {
    const std::vector<BEVSimpleRowScan>* sparse_rows = nullptr;  ///< 稀疏行扫描结果指针（可为nullptr）
    BEVSegmentConnectivityResult origin_to_cross_sample_midpoint_connectivity{};
};

/// 视觉元素管线输出，只发布元素检测证据。
struct VisualElementPipelineResult {
    port::VisualElementEvidenceFrame evidence{};
};

/// 运行完整的视觉元素管线
/// 当前只执行 Cross 检测；路径规划由独立 owner 负责。
/// @param input 管线输入
/// @param params 运行时参数
/// @return 管线结果
VisualElementPipelineResult RunVisualElementPipeline(const VisualElementPipelineInput& input,
                                                     const port::RuntimeParameters& params);

}  // namespace ls2k::vision

#endif  // LS2K_LEGACY_STEERING_VISUAL_ELEMENT_PIPELINE_HPP
