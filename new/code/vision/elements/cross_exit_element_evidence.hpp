#ifndef LS2K_LEGACY_STEERING_CROSS_EXIT_ELEMENT_EVIDENCE_HPP
#define LS2K_LEGACY_STEERING_CROSS_EXIT_ELEMENT_EVIDENCE_HPP

#include <vector>

#include "port/runtime_parameter_types.hpp"
#include "port/visual_element_evidence_types.hpp"
#include "vision/bev/bev_row_facts.hpp"
#include "vision/bev/bev_segment_connectivity.hpp"

namespace ls2k::vision {

/// 检测十字出口元素证据（从稀疏行扫描中检测十字路口出口特征）
/// @param rows 稀疏行扫描结果
/// @param params 运行时参数
/// @return 十字出口元素证据检测结果
port::CrossExitElementEvidence DetectCrossExitEvidence(
    const std::vector<BEVSimpleRowScan>& rows,
    const BEVSegmentConnectivityResult& origin_to_cross_sample_midpoint_connectivity,
    const port::RuntimeParameters& params);

}  // namespace ls2k::vision

#endif  // LS2K_LEGACY_STEERING_CROSS_EXIT_ELEMENT_EVIDENCE_HPP
