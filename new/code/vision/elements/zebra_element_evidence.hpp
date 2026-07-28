#ifndef LS2K_VISION_ELEMENTS_ZEBRA_ELEMENT_EVIDENCE_HPP
#define LS2K_VISION_ELEMENTS_ZEBRA_ELEMENT_EVIDENCE_HPP

#include <vector>

#include "port/runtime_parameter_types.hpp"
#include "port/visual_element_evidence_types.hpp"
#include "vision/bev/bev_row_facts.hpp"

namespace ls2k::vision {

/// 从现有稀疏 BEV 二值跳变事实识别当前帧中的 Zebra。
///
/// Zebra 只发布识别证据，不维护跨帧状态，也不生成视觉参考候选。
port::VisualElementEvidenceRecord DetectZebraEvidence(
    const std::vector<BEVSimpleRowScan>& rows,
    const port::RuntimeParameters& params);

}  // namespace ls2k::vision

#endif  // LS2K_VISION_ELEMENTS_ZEBRA_ELEMENT_EVIDENCE_HPP
