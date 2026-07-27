#ifndef LS2K_VISION_ELEMENTS_CROSS_STRAIGHT_PATH_PLANNER_HPP
#define LS2K_VISION_ELEMENTS_CROSS_STRAIGHT_PATH_PLANNER_HPP

#include <vector>

#include "port/runtime_parameter_types.hpp"
#include "port/visual_element_evidence_types.hpp"
#include "port/visual_reference_orchestration_types.hpp"
#include "vision/bev/bev_row_facts.hpp"
#include "vision/bev/bev_segment_connectivity.hpp"

namespace ls2k::vision {

/// Build the current-frame straight-through route for a detected Cross.
///
/// Route identity comes from the unique origin-connected white run and is then
/// continued by observed white-run topology. Geometry combines a stable entry
/// center line, a slope-continuous center guide through the observed in-Cross
/// corridor, and the bounded far-exit center line.
port::VisualReferenceCandidate BuildCrossStraightPathCandidate(
    const std::vector<BEVSimpleRowScan>& rows,
    const port::CrossExitElementEvidence& evidence,
    const port::RuntimeParameters& params,
    const BEVSegmentConnectivityQuery& connectivity);

}  // namespace ls2k::vision

#endif  // LS2K_VISION_ELEMENTS_CROSS_STRAIGHT_PATH_PLANNER_HPP
