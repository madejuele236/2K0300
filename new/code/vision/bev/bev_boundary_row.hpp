#ifndef LS2K_VISION_BEV_BOUNDARY_ROW_HPP
#define LS2K_VISION_BEV_BOUNDARY_ROW_HPP

#include <vector>

#include "vision/bev/bev_row_facts.hpp"

namespace ls2k::vision {

void ExtractSparseBoundaryRowFacts(const std::vector<BEVRowLumaSample>& samples,
                                   float min_span_width_m,
                                   BEVSimpleRowScan& row);

}  // namespace ls2k::vision

#endif  // LS2K_VISION_BEV_BOUNDARY_ROW_HPP
