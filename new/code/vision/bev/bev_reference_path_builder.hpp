#ifndef LS2K_VISION_BEV_REFERENCE_PATH_BUILDER_HPP
#define LS2K_VISION_BEV_REFERENCE_PATH_BUILDER_HPP

#include <vector>

#include "port/bev_reference_types.hpp"
#include "port/runtime_parameter_types.hpp"
#include "vision/bev/bev_row_facts.hpp"
#include "vision/bev/bev_segment_connectivity.hpp"

namespace ls2k::vision {

/// Build an ordinary-road path by accepting the first image-connected candidate
/// in each row. Accepted samples are compacted while retaining their measured
/// forward coordinate.
port::BEVReferencePath BuildConnectedReferencePath(
    const std::vector<BEVSimpleRowScan>& rows,
    const port::RuntimeParameters& params,
    const BEVSegmentConnectivityQuery& connectivity_query);

port::BEVReferencePath BuildReferencePath(const std::vector<BEVSimpleRowScan>& rows,
                                          const port::RuntimeParameters& params,
                                          const BEVSegmentConnectivityQuery& connectivity_query);

}  // namespace ls2k::vision

#endif  // LS2K_VISION_BEV_REFERENCE_PATH_BUILDER_HPP
