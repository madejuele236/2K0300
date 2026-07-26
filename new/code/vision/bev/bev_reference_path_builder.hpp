#ifndef LS2K_VISION_BEV_REFERENCE_PATH_BUILDER_HPP
#define LS2K_VISION_BEV_REFERENCE_PATH_BUILDER_HPP

#include <array>
#include <vector>

#include "port/bev_reference_types.hpp"
#include "port/runtime_parameter_types.hpp"
#include "vision/bev/bev_row_facts.hpp"
#include "vision/bev/bev_segment_connectivity.hpp"

namespace ls2k::vision {

struct BEVRoadPathPointFact {
    bool present = false;
    port::BEVPoint point{};
    float confidence = 0.0F;
};

/// Selected ordinary-road facts. Entries at the same index come from the same
/// accepted row candidate. A missing boundary remains absent; it is never
/// reconstructed from the center or the opposite boundary.
struct BEVRoadPathFacts {
    std::array<BEVRoadPathPointFact, port::kBevReferenceSampleCount> center{};
    std::array<BEVRoadPathPointFact, port::kBevReferenceSampleCount> actual_left_boundary{};
    std::array<BEVRoadPathPointFact, port::kBevReferenceSampleCount> actual_right_boundary{};
};

enum class ObservedBoundarySide {
    kLeft,
    kRight,
};

BEVRoadPathFacts BuildConnectedRoadPathFacts(
    const std::vector<BEVSimpleRowScan>& rows,
    const port::RuntimeParameters& params,
    const BEVSegmentConnectivityQuery& connectivity_query);

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

/// Build a literal observed-boundary path for a caller-selected side. The
/// selected facts are copied without offsetting or smoothing. If fewer than
/// min_boundary_samples are available, the returned path remains kNone.
port::BEVReferencePath BuildObservedBoundaryReferencePath(
    const BEVRoadPathFacts& facts,
    ObservedBoundarySide side,
    std::size_t min_boundary_samples);

}  // namespace ls2k::vision

#endif  // LS2K_VISION_BEV_REFERENCE_PATH_BUILDER_HPP
