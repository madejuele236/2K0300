#ifndef LS2K_VISION_BEV_REFERENCE_CONNECTIVITY_FILTER_HPP
#define LS2K_VISION_BEV_REFERENCE_CONNECTIVITY_FILTER_HPP

#include <cstddef>

#include "port/bev_reference_path_utils.hpp"
#include "vision/bev/bev_segment_connectivity.hpp"

namespace ls2k::vision {

// Keep the maximal ordered subset whose points are individually reachable
// from the last accepted point. A rejected point never becomes a predecessor;
// the next point is evaluated independently from the last accepted geometry.
inline std::size_t KeepConnectedReferenceSamples(
    port::BEVReferencePath& path,
    const BEVSegmentConnectivityQuery& connectivity) {
    port::BEVPoint previous{};
    bool first = true;
    std::size_t output_index = 0U;
    for (const port::BEVPathSample sample : path.sampled_path) {
        if (!port::IsFiniteReferenceSample(sample)) {
            continue;
        }
        const BEVSegmentVisibilityPolicy policy =
            first ? BEVSegmentVisibilityPolicy::kAllowFromEndpointClip
                  : BEVSegmentVisibilityPolicy::kRequireFullSegment;
        if (connectivity.Evaluate(previous, sample.point, policy).status !=
            BEVSegmentConnectivityStatus::kConnected) {
            continue;
        }
        path.sampled_path[output_index++] = sample;
        previous = sample.point;
        first = false;
    }
    for (std::size_t index = output_index; index < path.sampled_path.size(); ++index) {
        path.sampled_path[index] = {};
    }
    if (output_index == 0U) {
        path.mode = port::ReferenceMode::kNone;
    }
    return output_index;
}

}  // namespace ls2k::vision

#endif  // LS2K_VISION_BEV_REFERENCE_CONNECTIVITY_FILTER_HPP
