#ifndef LS2K_VISION_BEV_SEGMENT_CONNECTIVITY_HPP
#define LS2K_VISION_BEV_SEGMENT_CONNECTIVITY_HPP

#include <cstddef>

#include "port/bev_geometry_types.hpp"

namespace ls2k::vision {

enum class BEVSegmentConnectivityStatus {
    kConnected,
    kBlocked,
    kUnobservable,
};

enum class BEVSegmentVisibilityPolicy {
    kRequireFullSegment,
    kAllowFromEndpointClip,
};

struct BEVSegmentConnectivityResult {
    BEVSegmentConnectivityStatus status = BEVSegmentConnectivityStatus::kUnobservable;
    std::size_t sampled_point_count = 0U;
    bool visible_segment_clipped = false;
};

class BEVSegmentConnectivityQuery {
public:
    virtual ~BEVSegmentConnectivityQuery() = default;

    virtual BEVSegmentConnectivityResult Evaluate(
        const port::BEVPoint& from,
        const port::BEVPoint& to,
        BEVSegmentVisibilityPolicy policy) const = 0;
};

}  // namespace ls2k::vision

#endif  // LS2K_VISION_BEV_SEGMENT_CONNECTIVITY_HPP
