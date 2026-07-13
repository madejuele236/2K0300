#include "vision/bev/bev_image_segment_connectivity.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "vision/image/luma_sampler.hpp"

namespace ls2k::vision {
namespace {

bool Inside(const port::ImagePoint& point, const port::CameraPixelFrameView& frame) {
    return std::isfinite(point.row_px) && std::isfinite(point.col_px) &&
           point.row_px >= 0.0F && point.col_px >= 0.0F &&
           point.row_px <= static_cast<float>(frame.height - 1) &&
           point.col_px <= static_cast<float>(frame.width - 1);
}

bool ClipFromEndpointToRectangle(port::ImagePoint& from,
                                 const port::ImagePoint& to,
                                 const port::CameraPixelFrameView& frame) {
    const float dr = to.row_px - from.row_px;
    const float dc = to.col_px - from.col_px;
    float enter = 0.0F;
    float exit = 1.0F;

    const auto clip = [&enter, &exit](float p, float q) {
        if (p == 0.0F) {
            return q >= 0.0F;
        }
        const float ratio = q / p;
        if (p < 0.0F) {
            enter = std::max(enter, ratio);
        } else {
            exit = std::min(exit, ratio);
        }
        return enter <= exit;
    };

    if (!clip(-dr, from.row_px) ||
        !clip(dr, static_cast<float>(frame.height - 1) - from.row_px) ||
        !clip(-dc, from.col_px) ||
        !clip(dc, static_cast<float>(frame.width - 1) - from.col_px)) {
        return false;
    }
    from.row_px += enter * dr;
    from.col_px += enter * dc;
    return Inside(from, frame);
}

}  // namespace

BEVSegmentConnectivityResult BEVImageSegmentConnectivity::Evaluate(
    const port::BEVPoint& from,
    const port::BEVPoint& to,
    BEVSegmentVisibilityPolicy policy) const {
    BEVSegmentConnectivityResult result{};
    if (!frame_.Valid()) {
        return result;
    }

    port::ImagePoint from_image{};
    port::ImagePoint to_image{};
    if (!projector_.ProjectVehicleToImage(from, from_image) ||
        !projector_.ProjectVehicleToImage(to, to_image)) {
        return result;
    }

    const bool from_inside = Inside(from_image, frame_);
    const bool to_inside = Inside(to_image, frame_);
    if (policy == BEVSegmentVisibilityPolicy::kRequireFullSegment) {
        if (!from_inside || !to_inside) {
            return result;
        }
    } else {
        if (!to_inside) {
            return result;
        }
        if (!from_inside) {
            if (!ClipFromEndpointToRectangle(from_image, to_image, frame_)) {
                return result;
            }
            result.visible_segment_clipped = true;
        }
    }

    const float dr = to_image.row_px - from_image.row_px;
    const float dc = to_image.col_px - from_image.col_px;
    const int interval_count = std::max(
        1, static_cast<int>(std::ceil(std::max(std::abs(dr), std::abs(dc)))));
    std::uint8_t previous = 0U;
    for (int i = 0; i <= interval_count; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(interval_count);
        std::uint8_t current = 0U;
        if (!SampleLumaAt(frame_,
                          from_image.row_px + t * dr,
                          from_image.col_px + t * dc,
                          current)) {
            result.status = BEVSegmentConnectivityStatus::kUnobservable;
            return result;
        }
        ++result.sampled_point_count;
        if (i > 0 && std::abs(static_cast<int>(current) - static_cast<int>(previous)) >=
                         boundary_params_.local_jump_min_y) {
            result.status = BEVSegmentConnectivityStatus::kBlocked;
            return result;
        }
        previous = current;
    }

    result.status = BEVSegmentConnectivityStatus::kConnected;
    return result;
}

}  // namespace ls2k::vision
