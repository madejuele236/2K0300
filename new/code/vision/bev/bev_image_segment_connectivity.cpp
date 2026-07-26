#include "vision/bev/bev_image_segment_connectivity.hpp"

#include <algorithm>
#include <cmath>

#include "vision/image/luma_sampler.hpp"
#include "vision/image/otsu_threshold.hpp"

namespace ls2k::vision {
namespace {

bool Inside(const port::ImagePoint& point,
            const port::CameraPixelFrameView& frame) {
    return std::isfinite(point.row_px) && std::isfinite(point.col_px) &&
           point.row_px >= 0.0F && point.col_px >= 0.0F &&
           point.row_px <= static_cast<float>(frame.height - 1) &&
           point.col_px <= static_cast<float>(frame.width - 1);
}

bool ClipRange(float p, float q, float& begin, float& end) {
    if (p == 0.0F) {
        return q >= 0.0F;
    }
    const float ratio = q / p;
    if (p < 0.0F) {
        if (ratio > end) {
            return false;
        }
        begin = std::max(begin, ratio);
    } else {
        if (ratio < begin) {
            return false;
        }
        end = std::min(end, ratio);
    }
    return begin <= end;
}

bool ClipImageSegmentToFrame(const port::CameraPixelFrameView& frame,
                             const port::ImagePoint& from,
                             const port::ImagePoint& to,
                             port::ImagePoint& clipped_from,
                             port::ImagePoint& clipped_to) {
    const float delta_col = to.col_px - from.col_px;
    const float delta_row = to.row_px - from.row_px;
    float begin = 0.0F;
    float end = 1.0F;
    if (!ClipRange(-delta_col, from.col_px, begin, end) ||
        !ClipRange(delta_col,
                   static_cast<float>(frame.width - 1) - from.col_px,
                   begin,
                   end) ||
        !ClipRange(-delta_row, from.row_px, begin, end) ||
        !ClipRange(delta_row,
                   static_cast<float>(frame.height - 1) - from.row_px,
                   begin,
                   end)) {
        return false;
    }
    clipped_from = {from.row_px + begin * delta_row,
                    from.col_px + begin * delta_col};
    clipped_to = {from.row_px + end * delta_row,
                  from.col_px + end * delta_col};
    return Inside(clipped_from, frame) && Inside(clipped_to, frame);
}

int PixelIndex(float coordinate, int extent) {
    return std::clamp(static_cast<int>(std::floor(coordinate + 0.5F)),
                      0,
                      extent - 1);
}

bool VisitPixel(const port::CameraPixelFrameView& frame,
                const port::OtsuThresholdState& threshold,
                int row,
                int col,
                BEVSegmentConnectivityResult& result) {
    std::uint8_t y = 0U;
    if (!SampleLumaAt(frame,
                      static_cast<float>(row),
                      static_cast<float>(col),
                      y)) {
        result.status = BEVSegmentConnectivityStatus::kUnobservable;
        return false;
    }
    ++result.sampled_point_count;
    if (!IsOtsuWhite(y, threshold)) {
        result.status = BEVSegmentConnectivityStatus::kBlocked;
        return false;
    }
    return true;
}

bool ImageSegmentIsWhite(const port::CameraPixelFrameView& frame,
                         const port::OtsuThresholdState& threshold,
                         const port::ImagePoint& from,
                         const port::ImagePoint& to,
                         BEVSegmentConnectivityResult& result) {
    int col = PixelIndex(from.col_px, frame.width);
    int row = PixelIndex(from.row_px, frame.height);
    const int end_col = PixelIndex(to.col_px, frame.width);
    const int end_row = PixelIndex(to.row_px, frame.height);

    const float x0 = from.col_px + 0.5F;
    const float y0 = from.row_px + 0.5F;
    const float x1 = to.col_px + 0.5F;
    const float y1 = to.row_px + 0.5F;
    const float dx = x1 - x0;
    const float dy = y1 - y0;
    const int step_col = dx > 0.0F ? 1 : (dx < 0.0F ? -1 : 0);
    const int step_row = dy > 0.0F ? 1 : (dy < 0.0F ? -1 : 0);
    constexpr float kInfinity = 1.0e30F;
    const float delta_t_col = step_col == 0 ? kInfinity : std::fabs(1.0F / dx);
    const float delta_t_row = step_row == 0 ? kInfinity : std::fabs(1.0F / dy);
    float next_t_col = kInfinity;
    if (step_col > 0) {
        next_t_col = (static_cast<float>(col + 1) - x0) / dx;
    } else if (step_col < 0) {
        next_t_col = (x0 - static_cast<float>(col)) / -dx;
    }
    float next_t_row = kInfinity;
    if (step_row > 0) {
        next_t_row = (static_cast<float>(row + 1) - y0) / dy;
    } else if (step_row < 0) {
        next_t_row = (y0 - static_cast<float>(row)) / -dy;
    }

    constexpr float kTieTolerance = 1.0e-6F;
    while (true) {
        if (!VisitPixel(frame, threshold, row, col, result)) {
            return false;
        }
        if (row == end_row && col == end_col) {
            return true;
        }
        if (std::fabs(next_t_col - next_t_row) <= kTieTolerance) {
            if (step_col != 0 &&
                !VisitPixel(frame, threshold, row, col + step_col, result)) {
                return false;
            }
            if (step_row != 0 &&
                !VisitPixel(frame, threshold, row + step_row, col, result)) {
                return false;
            }
            col += step_col;
            row += step_row;
            next_t_col += delta_t_col;
            next_t_row += delta_t_row;
        } else if (next_t_col < next_t_row) {
            col += step_col;
            next_t_col += delta_t_col;
        } else {
            row += step_row;
            next_t_row += delta_t_row;
        }
    }
}

}  // namespace

BEVSegmentConnectivityResult BEVImageSegmentConnectivity::Evaluate(
    const port::BEVPoint& from,
    const port::BEVPoint& to,
    BEVSegmentVisibilityPolicy policy) const {
    BEVSegmentConnectivityResult result{};
    if (!frame_.Valid() || !threshold_.valid) {
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
            port::ImagePoint clipped_from{};
            port::ImagePoint clipped_to{};
            if (!ClipImageSegmentToFrame(frame_,
                                         from_image,
                                         to_image,
                                         clipped_from,
                                         clipped_to)) {
                return result;
            }
            from_image = clipped_from;
            to_image = clipped_to;
            result.visible_segment_clipped = true;
        }
    }

    if (!ImageSegmentIsWhite(frame_,
                             threshold_,
                             from_image,
                             to_image,
                             result)) {
        return result;
    }
    if (result.sampled_point_count == 0U) {
        return result;
    }
    result.status = BEVSegmentConnectivityStatus::kConnected;
    return result;
}

}  // namespace ls2k::vision
