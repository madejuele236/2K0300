#ifndef LS2K_VISION_BEV_IMAGE_SEGMENT_CONNECTIVITY_HPP
#define LS2K_VISION_BEV_IMAGE_SEGMENT_CONNECTIVITY_HPP

#include "port/camera_frame_types.hpp"
#include "vision/bev/bev_projector.hpp"
#include "vision/bev/bev_segment_connectivity.hpp"

namespace ls2k::vision {

class BEVImageSegmentConnectivity final : public BEVSegmentConnectivityQuery {
public:
    BEVImageSegmentConnectivity(const port::CameraPixelFrameView& frame,
                                const BEVProjector& projector,
                                const port::BEVBoundaryParameters& boundary_params)
        : frame_(frame), projector_(projector), boundary_params_(boundary_params) {}

    BEVSegmentConnectivityResult Evaluate(
        const port::BEVPoint& from,
        const port::BEVPoint& to,
        BEVSegmentVisibilityPolicy policy) const override;

private:
    const port::CameraPixelFrameView& frame_;
    const BEVProjector& projector_;
    const port::BEVBoundaryParameters& boundary_params_;
};

}  // namespace ls2k::vision

#endif  // LS2K_VISION_BEV_IMAGE_SEGMENT_CONNECTIVITY_HPP
