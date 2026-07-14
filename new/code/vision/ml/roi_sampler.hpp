#ifndef LS2K_VISION_ML_ROI_SAMPLER_HPP
#define LS2K_VISION_ML_ROI_SAMPLER_HPP

#include "port/camera_frame_types.hpp"
#include "port/ml_types.hpp"
#include "vision/bev/bev_projector.hpp"

namespace ls2k::vision::ml {

port::MlGrayRoi32 SampleSquareRoi32(const port::CameraPixelFrameView& frame,
                                    const BEVProjector& projector,
                                    const port::MlOrientedRectangle& rectangle);

}  // namespace ls2k::vision::ml

#endif
