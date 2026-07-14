#ifndef LS2K_VISION_ML_RED_RECTANGLE_DETECTOR_HPP
#define LS2K_VISION_ML_RED_RECTANGLE_DETECTOR_HPP

#include "port/camera_frame_types.hpp"
#include "port/ml_types.hpp"
#include "port/runtime_parameter_types.hpp"
#include "vision/bev/bev_projector.hpp"

namespace ls2k::vision::ml {

port::MlOrientedRectangle DetectRedRectangle(
    const port::CameraPixelFrameView& frame,
    const BEVProjector& projector,
    const port::MlRoiParameters& params);

}  // namespace ls2k::vision::ml

#endif
