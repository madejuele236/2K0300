#ifndef LS2K_VISION_ML_RED_RECTANGLE_DETECTOR_HPP
#define LS2K_VISION_ML_RED_RECTANGLE_DETECTOR_HPP

#include <vector>

#include "port/camera_frame_types.hpp"
#include "port/ml_types.hpp"
#include "port/runtime_parameter_types.hpp"
#include "vision/bev/bev_projector.hpp"

namespace ls2k::vision::ml {

struct MlRedRectangleProjectionEntry {
    float forward_m = 0.0F;
    float lateral_m = 0.0F;
    port::ImagePoint image{};
    bool sampleable = false;
};

struct MlRedRectangleProjectionLut {
    bool valid = false;
    port::BEVProjectorCalibration calibration{};
    int frame_width = 0;
    int frame_height = 0;
    int rows = 0;
    int cols = 0;
    double search_forward_min_m = 0.0;
    double search_forward_max_m = 0.0;
    double search_lateral_limit_m = 0.0;
    double grid_forward_step_m = 0.0;
    double grid_lateral_step_m = 0.0;
    std::vector<MlRedRectangleProjectionEntry> entries{};
};

port::MlOrientedRectangle DetectRedRectangle(
    const port::CameraPixelFrameView& frame,
    const BEVProjector& projector,
    const port::MlRoiParameters& params,
    MlRedRectangleProjectionLut* projection_lut = nullptr);

}  // namespace ls2k::vision::ml

#endif
