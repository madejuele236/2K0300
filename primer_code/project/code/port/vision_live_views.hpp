#pragma once

#include "port/vision_facts.hpp"
#include "port/vision_geometry.hpp"

namespace primer::port::vision {

struct ControlLiveView {
    const ElementFacts &elements;
    const ImageInformation &image;
    const CornerPoint &left_high_corner;
    const CornerPoint &right_high_corner;
    const float (&row_distance)[kProcessedHeight];
    const float &direction_error;
    const float &steering_difference_error;
    const uint16_t &jump_point;
};

}  // namespace primer::port::vision
