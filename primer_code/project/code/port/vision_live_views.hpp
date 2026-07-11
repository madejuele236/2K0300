#pragma once

#include "port/vision_facts.hpp"
#include "port/vision_geometry.hpp"

#include <opencv2/core.hpp>

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

struct PresentationLiveView {
    const uint8_t (&binary_image)[kProcessedHeight][kProcessedWidth];
    const int (&left_sideline)[kSourceHeight];
    const int (&right_sideline)[kSourceHeight];
    const int (&midline)[kSourceHeight];
    const ElementFacts &elements;
    const ImageInformation &image;
    const CornerPoint &left_low_corner;
    const CornerPoint &left_high_corner;
    const CornerPoint &right_low_corner;
    const CornerPoint &right_high_corner;
    const CornerPoint &left_high_corner_secondary;
    const CornerPoint &right_high_corner_secondary;
    const float (&row_distance)[kProcessedHeight];
    const cv::Mat &resized_frame;
    const float &direction_error;
    const float &distance;
    const float &roundabout_yaw_error;
    const float &black_ratio;
    const uint16_t &jump_point;
    const int &maxlong_column;
    const int &long_max;
    const int &jump_point_secondary;
    const int &picture_white;
    const int &picture_black;
    const int &red_find_x;
    const int &red_find_y;
};

}  // namespace primer::port::vision
