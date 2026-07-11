#pragma once

#include "vision_facts.hpp"

namespace primer::vision {

struct VisionControlFacts {
    const YuanSu &elements;
    const imageInformation &image;
    const Guaidian &left_high_corner;
    const Guaidian &right_high_corner;
    const float (&row_distance)[LCDH_1];
    float direction_error;
    float steering_difference_error;
    uint16_t jump_point;
};

struct VisionPresentationFacts {
    const uint8_t (&binary_image)[LCDH_1][LCDW_1];
    const int (&left_sideline)[LCDH_0];
    const int (&right_sideline)[LCDH_0];
    const int (&midline)[LCDH_0];
    const YuanSu &elements;
    const imageInformation &image;
    const Guaidian &left_low_corner;
    const Guaidian &left_high_corner;
    const Guaidian &right_low_corner;
    const Guaidian &right_high_corner;
    const Guaidian &left_high_corner_secondary;
    const Guaidian &right_high_corner_secondary;
    const float (&row_distance)[LCDH_1];
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

VisionControlFacts ObserveVisionControlFacts();
VisionPresentationFacts ObserveVisionPresentationFacts();

int VisionDynamicForward();
void SetVisionDynamicForward(int value);

float VisionRoundaboutYaw();
void SetVisionRoundaboutYawCorrection(float corrected_yaw, float yaw_error);

}  // namespace primer::vision

int real_distance_to_row(float distance);
