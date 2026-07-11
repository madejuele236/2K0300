#pragma once

#include "../../vision_facts.hpp"
#include "../legacy_vision_constants.hpp"
#include "../../../port/vision_stage_ports.hpp"

#include <vector>

extern int Left_Sideline[LCDH_0], Right_Sideline[LCDH_0];
extern std::vector<RedObject> red_objects;
extern Guaidian L_h_guai, R_h_guai;
extern YuanSu Flag;
extern imageInformation imgInfo;
extern cv::Mat resizedFrame;
extern float real_distance[60];
extern float distance_picture;
extern int resize_cx, resize_cy;

int real_distance_to_row(float distance);
float xielv_sideline(int x1, int y1, int x2, int y2, char data);
void DetectRedBlock(cv::Mat &src, int roi_x, int roi_y, int width, int height);

struct LegacySpeedBinding {
    operator float() const { return primer::port::VisionCurrentSpeed(); }
};

static constexpr LegacySpeedBinding Now_Speed{};
