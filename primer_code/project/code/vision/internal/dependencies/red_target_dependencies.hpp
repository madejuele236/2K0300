#pragma once

#include "../../vision_facts.hpp"
#include "../legacy_vision_constants.hpp"
#include "../../../port/vision_stage_ports.hpp"

#include <vector>

#define MODEL_INPUT_WIDTH 40
#define MODEL_INPUT_HEIGHT 40

extern YuanSu Flag;
extern Guaidian L_h_guai, R_h_guai;
extern cv::Mat lq_frame;
extern float real_distance[60];
extern float real_picture_distance;
extern int roi_x_1;
extern int roi_y_1;
extern int ROI_SIZE;
extern int roix1;
extern int roiy1;
extern int resize_cx;
extern int resize_cy;

bool GenerateROI(const cv::Point &center, cv::Rect &roi, const cv::Mat &src);

#define Flash (primer::port::VisionParameters())
#define classifier (primer::port::VisionClassifier())
