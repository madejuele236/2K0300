#pragma once

#include "../vision_stage_contracts.hpp"
#include "../../../port/vision_stage_ports.hpp"

extern uint8_t Image_Use[LCDH_1][LCDW_1];
extern YuanSu Flag;
extern imageInformation imgInfo;
extern cv::Mat lq_frame;
extern cv::Mat grayFrame;
extern cv::Mat resizedFrame;

static primer::port::VisionCameraPort &cam = primer::port::VisionCamera();
