#pragma once

#include "../vision_stage_contracts.hpp"
#include "../../../port/vision_stage_ports.hpp"

extern cv::Mat grayFrame;
extern cv::Mat binaryFrame;
extern cv::Mat resizedFrame;
extern cv::Mat translatedFrame;
extern cv::Mat translationMatrix;

#define classifier (primer::port::VisionClassifier())
#define camera_server (primer::port::VisionStream())
#define cam (primer::port::VisionCamera())
