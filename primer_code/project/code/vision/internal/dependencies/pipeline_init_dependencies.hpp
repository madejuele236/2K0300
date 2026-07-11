#pragma once

#include "../vision_stage_contracts.hpp"
#include "../../../port/vision_stage_ports.hpp"

extern cv::Mat grayFrame;
extern cv::Mat binaryFrame;
extern cv::Mat resizedFrame;
extern cv::Mat translatedFrame;
extern cv::Mat translationMatrix;

static primer::port::VisionClassifierPort &classifier =
    primer::port::VisionClassifier();
static primer::port::VisionStreamPort &camera_server = primer::port::VisionStream();
static primer::port::VisionCameraPort &cam = primer::port::VisionCamera();
