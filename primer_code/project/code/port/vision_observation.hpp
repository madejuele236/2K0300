#pragma once

#include "port/vision_facts.hpp"
#include "port/vision_geometry.hpp"

#include <cstdint>

namespace cv {
class Mat;
}

namespace primer::port::vision {

using BinaryImage = uint8_t[kProcessedHeight][kProcessedWidth];
using SourceLine = int[kSourceHeight];
using RowDistance = float[kProcessedHeight];

const BinaryImage &ObserveBinaryImage();
const SourceLine &ObserveLeftSideline();
const SourceLine &ObserveRightSideline();
const SourceLine &ObserveMidline();
const ElementFacts &ObserveElementFacts();
const ImageInformation &ObserveImageInformation();
const CornerPoint &ObserveLeftLowCorner();
const CornerPoint &ObserveLeftHighCorner();
const CornerPoint &ObserveRightLowCorner();
const CornerPoint &ObserveRightHighCorner();
const CornerPoint &ObserveLeftHighCornerSecondary();
const CornerPoint &ObserveRightHighCornerSecondary();
const RowDistance &ObserveRowDistance();
const cv::Mat &ObserveResizedFrame();
const float &ObserveDirectionError();
const float &ObserveDistance();
const float &ObserveRoundaboutYawError();
const float &ObserveBlackRatio();
const uint16_t &ObserveJumpPoint();
const int &ObserveMaxlongColumn();
const int &ObserveLongMax();
const int &ObserveJumpPointSecondary();
const int &ObservePictureWhite();
const int &ObservePictureBlack();
const int &ObserveRedFindX();
const int &ObserveRedFindY();

}  // namespace primer::port::vision
