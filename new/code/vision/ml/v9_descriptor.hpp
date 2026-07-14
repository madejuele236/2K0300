#ifndef LS2K_VISION_ML_V9_DESCRIPTOR_HPP
#define LS2K_VISION_ML_V9_DESCRIPTOR_HPP

#include "port/ml_types.hpp"

namespace ls2k::vision::ml {

port::V9Descriptor BuildV9Descriptor(const port::MlGrayRoi32& roi);

}  // namespace ls2k::vision::ml

#endif
