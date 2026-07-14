#ifndef LS2K_VISION_ML_CLASS_MAPPING_HPP
#define LS2K_VISION_ML_CLASS_MAPPING_HPP

#include <string>

#include "port/ml_types.hpp"
#include "port/runtime_parameter_types.hpp"

namespace ls2k::vision::ml {

port::MlAction ParseMlAction(const std::string& token);
port::MlAction MapMlClass(int class_id,
                          const port::MlClassMappingParameters& mapping);

}  // namespace ls2k::vision::ml

#endif
