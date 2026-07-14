#ifndef LS2K_VISION_ML_V9_REPLAY_HPP
#define LS2K_VISION_ML_V9_REPLAY_HPP

#include <string>

#include "port/ml_types.hpp"
#include "port/runtime_parameter_types.hpp"

namespace ls2k::vision::ml {

bool ValidateV9Artifact(const port::V9ArtifactView& artifact);
port::V9ReplayResult ReplayV9Descriptor(const port::V9Descriptor& query,
                                        const port::V9ArtifactView& artifact);
bool AcceptV9Result(const port::V9ReplayResult& result,
                    const port::MlV9Parameters& acceptance);
port::V9AcceptanceResult StepV9Acceptance(const port::V9ReplayResult& result,
                                          const port::MlV9Parameters& acceptance,
                                          port::V9AcceptanceState& state);
void ResetV9Acceptance(port::V9AcceptanceState& state);
port::MlAction ParseMlAction(const std::string& token);
port::MlAction MapV9Class(int class_id, const port::MlClassMappingParameters& mapping);

}  // namespace ls2k::vision::ml

#endif
