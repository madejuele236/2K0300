#ifndef LS2K_VISION_ELEMENTS_ZEBRA_STOP_SCENE_HPP
#define LS2K_VISION_ELEMENTS_ZEBRA_STOP_SCENE_HPP

#include <cstdint>

#include "port/visual_element_evidence_types.hpp"
#include "port/zebra_types.hpp"

namespace ls2k::vision {

struct ZebraStopSceneInput {
    bool motion_session_active = false;
    bool zebra_detected = false;
    uint64_t capture_time_ms = 0;
};

struct ZebraStopSceneResult {
    port::ZebraStopMemory next_memory{};
    port::ZebraStopTelemetry telemetry{};
};

ZebraStopSceneResult StepZebraStopScene(
    const ZebraStopSceneInput& input,
    const port::BEVElementParameters& params,
    const port::ZebraStopMemory& prior_memory);

const char* ToString(port::ZebraStopPhase phase);

}  // namespace ls2k::vision

#endif  // LS2K_VISION_ELEMENTS_ZEBRA_STOP_SCENE_HPP
