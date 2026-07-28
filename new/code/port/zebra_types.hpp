#ifndef LS2K_PORT_ZEBRA_TYPES_HPP
#define LS2K_PORT_ZEBRA_TYPES_HPP

#include <cstdint>
#include <string>

namespace ls2k::port {

enum class ZebraStopPhase {
    kWaitingForFirstDetection,
    kWaitingForClearance,
    kArmedForReentry,
    kStopDelay,
    kStopRequested,
};

struct ZebraStopMemory {
    ZebraStopPhase phase = ZebraStopPhase::kWaitingForFirstDetection;
    uint64_t last_detection_time_ms = 0;
    uint64_t stop_delay_start_time_ms = 0;
};

struct ZebraStopTelemetry {
    std::string frame_phase = "waiting_for_first_detection";
    std::string next_phase = "waiting_for_first_detection";
    std::string reason = "motion_session_inactive";
    bool motion_session_active = false;
    bool detected = false;
    uint64_t absence_elapsed_ms = 0;
    uint64_t stop_delay_elapsed_ms = 0;
    bool controlled_stop_requested = false;
};

}  // namespace ls2k::port

#endif  // LS2K_PORT_ZEBRA_TYPES_HPP
