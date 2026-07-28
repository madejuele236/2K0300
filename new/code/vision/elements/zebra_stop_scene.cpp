#include "vision/elements/zebra_stop_scene.hpp"

namespace ls2k::vision {
namespace {

uint64_t ElapsedMs(uint64_t now_ms, uint64_t start_ms) {
    return now_ms >= start_ms ? now_ms - start_ms : 0;
}

}  // namespace

const char* ToString(port::ZebraStopPhase phase) {
    switch (phase) {
        case port::ZebraStopPhase::kWaitingForFirstDetection:
            return "waiting_for_first_detection";
        case port::ZebraStopPhase::kWaitingForClearance:
            return "waiting_for_clearance";
        case port::ZebraStopPhase::kArmedForReentry:
            return "armed_for_reentry";
        case port::ZebraStopPhase::kStopDelay:
            return "stop_delay";
        case port::ZebraStopPhase::kStopRequested:
            return "stop_requested";
    }
    return "waiting_for_first_detection";
}

ZebraStopSceneResult StepZebraStopScene(
    const ZebraStopSceneInput& input,
    const port::BEVElementParameters& params,
    const port::ZebraStopMemory& prior_memory) {
    ZebraStopSceneResult result{};
    result.telemetry.motion_session_active = input.motion_session_active;
    result.telemetry.detected = input.zebra_detected;
    result.telemetry.frame_phase = ToString(prior_memory.phase);

    if (!input.motion_session_active) {
        result.telemetry.reason = "motion_session_inactive";
        result.telemetry.next_phase = ToString(result.next_memory.phase);
        return result;
    }

    result.next_memory = prior_memory;
    switch (prior_memory.phase) {
        case port::ZebraStopPhase::kWaitingForFirstDetection:
            result.telemetry.reason = "waiting_for_first_detection";
            if (input.zebra_detected) {
                result.next_memory.phase =
                    port::ZebraStopPhase::kWaitingForClearance;
                result.next_memory.last_detection_time_ms =
                    input.capture_time_ms;
                result.telemetry.reason = "first_detection_recorded";
            }
            break;

        case port::ZebraStopPhase::kWaitingForClearance:
            if (input.zebra_detected) {
                result.next_memory.last_detection_time_ms =
                    input.capture_time_ms;
                result.telemetry.reason = "zebra_still_present";
            } else {
                result.telemetry.absence_elapsed_ms =
                    ElapsedMs(input.capture_time_ms,
                              prior_memory.last_detection_time_ms);
                if (result.telemetry.absence_elapsed_ms >=
                    static_cast<uint64_t>(
                        params.zebra_reentry_arm_absence_ms)) {
                    result.next_memory.phase =
                        port::ZebraStopPhase::kArmedForReentry;
                    result.telemetry.reason = "reentry_armed";
                } else {
                    result.telemetry.reason =
                        "waiting_for_continuous_absence";
                }
            }
            break;

        case port::ZebraStopPhase::kArmedForReentry:
            result.telemetry.reason = "waiting_for_reentry";
            if (input.zebra_detected) {
                result.next_memory.stop_delay_start_time_ms =
                    input.capture_time_ms;
                if (params.zebra_controlled_stop_delay_ms == 0) {
                    result.next_memory.phase =
                        port::ZebraStopPhase::kStopRequested;
                    result.telemetry.reason = "stop_delay_complete";
                } else {
                    result.next_memory.phase =
                        port::ZebraStopPhase::kStopDelay;
                    result.telemetry.reason = "reentry_detected";
                }
            }
            break;

        case port::ZebraStopPhase::kStopDelay:
            result.telemetry.stop_delay_elapsed_ms =
                ElapsedMs(input.capture_time_ms,
                          prior_memory.stop_delay_start_time_ms);
            if (result.telemetry.stop_delay_elapsed_ms >=
                static_cast<uint64_t>(
                    params.zebra_controlled_stop_delay_ms)) {
                result.next_memory.phase =
                    port::ZebraStopPhase::kStopRequested;
                result.telemetry.reason = "stop_delay_complete";
            } else {
                result.telemetry.reason = "stop_delay_active";
            }
            break;

        case port::ZebraStopPhase::kStopRequested:
            result.telemetry.reason = "controlled_stop_requested";
            break;
    }

    result.telemetry.controlled_stop_requested =
        result.next_memory.phase == port::ZebraStopPhase::kStopRequested;
    result.telemetry.next_phase = ToString(result.next_memory.phase);
    return result;
}

}  // namespace ls2k::vision
