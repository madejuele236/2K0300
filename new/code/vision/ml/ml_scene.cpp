#include "vision/ml/ml_scene.hpp"

#include <algorithm>
#include <cmath>

#include "port/bev_reference_path_utils.hpp"
#include "vision/ml/ml_path_generator.hpp"

namespace ls2k::vision::ml {
namespace {

void PopulateObservationTelemetry(const MlObservation* observation,
                                  const port::RuntimeParameters& params,
                                  port::MlTelemetrySnapshot& telemetry) {
    telemetry.enabled = params.ml.enabled;
    telemetry.maneuver_enabled = params.ml.maneuver.enabled;
    if (observation == nullptr) {
        return;
    }
    telemetry.detector_valid = observation->detector_valid;
    telemetry.detector = observation->detector;
    telemetry.roi = observation->roi;
    telemetry.descriptor = observation->descriptor;
    telemetry.replay = observation->replay;
    telemetry.classification = observation->classification;
    telemetry.mapped_action = observation->mapped_action;
    telemetry.artifact_candidate_id = observation->artifact_candidate_id;
    telemetry.template_codes_sha256 = observation->artifact_sha256;
    telemetry.artifact_prototype_count = observation->artifact_item_count;
    telemetry.detector_us = observation->detector_us;
    telemetry.roi_us = observation->roi_us;
    telemetry.replay_us = observation->classifier_us;
    telemetry.classifier_us = observation->classifier_us;
    telemetry.total_us = observation->total_us;
}

void PopulateManeuverTelemetry(const port::MlSceneMemory& memory,
                               uint64_t now_ms,
                               port::MlTelemetrySnapshot& telemetry) {
    telemetry.phase = memory.phase;
    telemetry.active = memory.phase == port::MlScenePhase::kActive;
    telemetry.locked_action = memory.locked_action;
    telemetry.traveled_forward_m = static_cast<float>(memory.traveled_forward_m);
    telemetry.elapsed_ms = now_ms >= memory.maneuver_start_ms
        ? now_ms - memory.maneuver_start_ms : 0U;
}

void EnterCooldown(uint64_t now_ms, const char* reason, MlManeuverResult& out) {
    out.next_memory.phase = port::MlScenePhase::kCooldown;
    out.next_memory.confirmation = {};
    out.next_memory.locked_action = port::MlAction::kUnmapped;
    out.next_memory.maneuver_start_ms = 0U;
    out.next_memory.distance_cursor_ms = 0U;
    out.next_memory.traveled_forward_m = 0.0;
    out.next_memory.cooldown_start_ms = now_ms;
    out.next_memory.cooldown_reason = reason;
    out.telemetry.phase = port::MlScenePhase::kCooldown;
    out.telemetry.reason = reason;
    out.telemetry.active = false;
}

port::BEVReferencePath BuildCurrentPath(const MlManeuverInput& input,
                                        const port::MlManeuverParameters& params,
                                        port::MlAction action) {
    if (input.road_path_facts == nullptr) {
        return {};
    }
    return BuildMlBoundaryOffsetPath(
        action,
        *input.road_path_facts,
        static_cast<std::size_t>(params.min_boundary_samples),
        static_cast<float>(params.path_outward_offset_m));
}

port::VisualReferenceCandidate MakeMlCandidate(const port::BEVReferencePath& path) {
    port::VisualReferenceCandidate out{};
    out.present = path.mode == port::ReferenceMode::kMlBoundaryOffset &&
                  port::CountFiniteReferenceSamples(path) > 0U;
    out.kind = port::VisualReferenceCandidateKind::kMlGrounded;
    out.reference_path = path;
    out.source = "ml_boundary_offset";
    out.reason = out.present ? "ml_active" : "ml_path_absent";
    return out;
}

bool IntegrateForwardDistance(const port::MotionHistory* history,
                              uint64_t target_time_ms,
                              const port::MotionOdometryParameters& odometry,
                              const port::MlManeuverParameters& params,
                              port::MlSceneMemory& memory,
                              const char*& reason) {
    if (history == nullptr || history->count < 2U) {
        reason = "motion_history_unavailable";
        return false;
    }
    if (!(odometry.encoder_ticks_to_meter > 0.0) ||
        !std::isfinite(odometry.encoder_ticks_to_meter)) {
        reason = "encoder_scale_unavailable";
        return false;
    }
    if (target_time_ms < memory.distance_cursor_ms) {
        reason = "time_regressed";
        return false;
    }

    const uint64_t newest_time = history->OldestOffset(history->count - 1U).time_ms;
    if (newest_time <= memory.distance_cursor_ms) {
        reason = "waiting_for_sample";
        return true;
    }
    const uint64_t integration_target = std::min(target_time_ms, newest_time);
    const uint64_t max_gap = static_cast<uint64_t>(std::max(1, params.max_integration_gap_ms));
    uint64_t cursor = memory.distance_cursor_ms;

    for (std::size_t index = 1U;
         index < history->count && cursor < integration_target;
         ++index) {
        const port::MotionHistorySample& prev = history->OldestOffset(index - 1U);
        const port::MotionHistorySample& curr = history->OldestOffset(index);
        if (curr.time_ms <= cursor) {
            continue;
        }
        if (prev.time_ms > cursor || curr.time_ms <= prev.time_ms ||
            curr.time_ms - prev.time_ms > max_gap) {
            reason = "motion_history_gap";
            return false;
        }
        if (!curr.encoder_valid) {
            reason = "encoder_invalid";
            return false;
        }
        const uint64_t segment_end = std::min(curr.time_ms, integration_target);
        const float fraction = static_cast<float>(segment_end - cursor) /
                               static_cast<float>(curr.time_ms - prev.time_ms);
        const float mean_ticks = 0.5F * static_cast<float>(
            curr.left_encoder_delta + curr.right_encoder_delta);
        memory.traveled_forward_m +=
            static_cast<double>(mean_ticks) * odometry.encoder_ticks_to_meter *
            static_cast<double>(fraction);
        cursor = segment_end;
        memory.distance_cursor_ms = cursor;
    }
    if (cursor < integration_target) {
        reason = "motion_history_gap";
        return false;
    }
    reason = "ok";
    return true;
}

}  // namespace

bool StepMlConfirmation(port::MlAction action,
                        bool requested_boundary_available,
                        int confirm_frames,
                        port::MlConfirmationState& state) {
    if ((action != port::MlAction::kLeft && action != port::MlAction::kRight) ||
        !requested_boundary_available || confirm_frames < 1) {
        state = {};
        return false;
    }
    if (state.pending_action != action) {
        state.pending_action = action;
        state.consecutive_frames = 1;
    } else {
        ++state.consecutive_frames;
    }
    return state.consecutive_frames >= confirm_frames;
}

MlManeuverResult StepMlManeuver(const MlManeuverInput& input,
                                const port::RuntimeParameters& params,
                                const port::MlSceneMemory& prior_memory) {
    MlManeuverResult out{};
    out.next_memory = prior_memory;
    PopulateObservationTelemetry(input.observation, params, out.telemetry);

    if (!params.ml.enabled || !params.ml.maneuver.enabled) {
        ResetMlSceneMemory(out.next_memory);
        out.next_memory.phase = port::MlScenePhase::kDisabled;
        out.telemetry.phase = port::MlScenePhase::kDisabled;
        out.telemetry.reason = params.ml.enabled ? "observe_only" : "disabled";
        return out;
    }

    if (prior_memory.phase == port::MlScenePhase::kCooldown) {
        const uint64_t elapsed = input.capture_time_ms >= prior_memory.cooldown_start_ms
            ? input.capture_time_ms - prior_memory.cooldown_start_ms : 0U;
        if (elapsed >= static_cast<uint64_t>(params.ml.maneuver.cooldown_ms)) {
            ResetMlSceneMemory(out.next_memory);
            out.telemetry.phase = port::MlScenePhase::kIdle;
            out.telemetry.reason = "cooldown_complete";
        } else {
            out.telemetry.phase = port::MlScenePhase::kCooldown;
            out.telemetry.reason = prior_memory.cooldown_reason;
        }
        return out;
    }

    if (prior_memory.phase == port::MlScenePhase::kActive) {
        const char* odometry_reason = "not_run";
        out.telemetry.odometry_valid = IntegrateForwardDistance(
            input.motion_history, input.capture_time_ms, params.motion_odometry,
            params.ml.maneuver, out.next_memory, odometry_reason);
        out.telemetry.odometry_reason = odometry_reason;
        PopulateManeuverTelemetry(out.next_memory, input.capture_time_ms, out.telemetry);

        if (out.next_memory.traveled_forward_m >= params.ml.maneuver.exit_forward_m) {
            EnterCooldown(input.capture_time_ms, "distance_complete", out);
            return out;
        }
        if (out.telemetry.elapsed_ms >=
            static_cast<uint64_t>(params.ml.maneuver.max_duration_ms)) {
            EnterCooldown(input.capture_time_ms, "timeout", out);
            return out;
        }

        const port::BEVReferencePath path = BuildCurrentPath(
            input, params.ml.maneuver, prior_memory.locked_action);
        out.candidate = MakeMlCandidate(path);
        out.telemetry.path_sample_count = port::CountFiniteReferenceSamples(path);
        out.telemetry.reason = out.candidate.present ? "active" : "active_path_unavailable";
        return out;
    }

    if (input.observation == nullptr || input.road_path_facts == nullptr) {
        out.next_memory.confirmation = {};
        out.next_memory.phase = port::MlScenePhase::kIdle;
        out.telemetry.phase = port::MlScenePhase::kIdle;
        out.telemetry.reason = "input_unavailable";
        return out;
    }
    const MlObservation& observation = *input.observation;
    if (!observation.accepted) {
        out.next_memory.confirmation = {};
        out.next_memory.phase = port::MlScenePhase::kIdle;
        out.telemetry.phase = port::MlScenePhase::kIdle;
        out.telemetry.reason = observation.reason;
        return out;
    }

    const port::MlAction action = observation.mapped_action;
    if (action == port::MlAction::kStraight || action == port::MlAction::kUnmapped) {
        out.next_memory.confirmation = {};
        out.next_memory.phase = port::MlScenePhase::kIdle;
        out.telemetry.phase = port::MlScenePhase::kIdle;
        out.telemetry.reason = action == port::MlAction::kStraight
            ? "straight_reset" : "unmapped_reset";
        return out;
    }

    const port::BEVReferencePath path = BuildCurrentPath(input, params.ml.maneuver, action);
    const bool boundary_available =
        port::CountFiniteReferenceSamples(path) >=
        static_cast<std::size_t>(params.ml.maneuver.min_boundary_samples);
    const MlClassificationPolicy classification_policy =
        SelectMlClassificationPolicy(observation.classification.backend, params.ml);
    const bool confirmed = StepMlConfirmation(
        action, boundary_available, classification_policy.confirm_frames,
        out.next_memory.confirmation);
    if (!boundary_available) {
        out.next_memory.phase = port::MlScenePhase::kIdle;
        out.telemetry.phase = port::MlScenePhase::kIdle;
        out.telemetry.reason = "requested_boundary_unavailable";
        return out;
    }

    out.telemetry.confirm_count = out.next_memory.confirmation.consecutive_frames;
    out.next_memory.phase = port::MlScenePhase::kCandidate;
    out.telemetry.phase = port::MlScenePhase::kCandidate;
    if (!confirmed) {
        out.telemetry.reason = "confirming";
        return out;
    }

    out.next_memory.phase = port::MlScenePhase::kActive;
    out.next_memory.confirmation = {};
    out.next_memory.locked_action = action;
    out.next_memory.maneuver_start_ms = input.capture_time_ms;
    out.next_memory.distance_cursor_ms = input.capture_time_ms;
    out.next_memory.traveled_forward_m = 0.0;
    PopulateManeuverTelemetry(out.next_memory, input.capture_time_ms, out.telemetry);
    out.telemetry.odometry_reason = "locked";
    out.telemetry.path_sample_count = port::CountFiniteReferenceSamples(path);
    out.telemetry.reason = "locked";
    out.candidate = MakeMlCandidate(path);
    return out;
}

void ResetMlSceneMemory(port::MlSceneMemory& memory) { memory = {}; }

}  // namespace ls2k::vision::ml
