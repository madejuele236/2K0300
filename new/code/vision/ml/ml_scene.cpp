#include "vision/ml/ml_scene.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "vision/ml/ml_inertial_path_tracker.hpp"
#include "vision/ml/ml_path_generator.hpp"
#include "vision/ml/ml_reference_adapter.hpp"
#include "vision/ml/red_rectangle_detector.hpp"
#include "vision/ml/roi_sampler.hpp"
#include "vision/ml/ml_class_mapping.hpp"

namespace ls2k::vision::ml {
namespace {

using MlClock = std::chrono::steady_clock;

std::uint32_t ElapsedUs(MlClock::time_point start, MlClock::time_point end) {
    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    return static_cast<std::uint32_t>(std::min<std::int64_t>(
        std::max<std::int64_t>(elapsed, 0), std::numeric_limits<std::uint32_t>::max()));
}

void PopulateLockedTelemetry(const port::MlSceneMemory& memory,
                             port::MlTelemetrySnapshot& telemetry) {
    telemetry.phase = memory.phase;
    telemetry.active = memory.phase == port::MlScenePhase::kActive;
    telemetry.locked_action = memory.locked.action;
    telemetry.anchor = memory.locked.anchor;
    telemetry.pose_delta = memory.tracker.pose_delta;
    telemetry.path_sample_count = memory.locked.marker_path.sample_count;
}

void EnterCooldown(uint64_t now_ms, const char* reason, MlSceneResult& out) {
    out.next_memory.phase = port::MlScenePhase::kCooldown;
    out.next_memory.cooldown_start_ms = now_ms;
    out.next_memory.confirmation = {};
    out.next_memory.locked = {};
    MlInertialPathTracker::Reset(out.next_memory.tracker);
    out.telemetry.phase = port::MlScenePhase::kCooldown;
    out.telemetry.reason = reason;
    out.telemetry.active = false;
    // The frame began in Active. Release the ML candidate immediately, while
    // keeping cross/circle out of this frame's arbitration so ordinary line or
    // the existing hold contract is the direct takeover path.
    out.suppress_other_scene_candidates = true;
}

port::VisualReferenceCandidate MakeMlCandidate(const port::BEVReferencePath& path) {
    port::VisualReferenceCandidate out{};
    out.present = path.mode == port::ReferenceMode::kMlObservedBoundary &&
                  path.sampled_path[0].present;
    out.kind = port::VisualReferenceCandidateKind::kMlGrounded;
    out.reference_path = path;
    out.source = "ml_observed_boundary";
    out.reason = out.present ? "ml_inertial_path_active" : "ml_path_absent";
    return out;
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

MlSceneResult StepMlScene(const MlSceneInput& input,
                          const port::RuntimeParameters& params,
                          const port::MlSceneMemory& prior_memory) {
    MlSceneResult out{};
    out.next_memory = prior_memory;
    out.telemetry.enabled = params.ml.enabled;
    if (input.classifier != nullptr) {
        out.telemetry.artifact_candidate_id = input.classifier->ArtifactId();
        out.telemetry.template_codes_sha256 = input.classifier->ArtifactSha256();
        out.telemetry.artifact_prototype_count = input.classifier->ArtifactItemCount();
    }
    if (!params.ml.enabled) {
        ResetMlSceneMemory(out.next_memory);
        out.next_memory.phase = port::MlScenePhase::kDisabled;
        out.telemetry.phase = port::MlScenePhase::kDisabled;
        out.telemetry.reason = "disabled";
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
            out.telemetry.reason = "cooldown";
        }
        return out;
    }

    if (prior_memory.phase == port::MlScenePhase::kActive) {
        PopulateLockedTelemetry(prior_memory, out.telemetry);
        if (input.motion_history == nullptr) {
            EnterCooldown(input.capture_time_ms, "motion_history_unavailable", out);
            return out;
        }
        if (input.capture_time_ms < prior_memory.locked.lock_time_ms ||
            input.capture_time_ms - prior_memory.locked.lock_time_ms >=
                static_cast<uint64_t>(params.ml.maneuver.max_duration_ms)) {
            EnterCooldown(input.capture_time_ms, "timeout", out);
            return out;
        }
        const port::MlInertialTrackerResult tracked = MlInertialPathTracker::Step(
            prior_memory.locked, *input.motion_history, input.capture_time_ms,
            params.motion_odometry, params.ml.maneuver, out.next_memory.tracker);
        out.telemetry.pose_delta = tracked.pose_delta;
        out.telemetry.progress_m = tracked.progress_m;
        out.telemetry.lateral_error_m = tracked.lateral_error_m;
        out.telemetry.heading_error_rad = tracked.heading_error_rad;
        out.telemetry.path_sample_count = tracked.remaining_sample_count;
        if (!tracked.valid) {
            EnterCooldown(input.capture_time_ms, tracked.reason, out);
            return out;
        }
        if (tracked.progress_m >= params.ml.maneuver.exit_forward_m &&
            std::fabs(tracked.lateral_error_m) <=
                params.ml.maneuver.exit_max_abs_lateral_error_m &&
            std::fabs(tracked.heading_error_rad) <=
                params.ml.maneuver.exit_max_abs_heading_error_rad) {
            EnterCooldown(input.capture_time_ms, "maneuver_complete", out);
            return out;
        }
        out.candidate = MakeMlCandidate(tracked.reference_path);
        out.suppress_other_scene_candidates = true;
        out.telemetry.active = true;
        out.telemetry.phase = port::MlScenePhase::kActive;
        out.telemetry.reason = "active";
        return out;
    }

    if (input.frame == nullptr || input.projector == nullptr ||
        input.road_path_facts == nullptr || input.classifier == nullptr) {
        out.next_memory.confirmation = {};
        out.next_memory.phase = port::MlScenePhase::kIdle;
        out.telemetry.reason = "input_unavailable";
        return out;
    }
    const MlClock::time_point total_start = MlClock::now();
    const MlClock::time_point detector_start = total_start;
    out.telemetry.detector = DetectRedRectangle(*input.frame,
                                                *input.projector,
                                                params.ml.roi,
                                                input.rectangle_projection_lut);
    const MlClock::time_point detector_end = MlClock::now();
    out.telemetry.detector_us = ElapsedUs(detector_start, detector_end);
    out.telemetry.detector_valid = out.telemetry.detector.valid;
    const MlClock::time_point roi_start = detector_end;
    if (out.telemetry.detector.valid) {
        out.telemetry.roi = SampleSquareRoi32(*input.frame, *input.projector,
                                             out.telemetry.detector,
                                             params.ml.roi);
    }
    const MlClock::time_point roi_end = MlClock::now();
    out.telemetry.roi_us = ElapsedUs(roi_start, roi_end);
    const MlClock::time_point classifier_start = roi_end;
    if (out.telemetry.roi.valid) {
        const port::MlClassifierOutput classification =
            input.classifier->Predict(out.telemetry.roi);
        out.telemetry.classification = classification.classification;
        out.telemetry.descriptor = classification.v9_descriptor;
        out.telemetry.replay = classification.v9_replay;
    }
    const MlClock::time_point classifier_end = MlClock::now();
    out.telemetry.replay_us = ElapsedUs(classifier_start, classifier_end);
    out.telemetry.classifier_us = out.telemetry.replay_us;
    out.telemetry.total_us = ElapsedUs(total_start, classifier_end);
    const MlClassificationPolicy classification_policy =
        SelectMlClassificationPolicy(out.telemetry.classification.backend, params.ml);
    if (!AcceptMlClassification(out.telemetry.classification, classification_policy)) {
        out.next_memory.confirmation = {};
        out.next_memory.phase = port::MlScenePhase::kIdle;
        out.telemetry.phase = port::MlScenePhase::kIdle;
        out.telemetry.reason = "not_accepted";
        return out;
    }
    const port::MlAction action = MapMlClass(out.telemetry.classification.class_id,
                                            params.ml.class_mapping);
    out.telemetry.mapped_action = action;
    if (action == port::MlAction::kStraight || action == port::MlAction::kUnmapped) {
        out.next_memory.confirmation = {};
        out.next_memory.phase = port::MlScenePhase::kIdle;
        out.telemetry.phase = port::MlScenePhase::kIdle;
        out.telemetry.reason = action == port::MlAction::kStraight ? "straight_reset" : "unmapped_reset";
        return out;
    }
    const ObservedBoundaryMlPathGenerator path_generator{};
    const port::BEVReferencePath boundary = path_generator.Generate({
        action,
        input.road_path_facts,
        static_cast<std::size_t>(params.ml.maneuver.min_boundary_samples)});
    const bool boundary_available = boundary.mode != port::ReferenceMode::kNone;
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
    out.next_memory.locked = LockObservedBoundaryToMarker(
        action, input.capture_time_ms, out.telemetry.detector, boundary);
    if (!out.next_memory.locked.valid) {
        out.next_memory.confirmation = {};
        out.next_memory.phase = port::MlScenePhase::kIdle;
        out.telemetry.phase = port::MlScenePhase::kIdle;
        out.telemetry.reason = "marker_lock_failed";
        return out;
    }
    MlInertialPathTracker::Start(out.next_memory.locked, out.next_memory.tracker);
    out.next_memory.phase = port::MlScenePhase::kActive;
    out.next_memory.confirmation = {};
    PopulateLockedTelemetry(out.next_memory, out.telemetry);
    out.telemetry.reason = "locked";
    out.candidate = MakeMlCandidate(boundary);
    out.suppress_other_scene_candidates = true;
    return out;
}

void ResetMlSceneMemory(port::MlSceneMemory& memory) { memory = {}; }

}  // namespace ls2k::vision::ml
