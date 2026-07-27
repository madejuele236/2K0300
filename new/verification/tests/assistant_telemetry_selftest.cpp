#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include "transport/assistant_protocol.hpp"
#include "observability/assistant_telemetry_view.hpp"

namespace {

struct TestFailure {
    std::string message;
};

void Expect(bool condition, const std::string& message) {
    if (!condition) {
        throw TestFailure{message};
    }
}

bool Contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

void WriteStrictJsonArtifact(const std::string& filename, const std::string& json) {
    const char* artifact_dir = std::getenv("LS2K_STRICT_JSON_ARTIFACT_DIR");
    if (artifact_dir == nullptr || artifact_dir[0] == '\0') {
        return;
    }
    const std::string path = std::string(artifact_dir) + "/" + filename;
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    Expect(output.is_open(), "failed to open strict JSON artifact: " + path);
    output << json;
    Expect(output.good(), "failed to write strict JSON artifact: " + path);
}

ls2k::observability::ControlDebugSnapshot MakeSnapshot() {
    ls2k::observability::ControlDebugSnapshot snapshot{};
    snapshot.valid = true;
    snapshot.motion_phase = ls2k::control::MotionPhase::kRunning;
    snapshot.steering.otsu = {
        true, 87, ls2k::port::OtsuThresholdSource::kCached, 3U};
    snapshot.steering.element_evidence.cross_exit.present = true;
    snapshot.steering.element_evidence.cross_exit.forward_min_m = 0.20;
    snapshot.steering.element_evidence.cross_exit.forward_max_m = 0.42;
    snapshot.steering.element_evidence.cross_exit.lateral_min_m = -0.35;
    snapshot.steering.element_evidence.cross_exit.lateral_max_m = 0.36;
    snapshot.steering.element_evidence.cross_exit.sampleable_count = 120;
    snapshot.steering.element_evidence.cross_exit.boundary_jump_count = 0;
    snapshot.steering.element_evidence.cross_exit.boundary_span_count = 0;
    snapshot.steering.element_evidence.cross_exit.boundary_absent_row_count = 3;
    snapshot.steering.element_evidence.cross_exit.reason = "present";
    snapshot.steering.circle_v2.enabled = true;
    snapshot.steering.circle_v2.frame_phase = "approach";
    snapshot.steering.circle_v2.next_phase = "inner_trace";
    snapshot.steering.circle_v2.dir = "left";
    snapshot.steering.circle_v2.openings.left.available = true;
    snapshot.steering.circle_v2.openings.left.frontier_forward_m = 0.24F;
    snapshot.steering.circle_v2.openings.left.effective_lateral_m = -0.36F;
    snapshot.steering.circle_v2.openings.left.source =
        ls2k::port::CircleOpeningSource::kFovEdgeLowerBound;
    snapshot.steering.circle_v2.openings.left.outward_distance_m = 0.16F;
    snapshot.steering.circle_v2.openings.left.confirmed_forward_span_m = 0.12F;
    snapshot.steering.circle_v2.openings.left.origin_connected = true;
    snapshot.steering.circle_v2.openings.left.opposite_straight = true;
    ls2k::port::VisualElementEvidenceRecord record{};
    record.id = "synthetic_marker";
    record.present = true;
    record.confidence = 0.61F;
    record.reason = "synthetic_test_record";
    record.bounds.forward_min_m = 0.30F;
    record.bounds.forward_max_m = 0.70F;
    record.bounds.lateral_min_m = -0.42F;
    record.bounds.lateral_max_m = -0.18F;
    record.support.sampleable_count = 48;
    record.support.boundary_jump_count = 22;
    record.support.boundary_span_count = 9;
    record.candidate.reason = "not_built";
    snapshot.steering.element_evidence.records.push_back(record);
    snapshot.steering.visual_reference.present = true;
    snapshot.steering.visual_reference.source = "roadblock_bypass";
    snapshot.steering.visual_reference.reason = "special_visual_candidate_selected";
    snapshot.steering.visual_reference.candidate_count = 3;
    snapshot.steering.visual_reference.rejected_candidate_reason =
        "none_candidate_not_visual";
    snapshot.steering.reference.mode = "interval_center";
    snapshot.steering.reference.source = "roadblock_bypass";
    snapshot.steering.eligibility.usable = true;
    snapshot.steering.eligibility.leading_usable_samples = 5;
    snapshot.steering.eligibility.leading_min_forward_m = 0.05;
    snapshot.steering.eligibility.leading_max_forward_m = 0.25;
    snapshot.steering.eligibility.reason = "ok";
    snapshot.steering.lateral_error.computed = true;
    snapshot.steering.lateral_error.weighted_lateral_error_m = 0.015;
    snapshot.steering.lateral_error.weighted_sample_count = 4;
    snapshot.steering.lateral_error.weight_sum = 2.5;
    snapshot.steering.lateral_error.reason = "ok";
    snapshot.steering.reference_tracking_geometry.computed = true;
    snapshot.steering.reference_tracking_geometry.lateral_offset_m = 0.011;
    snapshot.steering.reference_tracking_geometry.heading_error_rad = -0.022;
    snapshot.steering.reference_tracking_geometry.curvature_m_inv = 0.33;
    snapshot.steering.reference_tracking_geometry.sample_count = 5;
    snapshot.steering.reference_tracking_geometry.reason = "ok";
    snapshot.steering.perception_health.projector_ok = true;
    snapshot.steering.perception_health.reason = "ok";
    snapshot.steering.reference_control.ready = true;
    snapshot.steering.reference_control.reason = "ready";
    snapshot.steering.safety_gate.veto_active = false;
    snapshot.steering.safety_gate.reason = "clear";
    snapshot.steering.degraded.active = false;
    snapshot.steering.degraded.reason = "none";
    snapshot.steering.yaw_control.valid = true;
    snapshot.steering.yaw_control.reason = "ok";
    snapshot.steering.yaw_control.turn_output_target = 0.12;
    snapshot.steering.yaw_control.lateral_term = 0.03;
    snapshot.steering.yaw_control.heading_term = -0.01;
    snapshot.steering.yaw_control.curvature_term = 0.10;
    snapshot.steering.ml.enabled = true;
    snapshot.steering.ml.maneuver_enabled = true;
    snapshot.steering.ml.takeover_selected = true;
    snapshot.steering.ml.artifact_candidate_id = "candidate-v9";
    snapshot.steering.ml.descriptor_config_hash = "descriptor-hash";
    snapshot.steering.ml.template_table_hash = "table-hash";
    snapshot.steering.ml.template_codes_sha256 = "codes-sha256";
    snapshot.steering.ml.artifact_prototype_count = 87;
    snapshot.steering.ml.detector_us = 101;
    snapshot.steering.ml.roi_us = 102;
    snapshot.steering.ml.descriptor_us = 103;
    snapshot.steering.ml.replay_us = 104;
    snapshot.steering.ml.classifier_us = 105;
    snapshot.steering.ml.total_us = 410;
    snapshot.steering.ml.detector_valid = true;
    snapshot.steering.ml.detector.valid = true;
    snapshot.steering.ml.detector.quality = 0.83F;
    snapshot.steering.ml.roi.valid = true;
    snapshot.steering.ml.descriptor.valid = true;
    snapshot.steering.ml.replay.valid = true;
    snapshot.steering.ml.replay.class_id = 1;
    snapshot.steering.ml.replay.best_distance = 7;
    snapshot.steering.ml.replay.margin = 11;
    snapshot.steering.ml.classification.valid = true;
    snapshot.steering.ml.classification.backend = ls2k::port::MlClassifierBackend::kTfliteInt8;
    snapshot.steering.ml.classification.class_id = 1;
    snapshot.steering.ml.classification.margin = 9;
    snapshot.steering.ml.classification.class_scores = {{-5, 17, 8}};
    snapshot.steering.ml.mapped_action = ls2k::port::MlAction::kLeft;
    snapshot.steering.ml.locked_action = ls2k::port::MlAction::kLeft;
    snapshot.steering.ml.phase = ls2k::port::MlScenePhase::kActive;
    snapshot.steering.ml.reason = "tracking";
    snapshot.steering.ml.confirm_count = 4;
    snapshot.steering.ml.active = true;
    snapshot.steering.ml.odometry_valid = true;
    snapshot.steering.ml.odometry_reason = "ok";
    snapshot.steering.ml.traveled_forward_m = 0.42F;
    snapshot.steering.ml.elapsed_ms = 1234U;
    snapshot.steering.ml.path_sample_count = 6;
    snapshot.steering.speed_selection_source = "ml_maneuver";
    snapshot.steering.effective_speed_target = 77.0;
    snapshot.steering.actuator.raw_turn_output = 12;
    snapshot.steering.actuator.applied_turn_output = 10;
    snapshot.raw_turn_output = 12;
    snapshot.applied_turn_output = 10;
    snapshot.apply_outcome = ls2k::safety::ControlApplyOutcome::kDriveCommandApplied;
    snapshot.actuators_armed = true;
    snapshot.last_confirmed_left_drive_pwm = 120;
    snapshot.last_confirmed_right_drive_pwm = 130;
    snapshot.last_confirmed_left_brushless_pwm = 500;
    snapshot.last_confirmed_right_brushless_pwm = 600;
    snapshot.tuning_mode_enabled = true;
    snapshot.turn_suppressed = false;
    snapshot.effective_speed_target = 1.2;
    snapshot.left_speed_target = 1.1;
    snapshot.right_speed_target = 1.3;
    snapshot.left_measured_speed = 1.0;
    snapshot.right_measured_speed = 1.25;
    snapshot.left_drive_pwm_command = 120;
    snapshot.right_drive_pwm_command = 130;
    snapshot.left_brushless_pwm_command = 500;
    snapshot.right_brushless_pwm_command = 600;
    snapshot.left_drive_pwm_unconstrained = 145.5;
    snapshot.right_drive_pwm_unconstrained = -50.25;
    snapshot.left_drive_pwm_requested = 146;
    snapshot.right_drive_pwm_requested = -50;
    snapshot.left_drive_pwm_desired = 146;
    snapshot.right_drive_pwm_desired = 0;
    snapshot.left_drive_pwm_step_limited = true;
    snapshot.right_drive_pwm_reverse_suppressed = true;
    snapshot.left_pid_error = 2.5;
    snapshot.right_pid_error = -1.5;
    snapshot.left_pid_integral = 9.0;
    snapshot.right_pid_integral = 4.0;
    snapshot.left_pid_integral_candidate = 11.5;
    snapshot.right_pid_integral_candidate = 2.5;
    snapshot.left_pid_anti_windup_active = true;
    snapshot.left_pid_anti_windup_reason =
        ls2k::control::WheelPidAntiWindupReason::kActuatorLimit;
    return snapshot;
}

void TestSnapshotFactsMapToAssistantView() {
    const ls2k::transport::AssistantTelemetryView telemetry =
        ls2k::observability::BuildAssistantTelemetryView(MakeSnapshot());
    Expect(telemetry.motion_phase == "RUNNING", "motion phase must be mapped");
    Expect(telemetry.element_evidence.cross_exit.present,
           "cross evidence presence must be copied");
    Expect(telemetry.element_evidence.cross_exit.reason == "present",
           "cross evidence reason must be copied");
    Expect(telemetry.element_evidence.records.size() == 1U,
           "generic element evidence records must be copied");
    Expect(telemetry.element_evidence.records[0].id == "synthetic_marker",
           "generic element evidence record id must be copied");
    Expect(telemetry.circle_v2.openings.left.available &&
               telemetry.circle_v2.openings.left.source ==
                   ls2k::port::CircleOpeningSource::kFovEdgeLowerBound,
           "CircleV2 opening facts must be copied to assistant telemetry");
    Expect(telemetry.visual_reference.present,
           "visual reference presence must be copied");
    Expect(telemetry.visual_reference.source == "roadblock_bypass",
           "visual reference source must be copied");
    Expect(telemetry.visual_reference.reason == "special_visual_candidate_selected",
           "visual reference reason must be copied");
    Expect(telemetry.visual_reference.candidate_count == 3,
           "visual reference candidate count must be copied");
    Expect(telemetry.visual_reference.rejected_candidate_reason ==
               "none_candidate_not_visual",
           "visual reference rejection reason must be copied");
    Expect(telemetry.otsu.valid && telemetry.otsu.threshold == 87 &&
               telemetry.otsu.source == ls2k::port::OtsuThresholdSource::kCached &&
               telemetry.otsu.stale_frames == 3U,
           "complete Otsu state must be copied");
    Expect(telemetry.reference.mode == "interval_center",
           "selected reference mode must be copied");
    Expect(telemetry.reference.source == "roadblock_bypass",
           "selected reference source must be copied");
    Expect(telemetry.reference_tracking_geometry.computed,
           "tracking geometry computed state must be copied");
    Expect(telemetry.reference_tracking_geometry.sample_count == 5,
           "tracking geometry sample count must be copied");
    Expect(telemetry.yaw_control.curvature_term > 0.09,
           "yaw curvature term must be copied");
    Expect(telemetry.ml.active && telemetry.ml.replay.class_id == 1,
           "ML scene and replay facts must be copied");
    Expect(telemetry.speed_selection_source == "ml_maneuver",
           "ML speed selection source must be copied");
    Expect(telemetry.left_drive_pwm_command == 120,
           "left drive PWM command must be copied");
    Expect(telemetry.right_drive_pwm_command == 130,
           "right drive PWM command must be copied");
    Expect(telemetry.left_brushless_pwm_command == 500,
           "left brushless PWM command must be copied");
    Expect(telemetry.right_brushless_pwm_command == 600,
           "right brushless PWM command must be copied");
    Expect(telemetry.left_drive_pwm_unconstrained == 145.5 &&
               telemetry.left_drive_pwm_requested == 146 &&
               telemetry.left_drive_pwm_desired == 146,
           "left PWM shaping chain must be copied");
    Expect(telemetry.right_drive_pwm_reverse_suppressed,
           "right reverse suppression must be copied");
    Expect(telemetry.left_pid_anti_windup_active &&
               telemetry.left_pid_anti_windup_reason == "actuator_limit",
           "left anti-windup state must be copied");
    Expect(telemetry.raw_turn_output == 12,
           "raw turn output must be copied");
    Expect(telemetry.applied_turn_output == 10,
           "applied turn output must be copied");
    Expect(telemetry.actuator_apply_outcome == "drive_command_applied",
           "actuator apply outcome must be copied");
    Expect(telemetry.actuators_armed && telemetry.last_confirmed_left_drive_pwm == 120 &&
               telemetry.last_confirmed_right_drive_pwm == 130,
           "assistant view must expose the last confirmed actuator state");
}

void TestAssistantTelemetryJsonEmitsVisualReferenceFacts() {
    const ls2k::transport::AssistantTelemetryView telemetry =
        ls2k::observability::BuildAssistantTelemetryView(MakeSnapshot());
    const std::string json = ls2k::transport::EncodeAssistantTelemetry(telemetry);
    WriteStrictJsonArtifact("assistant_telemetry.json", json);
    Expect(Contains(json,
                    "\"otsu\":{\"valid\":true,\"threshold\":87,\"source\":\"cached\",\"stale_frames\":3}"),
           "assistant telemetry must serialize the complete Otsu state");
    Expect(Contains(json, "\"element_evidence\":{\"cross_exit\":{\"present\":true"),
           "assistant telemetry must include element evidence object");
    Expect(!Contains(json, "\"cross_exit\":{\"present\":true,\"confidence\":"),
           "assistant telemetry must not expose removed cross confidence");
    Expect(Contains(json, "\"boundary_absent_row_count\":3,\"reason\":\"present\"}"),
           "cross telemetry must close after detection evidence");
    Expect(Contains(json, "\"records\":[{\"id\":\"synthetic_marker\""),
           "assistant telemetry must serialize generic element records");
    Expect(Contains(json,
                    "\"circle_v2\":{\"enabled\":true,\"frame_phase\":\"approach\",\"next_phase\":\"inner_trace\""),
           "assistant telemetry must serialize CircleV2 state");
    Expect(Contains(json,
                    "\"openings\":{\"left\":{\"available\":true,\"frontier_forward_m\":0.239999"),
           "assistant telemetry must serialize CircleV2 opening metrics");
    Expect(Contains(json, "\"source\":\"fov_edge_lower_bound\""),
           "assistant telemetry must serialize CircleV2 opening source");
    Expect(Contains(json, "\"boundary_span_count\":9"),
           "assistant telemetry must serialize generic record boundary support fields");
    Expect(Contains(json, "\"visual_reference\":{\"present\":true"),
           "assistant telemetry must include visual_reference object");
    Expect(Contains(json, "\"source\":\"roadblock_bypass\""),
           "assistant telemetry must include visual reference source");
    Expect(Contains(json, "\"reason\":\"special_visual_candidate_selected\""),
           "assistant telemetry must include visual reference reason");
    Expect(Contains(json, "\"candidate_count\":3"),
           "assistant telemetry must include visual reference candidate count");
    Expect(Contains(json,
                    "\"rejected_candidate_reason\":\"none_candidate_not_visual\""),
           "assistant telemetry must include rejected candidate reason");
    Expect(Contains(json,
                    "\"reference\":{\"mode\":\"interval_center\",\"source\":\"roadblock_bypass\"}"),
           "assistant telemetry must preserve selected reference facts");
    Expect(Contains(json, "\"reference_tracking_geometry\":{\"computed\":true"),
           "assistant telemetry must include tracking geometry object");
    Expect(Contains(json, "\"lateral_offset_m\":0.011"),
           "assistant telemetry must include tracking lateral offset");
    Expect(Contains(json, "\"heading_error_rad\":-0.022"),
           "assistant telemetry must include tracking heading error");
    Expect(Contains(json, "\"curvature_m_inv\":0.33"),
           "assistant telemetry must include tracking curvature");
    Expect(Contains(json, "\"sample_count\":5"),
           "assistant telemetry must include tracking sample count");
    Expect(Contains(json, "\"yaw_control\":{\"valid\":true,\"reason\":\"ok\",\"turn_output_target\":0.12"),
            "assistant telemetry must include yaw-control object");
    Expect(Contains(json,
                    "\"ml\":{\"enabled\":true,\"maneuver_enabled\":true,"
                    "\"takeover_selected\":true,\"artifact\":"),
           "assistant telemetry must include ML metadata");
    Expect(Contains(json, "\"candidate_id\":\"candidate-v9\""),
           "assistant telemetry must include generated artifact identity");
    Expect(Contains(json, "\"template_codes_sha256\":\"codes-sha256\""),
           "assistant telemetry must include generated artifact hash");
    Expect(Contains(json, "\"timing_us\":{\"detector\":101,\"roi\":102,\"descriptor\":103,\"replay\":104,\"classifier\":105,\"total\":410}"),
           "assistant telemetry must include stage timings");
    Expect(Contains(json, "\"detector_valid\":true"),
           "assistant telemetry must include detector validity");
    Expect(Contains(json, "\"roi\":{\"valid\":true") &&
               Contains(json, "\"width\":32,\"height\":32,\"pixel_format\":\"gray8\""),
           "assistant telemetry must include ROI metadata without bytes");
    Expect(Contains(json, "\"classification\":{\"valid\":true,\"backend\":\"tflite_int8\",\"class_id\":1,\"margin\":9") &&
               Contains(json, "\"scores\":[-5,17,8]"),
           "assistant telemetry must include backend-neutral classification facts");
    Expect(Contains(json, "\"mapped_action\":\"left\",\"locked_action\":\"left\""),
           "assistant telemetry must include ML actions");
    Expect(Contains(json, "\"phase\":\"active\",\"reason\":\"tracking\",\"confirm_count\":4"),
           "assistant telemetry must include ML phase, reason, and confirmation");
    Expect(Contains(json, "\"speed_selection\":{\"source\":\"ml_maneuver\",\"effective_speed_target\":77}"),
           "assistant telemetry must include effective ML speed selection");
    Expect(!Contains(json, "roi_bytes"),
           "assistant telemetry must not serialize ML ROI bytes");
    Expect(Contains(json, "\"lateral_term\":0.03"),
           "assistant telemetry must include lateral yaw term");
    Expect(Contains(json, "\"heading_term\":-0.01"),
           "assistant telemetry must include heading yaw term");
    Expect(Contains(json, "\"curvature_term\":0.1"),
           "assistant telemetry must include curvature yaw term");
    Expect(Contains(json, "\"raw_turn_output\":12"),
           "assistant telemetry must include top-level raw turn output");
    Expect(Contains(json, "\"applied_turn_output\":10"),
           "assistant telemetry must include top-level applied turn output");
    Expect(Contains(json, "\"left_drive_pwm_command\":120"),
           "assistant telemetry must include left drive PWM command");
    Expect(Contains(json, "\"right_drive_pwm_command\":130"),
           "assistant telemetry must include right drive PWM command");
    Expect(Contains(json, "\"left_brushless_pwm_command\":500"),
           "assistant telemetry must include left brushless PWM command");
    Expect(Contains(json, "\"right_brushless_pwm_command\":600"),
           "assistant telemetry must include right brushless PWM command");
    Expect(Contains(json, "\"left_drive_pwm_unconstrained\":145.5") &&
               Contains(json, "\"left_drive_pwm_requested\":146") &&
               Contains(json, "\"left_drive_pwm_desired\":146"),
           "assistant telemetry must expose the left PWM shaping chain");
    Expect(Contains(json, "\"right_drive_pwm_reverse_suppressed\":true"),
           "assistant telemetry must expose reverse suppression");
    Expect(Contains(json, "\"left_pid_anti_windup_active\":true") &&
               Contains(json, "\"left_pid_anti_windup_reason\":\"actuator_limit\""),
           "assistant telemetry must expose anti-windup state and reason");
    Expect(Contains(json, "\"actuator_apply_outcome\":\"drive_command_applied\""),
           "assistant telemetry must include actuator apply outcome");
    Expect(Contains(json, "\"actuators_armed\":true") &&
               Contains(json, "\"last_confirmed_left_drive_pwm\":120") &&
               Contains(json, "\"last_confirmed_right_drive_pwm\":130"),
           "assistant telemetry must include the last confirmed actuator state");
    Expect(!Contains(json, "\"actuator\":"),
           "assistant telemetry must not emit a second nested actuator standard");
    Expect(!Contains(json, "\"brushless_pwm_command\""),
           "assistant telemetry must not emit single brushless PWM key");
    Expect(!Contains(json, "\"left_pwm_command\""),
           "assistant telemetry must not emit old left PWM key");
    Expect(!Contains(json, "\"right_pwm_command\""),
           "assistant telemetry must not emit old right PWM key");
}

}  // namespace

int main() {
    try {
        TestSnapshotFactsMapToAssistantView();
        TestAssistantTelemetryJsonEmitsVisualReferenceFacts();
    } catch (const TestFailure& failure) {
        std::cerr << "assistant_telemetry_selftest failed: " << failure.message
                  << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "assistant_telemetry_selftest passed\n";
    return EXIT_SUCCESS;
}
