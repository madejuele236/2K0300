#include <array>
#include <fstream>
#include <cmath>
#include <iostream>
#include <limits>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "platform/bootstrap.hpp"
#include "control/steering_yaw_controller.hpp"

namespace {

class CaptureDiagnostics final : public ls2k::port::DiagnosticSink {
public:
    void Emit(const ls2k::port::DiagnosticEvent& event) override {
        events.push_back(event);
    }

    bool SawCode(const std::string& code) const {
        for (const ls2k::port::DiagnosticEvent& event : events) {
            if (event.code == code) {
                return true;
            }
        }
        return false;
    }

    bool SawMessageContaining(const std::string& token) const {
        for (const ls2k::port::DiagnosticEvent& event : events) {
            if (event.message.find(token) != std::string::npos) {
                return true;
            }
        }
        return false;
    }

    std::vector<ls2k::port::DiagnosticEvent> events{};
};

void Expect(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::string MinimalRuntimeParametersJson(const std::string& element_block) {
    std::string json =
        "{\n"
        "  \"RUNNING_SPEED_TARGET\": 300,\n"
        "  \"YAW_RATE_PID\": {\"P\": 12, \"I\": 0, \"D\": 0},\n"
        "  \"LEFT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, \"INTEGRAL_LIMIT\": 1000},\n"
        "  \"RIGHT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, \"INTEGRAL_LIMIT\": 1000},\n"
        "  \"assistant_tcp\": {\"host\": \"127.0.0.1\", \"port\": 8888}";
    if (!element_block.empty()) {
        json += ",\n";
        json += element_block;
    }
    json += "\n}\n";
    return json;
}

std::string RuntimeParametersJsonWithControlValues(double speed,
                                                   double left_filter_alpha,
                                                   double right_filter_alpha) {
    std::ostringstream json;
    json << "{\n"
         << "  \"RUNNING_SPEED_TARGET\": " << speed << ",\n"
         << "  \"YAW_RATE_PID\": {\"P\": 12, \"I\": 0, \"D\": 0},\n"
         << "  \"LEFT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, "
            "\"INTEGRAL_LIMIT\": 1000, \"MEASUREMENT_FILTER_ALPHA\": "
         << left_filter_alpha << "},\n"
         << "  \"RIGHT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, "
            "\"INTEGRAL_LIMIT\": 1000, \"MEASUREMENT_FILTER_ALPHA\": "
         << right_filter_alpha << "},\n"
         << "  \"assistant_tcp\": {\"host\": \"127.0.0.1\", \"port\": 8888}\n"
         << "}\n";
    return json.str();
}

std::string RuntimeParametersJsonWithIntegerValues(const std::string& control_period_ms,
                                                   const std::string& assistant_port) {
    return
        "{\n"
        "  \"RUNNING_SPEED_TARGET\": 300,\n"
        "  \"control_period_ms\": " + control_period_ms + ",\n"
        "  \"YAW_RATE_PID\": {\"P\": 12, \"I\": 0, \"D\": 0},\n"
        "  \"LEFT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, \"INTEGRAL_LIMIT\": 1000},\n"
        "  \"RIGHT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, \"INTEGRAL_LIMIT\": 1000},\n"
        "  \"assistant_tcp\": {\"host\": \"127.0.0.1\", \"port\": " + assistant_port + "}\n"
        "}\n";
}

std::string ReplaceFirst(std::string text,
                         const std::string& needle,
                         const std::string& replacement);

std::string JsonNumber(double value) {
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << std::scientific << value;
    return output.str();
}

std::string RuntimeParametersJsonWithYawPid(const std::string& p,
                                            const std::string& i,
                                            const std::string& d) {
    return ReplaceFirst(MinimalRuntimeParametersJson(""),
                        "\"P\": 12, \"I\": 0, \"D\": 0",
                        "\"P\": " + p + ", \"I\": " + i + ", \"D\": " + d);
}

float MaximumAcceptedSingleYawGain(char component) {
    const float denominator =
        component == 'P' ? ls2k::control::kMaximumYawErrorMagnitude
                         : component == 'I'
                               ? ls2k::control::kYawIntegralAccumulatorMagnitudeLimit
                               : ls2k::control::kMaximumYawDerivativeInputMagnitude;
    float candidate = std::numeric_limits<float>::max() / denominator;
    const auto accepted = [component](float value) {
        return ls2k::control::YawRatePidArithmeticIsFinite(
            component == 'P' ? value : 0.0,
            component == 'I' ? value : 0.0,
            component == 'D' ? value : 0.0);
    };
    while (!accepted(candidate)) {
        candidate = std::nextafter(candidate, 0.0F);
    }
    while (accepted(std::nextafter(candidate, std::numeric_limits<float>::infinity()))) {
        candidate = std::nextafter(candidate, std::numeric_limits<float>::infinity());
    }
    return candidate;
}

std::string ReplaceFirst(std::string text,
                         const std::string& needle,
                         const std::string& replacement) {
    const std::size_t position = text.find(needle);
    Expect(position != std::string::npos, "fixture token not found: " + needle);
    text.replace(position, needle.size(), replacement);
    return text;
}

void WriteText(const std::string& path, const std::string& text) {
    std::ofstream output(path);
    Expect(output.is_open(), "failed to open fixture for write: " + path);
    output << text;
}

std::string ReadText(const std::string& path) {
    std::ifstream input(path);
    Expect(input.is_open(), "failed to open runtime parameters: " + path);
    std::ostringstream text;
    text << input.rdbuf();
    return text.str();
}

ls2k::port::RuntimeParameters LoadFixture(const std::string& path, CaptureDiagnostics& diagnostics) {
    ls2k::port::RuntimeParameters params{};
    const std::unique_ptr<ls2k::port::IParamStore> store = ls2k::platform::MakeParamStore();
    Expect(store != nullptr, "MakeParamStore returned null");
    Expect(store->LoadRuntimeParameters(path, params, diagnostics), "LoadRuntimeParameters returned false");
    return params;
}

void ExpectRejectedFixture(const std::string& path,
                           const std::string& expected_code,
                           const std::string& expected_reason) {
    CaptureDiagnostics diagnostics{};
    ls2k::port::RuntimeParameters params{};
    params.running_speed_target = -123.0;
    params.yaw_rate_pid_d = -456.0;
    params.left_wheel_pid.p = -789.0;
    params.left_wheel_pid.integral_limit = 321.0;
    params.right_wheel_pid.d = -987.0;
    params.pwm_limit = 1234;
    params.pwm_floor = 123;
    params.assistant_tcp.host = "unchanged-sentinel";
    const std::unique_ptr<ls2k::port::IParamStore> store = ls2k::platform::MakeParamStore();
    Expect(store != nullptr, "MakeParamStore returned null");
    Expect(!store->LoadRuntimeParameters(path, params, diagnostics),
           expected_reason + " must reject runtime parameter loading: " + path);
    Expect(params.running_speed_target == -123.0,
           expected_reason + " must leave the output parameters unchanged");
    Expect(params.yaw_rate_pid_d == -456.0 && params.left_wheel_pid.p == -789.0 &&
               params.left_wheel_pid.integral_limit == 321.0 &&
               params.right_wheel_pid.d == -987.0 &&
               params.pwm_limit == 1234 &&
               params.pwm_floor == 123 &&
               params.assistant_tcp.host == "unchanged-sentinel",
           expected_reason + " must leave all representative output fields unchanged");
    Expect(diagnostics.SawCode(expected_code),
           expected_reason + " must emit " + expected_code);
    Expect(diagnostics.SawMessageContaining(expected_reason),
           expected_reason + " must be explicit in diagnostics");
    Expect(diagnostics.SawMessageContaining("refusing startup"),
           expected_reason + " must explicitly refuse startup");
    Expect(diagnostics.SawMessageContaining(path),
           expected_reason + " must include the source path in diagnostics");
    Expect(!diagnostics.SawMessageContaining("default"),
           expected_reason + " must not claim fallback to defaults");
}

void VerifyActiveRuntimeParameters(const std::string& path) {
    CaptureDiagnostics diagnostics{};
    const ls2k::port::RuntimeParameters params = LoadFixture(path, diagnostics);
    Expect(!params.loaded_from_defaults,
           "active runtime parameters must load without fallback");
    Expect(!params.parse_failure,
           "active runtime parameters must pass production validation");

    const std::string json = ReadText(path);
    const std::vector<std::string> removed_tokens = {
        "\"turn_output_to_wheel_delta_gain\"",
        "\"wheel_turn_target_scale\"",
        "\"camera_frame_width\"",
        "\"camera_frame_height\"",
        "\"LATERAL_ERROR_FAR_WEIGHT\"",
        "\"LATERAL_ERROR_TO_WHEEL_DELTA_GAIN\"",
        "\"LOOKAHEAD_VISIBLE_RANGE_RATIO\"",
        "\"LOOKAHEAD_MIN_M\"",
        "\"LOOKAHEAD_MAX_M\"",
        "\"PURE_PURSUIT_GAIN\"",
        "\"CURVATURE_COMMAND_LIMIT\"",
        "\"CURVATURE_TO_TURN_OUTPUT_GAIN\"",
        "\"CURVATURE_TO_YAW_RATE_TARGET_GAIN\"",
        "\"CROSS_WIDE_ROW_WHITE_RATIO_MIN\"",
        "\"CIRCLE_V2_ENTRY_BOTTOM_ROW_COUNT\"",
        "\"CIRCLE_V2_ENTRY_BOTTOM_MIN_ROW_COUNT\"",
        "\"CIRCLE_V2_ENTRY_BOTTOM_FORWARD_MIN_M\"",
        "\"CIRCLE_V2_ENTRY_BOTTOM_FORWARD_MAX_M\"",
        "\"CIRCLE_ENTRY_",
        "\"CIRCLE_EVIDENCE_",
        "\"CIRCLE_MIN_",
        "\"CIRCLE_OPEN",
        "\"CIRCLE_OPPOSITE_",
        "\"CIRCLE_PRESENT_",
        "\"delta_s_m\"",
        "\"BEV_ELEMENT_RASTER\"",
        "\"prohibit_reverse_pwm_step_limit\"",
        "\"motion_pwm_step_limit\"",
    };
    for (const std::string& token : removed_tokens) {
        Expect(json.find(token) == std::string::npos,
               "active runtime parameters retain removed token: " + token);
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Expect(argc == 1 || argc == 2,
               "usage: param_store_load_runtime_parameters_test [active_params.json]");
        if (argc == 2) {
            VerifyActiveRuntimeParameters(argv[1]);
        }
        const std::string base = "/tmp/param_store_load_runtime_parameters_test";
        const ls2k::port::RuntimeParameters builtin_defaults{};

        const std::string enabled_path = base + "_enabled.json";
        WriteText(enabled_path,
                  MinimalRuntimeParametersJson(
                      "  \"steering_media_downsample\": 4,\n"
                      "  \"steering_media_publish_latest_frame\": 1,\n"
                      "  \"steering_media_gray_bits\": 4,\n"
                      "  \"wheel_turn_accel_delta_scale\": 1.25,\n"
                      "  \"wheel_turn_decel_delta_scale\": 0.75,\n"
                      "  \"brushless_debug_fixed_pwm_enabled\": 0,\n"
                      "  \"brushless_debug_fixed_pwm\": 750,\n"
                      "  \"prohibit_reverse_pwm\": 1,\n"
                      "  \"drive_pwm_step_limit\": 750,\n"
                      "  \"MOTION_ODOMETRY\": {\"ENCODER_TICKS_TO_METER\": 0.001},\n"
                      "  \"ML\": {"
                      "\"ENABLED\": 1,"
                      "\"ROI\": {"
                      "\"SEARCH_FORWARD_MIN_M\": 0.1,\"SEARCH_FORWARD_MAX_M\": 1.0,"
                      "\"SEARCH_LATERAL_LIMIT_M\": 0.5,"
                      "\"GRID_FORWARD_STEP_M\": 0.02,\"GRID_LATERAL_STEP_M\": 0.03,"
                      "\"RED_Y_MIN\": 20,\"RED_Y_MAX\": 220,"
                      "\"RED_U_MIN\": 10,\"RED_U_MAX\": 120,"
                      "\"RED_V_MIN\": 130,\"RED_V_MAX\": 250,"
                      "\"EXPECTED_LONG_EDGE_M\": 0.2,\"EXPECTED_SHORT_EDGE_M\": 0.1,"
                      "\"CROP_LONG_OFFSET_M\": -0.004,\"CROP_FORWARD_OFFSET_M\": -0.005,"
                      "\"LONG_EDGE_TOLERANCE_M\": 0.05,\"SHORT_EDGE_TOLERANCE_M\": 0.03,"
                      "\"MAX_LONG_EDGE_TO_LATERAL_RAD\": 0.3,"
                      "\"MIN_COMPONENT_CELLS\": 4,\"MIN_RECTANGULARITY\": 0.5,"
                      "\"MIN_RED_FILL_RATIO\": 0.5,\"SCORE_SIZE_WEIGHT\": 2.0,"
                      "\"SCORE_RECTANGULARITY_WEIGHT\": 1.0,"
                      "\"SCORE_RED_FILL_WEIGHT\": 1.0,\"SCORE_ORIENTATION_WEIGHT\": 1.0},"
                      "\"V9\": {\"MIN_MARGIN\": 2,\"MAX_BEST_DISTANCE\": 50,\"CONFIRM_FRAMES\": 3},"
                      "\"TFLITE_IDENTITY\": {\"MIN_MARGIN\": 7,\"MAX_BEST_DISTANCE\": 2076,"
                      "\"CONFIRM_FRAMES\": 5},"
                      "\"CLASS_MAPPING\": {\"CLASS_0_ACTION\": \"straight\","
                      "\"CLASS_1_ACTION\": \"left\",\"CLASS_2_ACTION\": \"right\"},"
                      "\"MANEUVER\": {\"ENABLED\": 1,\"SPEED_TARGET\": 100,\"MIN_BOUNDARY_SAMPLES\": 3,"
                      "\"PATH_OUTWARD_OFFSET_M\": 0.12,\"EXIT_FORWARD_M\": 0.5,"
                      "\"MAX_DURATION_MS\": 1000,"
                      "\"MAX_INTEGRATION_GAP_MS\": 30,\"COOLDOWN_MS\": 200}},\n"
                      "  \"BEV_GEOMETRY\": {"
                      "\"NOMINAL_ROAD_HALF_WIDTH_M\": 0.33,"
                      "\"SPARSE_ROW_COUNT\": 12,"
                      "\"BOUNDARY_TRACE_MAX_ADJACENT_DISTANCE_M\": 0.37},\n"
                      "  \"BEV_CLASSIFICATION\": {"
                      "\"WHITE_CONFIDENCE_MIN\": 0.66,"
                      "\"UNKNOWN_CONFIDENCE_MIN\": 0.33,"
                      "\"HOLD_LAST_MAX_CYCLES\": 24},\n"
                      "  \"BEV_CONTROL_MODEL\": {"
                      "\"LATERAL_OFFSET_TO_WHEEL_DELTA_GAIN\": 321,"
                      "\"HEADING_ERROR_TO_WHEEL_DELTA_GAIN\": 45,"
                      "\"CURVATURE_TO_WHEEL_DELTA_GAIN\": 67,"
                      "\"TRACKING_FIT_MIN_SAMPLES\": 5},\n"
                      "  \"BEV_ELEMENT\": {"
                      "\"CROSS_MIN_SAMPLEABLE_PER_ROW\": 11,"
                      "\"CROSS_CONNECTIVITY_SAMPLE_INDEX\": 7,"
                      "\"CROSS_BOUNDARY_EXPANSION_MIN_M\": 0.071,"
                      "\"ZEBRA_FORWARD_MIN_M\": 0.06,"
                      "\"ZEBRA_FORWARD_MAX_M\": 0.55,"
                      "\"ZEBRA_MIN_JUMPS_PER_ROW\": 7,"
                      "\"ZEBRA_MAX_ADJACENT_FORWARD_GAP_M\": 0.13,"
                      "\"ZEBRA_MIN_SUPPORT_FORWARD_SPAN_M\": 0.045,"
                      "\"ZEBRA_REENTRY_ARM_ABSENCE_MS\": 650,"
                      "\"ZEBRA_CONTROLLED_STOP_DELAY_MS\": 750,"
                      "\"CIRCLE_V2_ENABLED\": 1,"
                      "\"CIRCLE_V2_NORMAL_TRACE_START_YAW_DEG\": 95,"
                      "\"CIRCLE_V2_EXIT_TRACE_START_YAW_DEG\": 260,"
                      "\"CIRCLE_V2_CALM_FALLBACK_YAW_DEG\": 330,"
                      "\"CIRCLE_V2_CALM_TRACE_MS\": 1200,"
                      "\"CIRCLE_V2_COOLDOWN_MS\": 2400,"
                      "\"CIRCLE_V2_INNER_TRACE_STALL_TIMEOUT_MS\": 4500,"
                      "\"CIRCLE_V2_INNER_TRACE_STALL_YAW_MIN_DEG\": 12.5,"
                      "\"CIRCLE_V2_INNER_TRACE_PATH_OFFSET_M\": 0.07,"
                      "\"CIRCLE_V2_OPPOSITE_STRAIGHT_CONFIDENCE_MIN\": 0.63,"
                      "\"CIRCLE_V2_MIN_SAMPLEABLE_WIDTH_M\": 0.4,"
                      "\"CIRCLE_V2_OPENING_FORWARD_MIN_M\": 0.06,"
                      "\"CIRCLE_V2_OPENING_FORWARD_MAX_M\": 1.4,"
                      "\"CIRCLE_V2_OPENING_DISTANCE_MIN_M\": 0.065,"
                      "\"CIRCLE_V2_OPENING_CONFIRM_FORWARD_SPAN_M\": 0.12,"
                      "\"CIRCLE_V2_ENTRY_FORWARD_MIN_M\": 0.11,"
                      "\"CIRCLE_V2_ENTRY_FORWARD_MAX_M\": 0.55,"
                      "\"CIRCLE_V2_INNER_GEOMETRY_FORWARD_MIN_M\": 0.07,"
                      "\"CIRCLE_V2_INNER_GEOMETRY_FORWARD_MAX_M\": 0.52,"
                      "\"CIRCLE_V2_EXIT_GEOMETRY_FORWARD_MIN_M\": 0.08,"
                      "\"CIRCLE_V2_EXIT_GEOMETRY_FORWARD_MAX_M\": 0.53,"
                      "\"CIRCLE_V2_EXIT_STRAIGHT_MAX_LATERAL_SPAN_M\": 0.14,"
                      "\"CIRCLE_V2_EXIT_TANGENT_FIT_SPAN_M\": 0.18}"));
        CaptureDiagnostics enabled_diagnostics{};
        const ls2k::port::RuntimeParameters enabled =
            LoadFixture(enabled_path, enabled_diagnostics);
        Expect(!enabled.loaded_from_defaults, "enabled fixture should not fall back to defaults");
        Expect(!enabled.parse_failure, "enabled fixture should parse cleanly");
        Expect(enabled.steering_media_downsample == 4,
               "steering_media_downsample should parse");
        Expect(enabled.steering_media_publish_latest_frame,
               "steering_media_publish_latest_frame should parse true");
        Expect(enabled.steering_media_gray_bits == 4,
               "steering_media_gray_bits should parse");
        Expect(std::abs(enabled.wheel_turn_accel_delta_scale - 1.25) < 1.0e-6,
               "wheel_turn_accel_delta_scale should parse");
        Expect(std::abs(enabled.wheel_turn_decel_delta_scale - 0.75) < 1.0e-6,
               "wheel_turn_decel_delta_scale should parse");
        Expect(!enabled.brushless_debug_fixed_pwm_enabled,
               "brushless_debug_fixed_pwm_enabled should parse false");
        Expect(enabled.brushless_debug_fixed_pwm == 750,
               "brushless_debug_fixed_pwm should parse");
        Expect(enabled.prohibit_reverse_pwm,
               "prohibit_reverse_pwm=1 should parse true");
        Expect(enabled.drive_pwm_step_limit == 750,
               "drive_pwm_step_limit should parse");
        Expect(enabled.ml.enabled && enabled.ml.maneuver.enabled &&
                   std::abs(enabled.motion_odometry.encoder_ticks_to_meter - 0.001) < 1.0e-9,
               "enabled ML maneuver must parse the shared motion-odometry scale");
        Expect(enabled.ml.v9.min_margin == 2 &&
                   enabled.ml.v9.max_best_distance == 50 &&
                   enabled.ml.v9.confirm_frames == 3,
               "ML V9 acceptance and confirmation parameters should parse");
        Expect(enabled.ml.tflite_identity.min_margin == 7 &&
                   enabled.ml.tflite_identity.max_best_distance == 2076 &&
                   enabled.ml.tflite_identity.confirm_frames == 5,
               "ML TFLITE_IDENTITY acceptance and confirmation parameters should parse");
        Expect(enabled.ml.class_mapping.class_1_action == "left" &&
                   enabled.ml.class_mapping.class_2_action == "right",
               "ML class mapping should parse independently of replay");
        Expect(std::abs(enabled.ml.roi.grid_forward_step_m - 0.02) < 1.0e-9 &&
                   std::abs(enabled.ml.roi.grid_lateral_step_m - 0.03) < 1.0e-9 &&
                   std::abs(enabled.ml.roi.expected_long_edge_m - 0.2) < 1.0e-9 &&
                   std::abs(enabled.ml.roi.crop_long_offset_m + 0.004) < 1.0e-9 &&
                   std::abs(enabled.ml.roi.crop_forward_offset_m + 0.005) < 1.0e-9 &&
                   std::abs(enabled.ml.maneuver.speed_target - 100.0) < 1.0e-9 &&
                   std::abs(enabled.ml.maneuver.path_outward_offset_m - 0.12) < 1.0e-9,
               "ML ROI and maneuver parameters should parse");
        Expect(std::abs(enabled.bev_geometry.nominal_road_half_width_m - 0.33F) <
                   1.0e-6F,
               "BEV_GEOMETRY.NOMINAL_ROAD_HALF_WIDTH_M should parse");
        Expect(enabled.bev_geometry.sparse_row_count == 12,
               "BEV_GEOMETRY.SPARSE_ROW_COUNT should parse");
        Expect(std::abs(enabled.bev_geometry.boundary_trace_max_adjacent_distance_m -
                        0.37F) < 1.0e-6F,
               "BEV_GEOMETRY.BOUNDARY_TRACE_MAX_ADJACENT_DISTANCE_M should parse");
        Expect(std::abs(enabled.bev_classification.white_confidence_min -
                        0.66F) < 1.0e-6F,
               "BEV_CLASSIFICATION.WHITE_CONFIDENCE_MIN should parse");
        Expect(std::abs(enabled.bev_classification.unknown_confidence_min -
                        0.33F) < 1.0e-6F,
               "BEV_CLASSIFICATION.UNKNOWN_CONFIDENCE_MIN should parse");
        Expect(enabled.bev_classification.hold_last_max_cycles == 24,
               "BEV_CLASSIFICATION.HOLD_LAST_MAX_CYCLES should parse");
        Expect(std::abs(enabled.bev_control_model.lateral_offset_to_wheel_delta_gain -
                        321.0) < 1.0e-6,
               "BEV_CONTROL_MODEL.LATERAL_OFFSET_TO_WHEEL_DELTA_GAIN should parse");
        Expect(std::abs(enabled.bev_control_model.heading_error_to_wheel_delta_gain -
                        45.0) < 1.0e-6,
               "BEV_CONTROL_MODEL.HEADING_ERROR_TO_WHEEL_DELTA_GAIN should parse");
        Expect(std::abs(enabled.bev_control_model.curvature_to_wheel_delta_gain -
                        67.0) < 1.0e-6,
               "BEV_CONTROL_MODEL.CURVATURE_TO_WHEEL_DELTA_GAIN should parse");
        Expect(enabled.bev_control_model.tracking_fit_min_samples == 5,
               "BEV_CONTROL_MODEL.TRACKING_FIT_MIN_SAMPLES should parse");
        Expect(enabled.bev_element.cross_min_sampleable_per_row == 11,
               "CROSS_MIN_SAMPLEABLE_PER_ROW should parse");
        Expect(enabled.bev_element.cross_connectivity_sample_index == 7,
               "CROSS_CONNECTIVITY_SAMPLE_INDEX should parse");
        Expect(std::abs(enabled.bev_element.cross_boundary_expansion_min_m - 0.071F) <
                   1.0e-6F,
               "CROSS_BOUNDARY_EXPANSION_MIN_M should parse");
        Expect(std::abs(enabled.bev_element.zebra_forward_min_m - 0.06F) < 1.0e-6F &&
                   std::abs(enabled.bev_element.zebra_forward_max_m - 0.55F) < 1.0e-6F,
               "Zebra forward ROI should parse");
        Expect(enabled.bev_element.zebra_min_jumps_per_row == 7,
               "ZEBRA_MIN_JUMPS_PER_ROW should parse");
        Expect(std::abs(enabled.bev_element.zebra_max_adjacent_forward_gap_m - 0.13F) <
                       1.0e-6F &&
                   std::abs(enabled.bev_element.zebra_min_support_forward_span_m - 0.045F) <
                       1.0e-6F,
               "Zebra metric support thresholds should parse");
        Expect(enabled.bev_element.zebra_reentry_arm_absence_ms == 650 &&
                   enabled.bev_element.zebra_controlled_stop_delay_ms == 750,
               "Zebra stop timing parameters should parse");
        Expect(enabled.bev_element.circle_v2_enabled,
               "CIRCLE_V2_ENABLED=1 should parse true");
        Expect(std::abs(enabled.bev_element.circle_v2_normal_trace_start_yaw_deg - 95.0F) <
                   1.0e-6F,
               "CIRCLE_V2_NORMAL_TRACE_START_YAW_DEG should parse");
        Expect(std::abs(enabled.bev_element.circle_v2_exit_trace_start_yaw_deg - 260.0F) <
                   1.0e-6F,
               "CIRCLE_V2_EXIT_TRACE_START_YAW_DEG should parse");
        Expect(std::abs(enabled.bev_element.circle_v2_calm_fallback_yaw_deg - 330.0F) <
                   1.0e-6F,
               "CIRCLE_V2_CALM_FALLBACK_YAW_DEG should parse");
        Expect(enabled.bev_element.circle_v2_calm_trace_ms == 1200,
               "CIRCLE_V2_CALM_TRACE_MS should parse");
        Expect(enabled.bev_element.circle_v2_cooldown_ms == 2400,
               "CIRCLE_V2_COOLDOWN_MS should parse");
        Expect(enabled.bev_element.circle_v2_inner_trace_stall_timeout_ms == 4500,
               "CIRCLE_V2_INNER_TRACE_STALL_TIMEOUT_MS should parse");
        Expect(std::abs(enabled.bev_element.circle_v2_inner_trace_stall_yaw_min_deg -
                        12.5F) < 1.0e-6F,
               "CIRCLE_V2_INNER_TRACE_STALL_YAW_MIN_DEG should parse");
        Expect(std::abs(enabled.bev_element.circle_v2_inner_trace_path_offset_m -
                        0.07F) < 1.0e-6F,
               "CIRCLE_V2_INNER_TRACE_PATH_OFFSET_M should parse");
        Expect(std::abs(enabled.bev_element.circle_v2_opposite_straight_confidence_min -
                        0.63F) < 1.0e-6F,
               "CIRCLE_V2_OPPOSITE_STRAIGHT_CONFIDENCE_MIN should parse");
        Expect(std::abs(enabled.bev_element.circle_v2_min_sampleable_width_m - 0.4F) <
                   1.0e-6F,
               "CIRCLE_V2_MIN_SAMPLEABLE_WIDTH_M should parse");
        Expect(std::abs(enabled.bev_element.circle_v2_opening_forward_min_m - 0.06F) <
                   1.0e-6F &&
                   std::abs(enabled.bev_element.circle_v2_opening_forward_max_m - 1.4F) <
                       1.0e-6F,
               "CircleV2 opening ROI should parse");
        Expect(std::abs(enabled.bev_element.circle_v2_opening_distance_min_m - 0.065F) <
                   1.0e-6F &&
                   std::abs(enabled.bev_element.circle_v2_opening_confirm_forward_span_m -
                            0.12F) < 1.0e-6F,
               "CircleV2 opening metric thresholds should parse");
        Expect(std::abs(enabled.bev_element.circle_v2_entry_forward_min_m - 0.11F) <
                   1.0e-6F &&
                   std::abs(enabled.bev_element.circle_v2_entry_forward_max_m - 0.55F) <
                       1.0e-6F,
               "CircleV2 entry ROI should parse");
        Expect(std::abs(enabled.bev_element.circle_v2_inner_geometry_forward_min_m -
                        0.07F) < 1.0e-6F &&
                   std::abs(enabled.bev_element.circle_v2_inner_geometry_forward_max_m -
                            0.52F) < 1.0e-6F,
               "CircleV2 inner geometry ROI should parse");
        Expect(std::abs(enabled.bev_element.circle_v2_exit_geometry_forward_min_m -
                        0.08F) < 1.0e-6F &&
                   std::abs(enabled.bev_element.circle_v2_exit_geometry_forward_max_m -
                            0.53F) < 1.0e-6F,
               "CircleV2 exit geometry ROI should parse");
        Expect(std::abs(enabled.bev_element.circle_v2_exit_straight_max_lateral_span_m -
                        0.14F) < 1.0e-6F,
               "CircleV2 exit straight span should parse");
        const std::string absent_path = base + "_absent.json";
        WriteText(absent_path, MinimalRuntimeParametersJson(""));
        CaptureDiagnostics absent_diagnostics{};
        const ls2k::port::RuntimeParameters absent =
            LoadFixture(absent_path, absent_diagnostics);
        Expect(!absent.loaded_from_defaults, "absent BEV_ELEMENT should not fall back to defaults");
        Expect(!absent.parse_failure, "absent BEV_ELEMENT should parse cleanly");
        Expect(absent.steering_media_downsample == 1,
               "missing steering_media_downsample should keep downsample default");
        Expect(!absent.steering_media_publish_latest_frame,
               "missing steering_media_publish_latest_frame should keep strict snapshot alignment");
        Expect(absent.steering_media_gray_bits == 2,
               "missing steering_media_gray_bits should keep gray2 default");
        Expect(std::abs(absent.wheel_turn_accel_delta_scale -
                        builtin_defaults.wheel_turn_accel_delta_scale) < 1.0e-6,
               "missing wheel_turn_accel_delta_scale should keep default");
        Expect(std::abs(absent.wheel_turn_decel_delta_scale - 1.0) < 1.0e-6,
               "missing wheel_turn_decel_delta_scale should keep default");
        Expect(absent.brushless_debug_fixed_pwm_enabled ==
                   builtin_defaults.brushless_debug_fixed_pwm_enabled,
               "missing brushless_debug_fixed_pwm_enabled should keep default");
        Expect(absent.brushless_debug_fixed_pwm ==
                   builtin_defaults.brushless_debug_fixed_pwm,
               "missing brushless_debug_fixed_pwm should keep default");
        Expect(absent.prohibit_reverse_pwm && builtin_defaults.prohibit_reverse_pwm,
               "missing prohibit_reverse_pwm should keep safe enabled default");
        Expect(absent.drive_pwm_step_limit == 1000 &&
                   builtin_defaults.drive_pwm_step_limit == 1000,
               "missing drive_pwm_step_limit should keep safe global default");
        Expect(absent.bev_element.cross_min_sampleable_per_row ==
                   builtin_defaults.bev_element.cross_min_sampleable_per_row,
               "missing BEV_ELEMENT should keep cross sampleable threshold default");
        Expect(absent.bev_element.cross_connectivity_sample_index == 9 &&
                   builtin_defaults.bev_element.cross_connectivity_sample_index == 9,
               "missing BEV_ELEMENT should keep cross connectivity sample index default");
        Expect(std::abs(absent.bev_element.cross_boundary_expansion_min_m - 0.055F) <
                   1.0e-6F &&
                   std::abs(builtin_defaults.bev_element.cross_boundary_expansion_min_m -
                            0.055F) < 1.0e-6F,
               "missing BEV_ELEMENT should keep cross expansion distance default");
        Expect(std::abs(absent.bev_element.zebra_forward_min_m - 0.05F) < 1.0e-6F &&
                   std::abs(absent.bev_element.zebra_forward_max_m - 0.50F) < 1.0e-6F &&
                   absent.bev_element.zebra_min_jumps_per_row == 6 &&
                   std::abs(absent.bev_element.zebra_max_adjacent_forward_gap_m - 0.12F) <
                       1.0e-6F &&
                   std::abs(absent.bev_element.zebra_min_support_forward_span_m - 0.04F) <
                       1.0e-6F &&
                   absent.bev_element.zebra_reentry_arm_absence_ms == 500 &&
                   absent.bev_element.zebra_controlled_stop_delay_ms == 500,
               "missing BEV_ELEMENT should keep Zebra defaults");
        Expect(absent.bev_element.circle_v2_enabled ==
                   builtin_defaults.bev_element.circle_v2_enabled,
               "missing BEV_ELEMENT should keep CircleV2 enabled");
        Expect(std::abs(absent.bev_element.circle_v2_normal_trace_start_yaw_deg -
                        builtin_defaults.bev_element.circle_v2_normal_trace_start_yaw_deg) < 1.0e-6F &&
                   std::abs(absent.bev_element.circle_v2_exit_trace_start_yaw_deg -
                            builtin_defaults.bev_element.circle_v2_exit_trace_start_yaw_deg) < 1.0e-6F &&
                   std::abs(absent.bev_element.circle_v2_calm_fallback_yaw_deg -
                            builtin_defaults.bev_element.circle_v2_calm_fallback_yaw_deg) < 1.0e-6F,
               "missing BEV_ELEMENT should keep CircleV2 angle thresholds");
        Expect(absent.bev_element.circle_v2_calm_trace_ms ==
                   builtin_defaults.bev_element.circle_v2_calm_trace_ms &&
                   absent.bev_element.circle_v2_cooldown_ms ==
                       builtin_defaults.bev_element.circle_v2_cooldown_ms,
               "missing BEV_ELEMENT should keep CircleV2 timed phases");
        Expect(absent.bev_element.circle_v2_inner_trace_stall_timeout_ms ==
                   builtin_defaults.bev_element.circle_v2_inner_trace_stall_timeout_ms,
               "missing BEV_ELEMENT should keep CircleV2 stall timeout default");
        Expect(std::abs(absent.bev_element.circle_v2_inner_trace_stall_yaw_min_deg -
                        builtin_defaults.bev_element.circle_v2_inner_trace_stall_yaw_min_deg) < 1.0e-6F,
               "missing BEV_ELEMENT should keep CircleV2 stall yaw default");
        Expect(std::abs(absent.bev_element.circle_v2_inner_trace_path_offset_m -
                        builtin_defaults.bev_element.circle_v2_inner_trace_path_offset_m) < 1.0e-6F,
               "missing BEV_ELEMENT should keep CircleV2 inner path offset default");
        Expect(std::abs(absent.bev_element.circle_v2_opposite_straight_confidence_min -
                        builtin_defaults.bev_element.circle_v2_opposite_straight_confidence_min) < 1.0e-6F,
               "missing BEV_ELEMENT should keep CircleV2 opposite-straight confidence default");
        Expect(std::abs(absent.bev_element.circle_v2_min_sampleable_width_m -
                        builtin_defaults.bev_element.circle_v2_min_sampleable_width_m) < 1.0e-6F &&
                   std::abs(absent.bev_element.circle_v2_opening_forward_min_m -
                            builtin_defaults.bev_element.circle_v2_opening_forward_min_m) < 1.0e-6F &&
                   std::abs(absent.bev_element.circle_v2_opening_forward_max_m -
                            builtin_defaults.bev_element.circle_v2_opening_forward_max_m) < 1.0e-6F &&
                   std::abs(absent.bev_element.circle_v2_opening_distance_min_m -
                            builtin_defaults.bev_element.circle_v2_opening_distance_min_m) < 1.0e-6F &&
                   std::abs(absent.bev_element.circle_v2_opening_confirm_forward_span_m -
                            builtin_defaults.bev_element.circle_v2_opening_confirm_forward_span_m) < 1.0e-6F,
               "missing BEV_ELEMENT should keep CircleV2 opening defaults");
        Expect(std::abs(absent.bev_element.circle_v2_entry_forward_min_m -
                        builtin_defaults.bev_element.circle_v2_entry_forward_min_m) < 1.0e-6F &&
                   std::abs(absent.bev_element.circle_v2_entry_forward_max_m -
                            builtin_defaults.bev_element.circle_v2_entry_forward_max_m) < 1.0e-6F,
               "missing BEV_ELEMENT should keep CircleV2 entry ROI defaults");
        Expect(std::abs(absent.bev_element.circle_v2_inner_geometry_forward_min_m -
                        builtin_defaults.bev_element.circle_v2_inner_geometry_forward_min_m) < 1.0e-6F &&
                   std::abs(absent.bev_element.circle_v2_inner_geometry_forward_max_m -
                            builtin_defaults.bev_element.circle_v2_inner_geometry_forward_max_m) < 1.0e-6F &&
                   std::abs(absent.bev_element.circle_v2_exit_geometry_forward_min_m -
                            builtin_defaults.bev_element.circle_v2_exit_geometry_forward_min_m) < 1.0e-6F &&
                   std::abs(absent.bev_element.circle_v2_exit_geometry_forward_max_m -
                            builtin_defaults.bev_element.circle_v2_exit_geometry_forward_max_m) < 1.0e-6F &&
                   std::abs(absent.bev_element.circle_v2_exit_straight_max_lateral_span_m -
                            builtin_defaults.bev_element.circle_v2_exit_straight_max_lateral_span_m) < 1.0e-6F,
               "missing BEV_ELEMENT should keep CircleV2 geometry defaults");
        Expect(std::abs(absent.bev_geometry.nominal_road_half_width_m - 0.225F) <
                   1.0e-6F,
               "missing BEV_GEOMETRY should keep nominal road half-width default");
        Expect(absent.bev_geometry.sparse_row_count ==
                   static_cast<int>(ls2k::port::kBevReferenceSampleCount),
               "missing BEV_GEOMETRY should keep sparse row count default");
        Expect(std::abs(absent.bev_geometry.boundary_trace_max_adjacent_distance_m -
                        0.195660427F) < 1.0e-6F,
               "missing BEV_GEOMETRY should keep boundary trace distance default");
        Expect(std::abs(absent.bev_classification.white_confidence_min -
                        0.55F) < 1.0e-6F,
               "missing BEV_CLASSIFICATION should keep white confidence default");
        Expect(std::abs(absent.bev_classification.unknown_confidence_min -
                        0.25F) < 1.0e-6F,
               "missing BEV_CLASSIFICATION should keep unknown confidence default");
        Expect(absent.bev_classification.hold_last_max_cycles == 32,
               "missing BEV_CLASSIFICATION should keep hold default");
        Expect(std::abs(absent.bev_control_model.lateral_offset_to_wheel_delta_gain -
                        builtin_defaults.bev_control_model.lateral_offset_to_wheel_delta_gain) < 1.0e-6,
               "missing BEV_CONTROL_MODEL should keep lateral offset gain default");
        Expect(absent.bev_control_model.tracking_fit_min_samples == 3,
               "missing BEV_CONTROL_MODEL should keep tracking fit sample default");
        Expect(absent.ml.tflite_identity.min_margin == 1 &&
                   absent.ml.tflite_identity.max_best_distance == 2076 &&
                   absent.ml.tflite_identity.confirm_frames == 3,
               "missing ML.TFLITE_IDENTITY should keep safe artifact-calibrated startup defaults");

        const std::string missing_file_path = base + "_does_not_exist/runtime.json";
        ExpectRejectedFixture(missing_file_path,
                              "params.missing",
                              "runtime parameter file missing");

        const std::string invalid_json_path = base + "_invalid_json.json";
        WriteText(invalid_json_path, "{");
        ExpectRejectedFixture(invalid_json_path,
                              "params.parse",
                              "invalid JSON syntax or non-object root");

        const std::string non_object_root_path = base + "_non_object_root.json";
        WriteText(non_object_root_path, "[]");
        ExpectRejectedFixture(non_object_root_path,
                              "params.parse",
                              "invalid JSON syntax or non-object root");

        const std::vector<std::pair<std::string, std::string>> invalid_json_documents = {
            {"trailing_garbage", MinimalRuntimeParametersJson("") + " garbage"},
            {"trailing_object", MinimalRuntimeParametersJson("") + " {}"},
            {"unclosed_block_comment", MinimalRuntimeParametersJson("") + " /* unclosed"},
            {"unclosed_string", "{\"key\": \"unterminated}"},
            {"leading_zero_number", "{\"value\": 01}"},
            {"missing_fraction_digits", "{\"value\": 1.}"},
            {"leading_plus_number", "{\"value\": +1}"},
            {"missing_exponent_digits", "{\"value\": 1e+}"},
            {"comment_token_join", "{\"value\": 1/* separator */2}"},
        };
        for (const auto& invalid_document : invalid_json_documents) {
            const std::string path = base + "_" + invalid_document.first + ".json";
            WriteText(path, invalid_document.second);
            ExpectRejectedFixture(path,
                                  "params.parse",
                                  "invalid JSON syntax or non-object root");
        }

        const std::string commented_json_path = base + "_comments_supported.json";
        std::string commented_json =
            ReplaceFirst(MinimalRuntimeParametersJson(""),
                         "{\n",
                         "{/* block comment */\n// line comment\n");
        commented_json = ReplaceFirst(commented_json,
                                      "\"RUNNING_SPEED_TARGET\": 300",
                                      "\"RUNNING_SPEED_TARGET\": /* value */ 300");
        WriteText(commented_json_path, commented_json);
        CaptureDiagnostics commented_json_diagnostics{};
        const ls2k::port::RuntimeParameters commented_params =
            LoadFixture(commented_json_path, commented_json_diagnostics);
        Expect(commented_params.running_speed_target == 300.0,
               "closed JSON comments used as whitespace must remain supported");

        const std::vector<std::pair<std::string, std::string>> valid_outer_whitespace = {
            {"space_tab_crlf", " \t\r\n" + MinimalRuntimeParametersJson("") + "\r\n\t "},
            {"line_comment_before", "// before root\n" + MinimalRuntimeParametersJson("")},
            {"line_comment_after", MinimalRuntimeParametersJson("") + "// after root"},
            {"block_comment_before", "/* before root */\r\n" + MinimalRuntimeParametersJson("")},
            {"block_comment_after", MinimalRuntimeParametersJson("") + "\n/* after root */"},
        };
        for (const auto& valid_document : valid_outer_whitespace) {
            const std::string path = base + "_outer_" + valid_document.first + ".json";
            WriteText(path, valid_document.second);
            CaptureDiagnostics diagnostics{};
            const ls2k::port::RuntimeParameters params = LoadFixture(path, diagnostics);
            Expect(params.running_speed_target == 300.0,
                   "outer JSON whitespace and closed comments must remain valid");
        }

        const std::string escaped_root_required_path = base + "_escaped_root_required.json";
        WriteText(escaped_root_required_path,
                  ReplaceFirst(MinimalRuntimeParametersJson(""),
                               "\"RUNNING_SPEED_TARGET\"",
                               "\"RUNNING_SPEED_T\\u0041RGET\""));
        CaptureDiagnostics escaped_root_required_diagnostics{};
        const ls2k::port::RuntimeParameters escaped_root_required =
            LoadFixture(escaped_root_required_path, escaped_root_required_diagnostics);
        Expect(escaped_root_required.running_speed_target == 300.0,
               "escaped root required member name must use decoded lookup semantics");

        const std::string escaped_nested_required_path = base + "_escaped_nested_required.json";
        WriteText(escaped_nested_required_path,
                  ReplaceFirst(MinimalRuntimeParametersJson(""),
                               "\"port\": 8888",
                               "\"po\\u0072t\": 8888"));
        CaptureDiagnostics escaped_nested_required_diagnostics{};
        const ls2k::port::RuntimeParameters escaped_nested_required =
            LoadFixture(escaped_nested_required_path, escaped_nested_required_diagnostics);
        Expect(escaped_nested_required.assistant_tcp.port == 8888,
               "escaped nested required member name must use decoded lookup semantics");

        const std::string escaped_optional_path = base + "_escaped_optional.json";
        WriteText(escaped_optional_path,
                  MinimalRuntimeParametersJson("  \"control_period_\\u006ds\": 17"));
        CaptureDiagnostics escaped_optional_diagnostics{};
        const ls2k::port::RuntimeParameters escaped_optional =
            LoadFixture(escaped_optional_path, escaped_optional_diagnostics);
        Expect(escaped_optional.control_period_ms == 17,
               "escaped optional member name must use decoded lookup semantics");

        const std::string escaped_unknown_path = base + "_escaped_unknown.json";
        WriteText(escaped_unknown_path,
                  MinimalRuntimeParametersJson("  \"unused\\/key\": 1"));
        CaptureDiagnostics escaped_unknown_diagnostics{};
        LoadFixture(escaped_unknown_path, escaped_unknown_diagnostics);

        const std::string escaped_surrogate_singleton_path =
            base + "_escaped_surrogate_singleton.json";
        WriteText(escaped_surrogate_singleton_path,
                  MinimalRuntimeParametersJson("  \"\\uD83D\\uDE80\": 1"));
        CaptureDiagnostics escaped_surrogate_singleton_diagnostics{};
        LoadFixture(escaped_surrogate_singleton_path,
                    escaped_surrogate_singleton_diagnostics);

        const std::vector<std::pair<std::string, std::string>> duplicate_member_documents = {
            {"root_good_then_bad",
             ReplaceFirst(MinimalRuntimeParametersJson(""),
                          "\"RUNNING_SPEED_TARGET\": 300",
                          "\"RUNNING_SPEED_TARGET\": 300, \"RUNNING_SPEED_TARGET\": -1")},
            {"root_bad_then_good",
             ReplaceFirst(MinimalRuntimeParametersJson(""),
                          "\"RUNNING_SPEED_TARGET\": 300",
                          "\"RUNNING_SPEED_TARGET\": -1, \"RUNNING_SPEED_TARGET\": 300")},
            {"root_equivalent_escape",
             ReplaceFirst(MinimalRuntimeParametersJson(""),
                          "\"RUNNING_SPEED_TARGET\": 300",
                          "\"RUNNING_SPEED_TARGET\": 300, "
                          "\"\\u0052UNNING_SPEED_TARGET\": 301")},
            {"nested_required",
             ReplaceFirst(MinimalRuntimeParametersJson(""),
                          "\"port\": 8888",
                          "\"port\": 8888, \"port\": 9999")},
            {"nested_optional",
             ReplaceFirst(MinimalRuntimeParametersJson(""),
                          "\"P\": 0, \"I\": 0, \"D\": 0, \"INTEGRAL_LIMIT\": 1000",
                          "\"P\": 0, \"I\": 0, \"D\": 0, \"INTEGRAL_LIMIT\": 1000, "
                          "\"MEASUREMENT_FILTER_ALPHA\": 0.4, "
                          "\"MEASUREMENT_FILTER_ALPHA\": 0.5")},
            {"ordinary_escape",
             MinimalRuntimeParametersJson(
                 "  \"unused/key\": 1, \"unused\\/key\": 2")},
            {"surrogate_escape",
             MinimalRuntimeParametersJson(
                 "  \"🚀\": 1, \"\\uD83D\\uDE80\": 2")},
        };
        for (const auto& duplicate_document : duplicate_member_documents) {
            const std::string path = base + "_duplicate_" + duplicate_document.first + ".json";
            WriteText(path, duplicate_document.second);
            ExpectRejectedFixture(path,
                                  "params.parse",
                                  "invalid JSON syntax or non-object root");
        }

        const std::string same_name_different_objects_path =
            base + "_same_name_different_objects.json";
        WriteText(same_name_different_objects_path, MinimalRuntimeParametersJson(""));
        CaptureDiagnostics same_name_different_objects_diagnostics{};
        const ls2k::port::RuntimeParameters same_name_different_objects =
            LoadFixture(same_name_different_objects_path,
                        same_name_different_objects_diagnostics);
        Expect(same_name_different_objects.left_wheel_pid.p == 0.0 &&
                   same_name_different_objects.right_wheel_pid.p == 0.0,
               "same member name in distinct objects must remain valid");

        const std::string missing_required_path = base + "_missing_required.json";
        WriteText(missing_required_path,
                  "{\n"
                  "  \"YAW_RATE_PID\": {\"P\": 12, \"I\": 0, \"D\": 0},\n"
                  "  \"LEFT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, "
                  "\"INTEGRAL_LIMIT\": 1000},\n"
                  "  \"RIGHT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, "
                  "\"INTEGRAL_LIMIT\": 1000},\n"
                  "  \"assistant_tcp\": {\"host\": \"127.0.0.1\", \"port\": 8888}\n"
                  "}\n");
        ExpectRejectedFixture(missing_required_path,
                              "params.validation",
                              "missing, malformed, or out-of-range required field");

        const std::string wrong_required_type_path = base + "_wrong_required_type.json";
        WriteText(wrong_required_type_path,
                  "{\n"
                  "  \"RUNNING_SPEED_TARGET\": \"fast\",\n"
                  "  \"YAW_RATE_PID\": {\"P\": 12, \"I\": 0, \"D\": 0},\n"
                  "  \"LEFT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, "
                  "\"INTEGRAL_LIMIT\": 1000},\n"
                  "  \"RIGHT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, "
                  "\"INTEGRAL_LIMIT\": 1000},\n"
                  "  \"assistant_tcp\": {\"host\": \"127.0.0.1\", \"port\": 8888}\n"
                  "}\n");
        ExpectRejectedFixture(wrong_required_type_path,
                              "params.validation",
                              "missing, malformed, or out-of-range required field");

        const std::string int_min = std::to_string(std::numeric_limits<int>::min());
        const std::string int_max = std::to_string(std::numeric_limits<int>::max());
        const std::string below_int_min =
            std::to_string(static_cast<long long>(std::numeric_limits<int>::min()) - 1LL);
        const std::string above_int_max =
            std::to_string(static_cast<long long>(std::numeric_limits<int>::max()) + 1LL);
        const std::vector<std::string> rejected_integer_values = {
            "1e999", "1e20", below_int_min, above_int_max, "1.0000001",
        };
        for (std::size_t i = 0; i < rejected_integer_values.size(); ++i) {
            const std::string optional_path =
                base + "_optional_int_rejected_" + std::to_string(i) + ".json";
            WriteText(optional_path,
                      RuntimeParametersJsonWithIntegerValues(rejected_integer_values[i], "8888"));
            ExpectRejectedFixture(optional_path,
                                  "params.validation",
                                  "malformed or out-of-range optional field");

            const std::string required_path =
                base + "_required_int_rejected_" + std::to_string(i) + ".json";
            WriteText(required_path,
                      RuntimeParametersJsonWithIntegerValues("10", rejected_integer_values[i]));
            ExpectRejectedFixture(required_path,
                                  "params.validation",
                                  "missing, malformed, or out-of-range required field");
        }

        const auto verify_optional_semantic_case =
            [&base](const std::string& name, const std::string& block, bool valid) {
                const std::string path = base + "_semantic_" + name + ".json";
                WriteText(path, MinimalRuntimeParametersJson("  " + block));
                if (valid) {
                    CaptureDiagnostics diagnostics{};
                    (void)LoadFixture(path, diagnostics);
                } else {
                    ExpectRejectedFixture(path,
                                          "params.validation",
                                          "malformed or out-of-range optional field");
                }
            };

        struct OptionalSemanticCase {
            const char* name;
            std::string block;
            bool valid;
        };
        const std::string maximum_interval =
            std::to_string(ls2k::port::kMaximumRuntimeIntervalMs);
        const std::string above_maximum_interval =
            std::to_string(ls2k::port::kMaximumRuntimeIntervalMs + 1);
        const std::string maximum_control_period =
            std::to_string(ls2k::port::kMaximumControlPeriodMs);
        const std::string above_maximum_control_period =
            std::to_string(ls2k::port::kMaximumControlPeriodMs + 1);
        const std::string maximum_confirm_cycles = std::to_string(
            ls2k::port::kMaximumRuntimeIntervalMs / builtin_defaults.control_period_ms);
        const std::string above_maximum_confirm_cycles = std::to_string(
            ls2k::port::kMaximumRuntimeIntervalMs / builtin_defaults.control_period_ms + 1);
        const std::vector<OptionalSemanticCase> optional_semantic_cases = {
            {"control_period_negative_finding", "\"control_period_ms\": -7", false},
            {"control_period_int_min", "\"control_period_ms\": " + int_min, false},
            {"control_period_zero", "\"control_period_ms\": 0", false},
            {"control_period_one", "\"control_period_ms\": 1", true},
            {"control_period_max", "\"control_period_ms\": " + maximum_control_period, true},
            {"control_period_above_max", "\"control_period_ms\": " + above_maximum_control_period, false},
            {"control_period_int_max_finding", "\"control_period_ms\": " + int_max, false},
            {"perception_stale_zero", "\"perception_stale_ms\": 0", false},
            {"perception_stale_one", "\"perception_stale_ms\": 1", true},
            {"perception_stale_max", "\"perception_stale_ms\": " + maximum_interval, true},
            {"perception_stale_above_max", "\"perception_stale_ms\": " + above_maximum_interval, false},
            {"low_voltage_threshold_zero", "\"low_voltage_raw_threshold\": 0", false},
            {"low_voltage_threshold_one", "\"low_voltage_raw_threshold\": 1", true},
            {"low_voltage_threshold_conservative_int_max", "\"low_voltage_raw_threshold\": " + int_max, true},
            {"raw_turn_limit_negative", "\"raw_turn_output_limit\": -1", false},
            {"raw_turn_limit_zero", "\"raw_turn_output_limit\": 0", true},
            {"raw_turn_limit_int_max_hard_capped", "\"raw_turn_output_limit\": " + int_max, true},
            {"motion_confirm_zero", "\"motion_unveto_confirm_cycles\": 0", false},
            {"motion_confirm_one", "\"motion_unveto_confirm_cycles\": 1", true},
            {"motion_confirm_max_duration", "\"motion_unveto_confirm_cycles\": " + maximum_confirm_cycles, true},
            {"motion_confirm_above_max_duration", "\"motion_unveto_confirm_cycles\": " + above_maximum_confirm_cycles, false},
            {"motion_confirm_control_period_combination_max", "\"control_period_ms\": 1000, \"motion_unveto_confirm_cycles\": 86400", true},
            {"motion_confirm_control_period_combination_above_max", "\"control_period_ms\": 1000, \"motion_unveto_confirm_cycles\": 86401", false},
            {"motion_spinup_negative", "\"motion_spinup_ms\": -1", false},
            {"motion_spinup_zero_immediate", "\"motion_spinup_ms\": 0", true},
            {"motion_spinup_max", "\"motion_spinup_ms\": " + maximum_interval, true},
            {"motion_spinup_above_max", "\"motion_spinup_ms\": " + above_maximum_interval, false},
            {"motion_turn_scale_negative", "\"motion_turn_limit_spinup\": -0.0001", false},
            {"motion_turn_scale_zero", "\"motion_turn_limit_spinup\": 0", true},
            {"motion_turn_scale_one", "\"motion_turn_limit_spinup\": 1", true},
            {"motion_turn_scale_above_one", "\"motion_turn_limit_spinup\": 1.0001", false},
            {"motion_turn_scale_nonfinite_finding", "\"motion_turn_limit_spinup\": 1e999", false},
            {"motion_stop_negative", "\"motion_stop_ms\": -1", false},
            {"motion_stop_zero_immediate", "\"motion_stop_ms\": 0", true},
            {"motion_stop_max", "\"motion_stop_ms\": " + maximum_interval, true},
            {"motion_stop_above_max", "\"motion_stop_ms\": " + above_maximum_interval, false},
            {"motion_stop_encoder_negative", "\"motion_stop_encoder_threshold\": -1", false},
            {"motion_stop_encoder_zero", "\"motion_stop_encoder_threshold\": 0", true},
            {"motion_stop_encoder_speed_domain_max", "\"motion_stop_encoder_threshold\": 5000", true},
            {"motion_stop_encoder_above_speed_domain", "\"motion_stop_encoder_threshold\": 5001", false},
            {"motion_fault_hold_negative", "\"motion_fault_rearm_hold_ms\": -1", false},
            {"motion_fault_hold_zero_immediate", "\"motion_fault_rearm_hold_ms\": 0", true},
            {"motion_fault_hold_max", "\"motion_fault_rearm_hold_ms\": " + maximum_interval, true},
            {"motion_fault_hold_above_max", "\"motion_fault_rearm_hold_ms\": " + above_maximum_interval, false},
            {"snapshot_interval_zero", "\"control_snapshot_emit_interval_ms\": 0", false},
            {"snapshot_interval_one", "\"control_snapshot_emit_interval_ms\": 1", true},
            {"snapshot_interval_max", "\"control_snapshot_emit_interval_ms\": " + maximum_interval, true},
            {"snapshot_interval_above_max", "\"control_snapshot_emit_interval_ms\": " + above_maximum_interval, false},
            {"media_port_negative", "\"steering_media_port\": -1", false},
            {"media_port_one", "\"steering_media_port\": 1", true},
            {"media_port_max", "\"steering_media_port\": 65535", true},
            {"media_port_65536_finding", "\"steering_media_port\": 65536", false},
            {"disabled_media_invalid_port", "\"steering_media_enabled\": 0, \"steering_media_port\": 65536", false},
            {"media_interval_negative", "\"steering_media_publish_interval_ms\": -1", false},
            {"media_interval_zero_each_tick", "\"steering_media_publish_interval_ms\": 0", true},
            {"media_interval_max", "\"steering_media_publish_interval_ms\": " + maximum_interval, true},
            {"media_interval_above_max", "\"steering_media_publish_interval_ms\": " + above_maximum_interval, false},
            {"low_voltage_sample_zero", "\"low_voltage_sample_interval_ms\": 0", false},
            {"low_voltage_sample_one", "\"low_voltage_sample_interval_ms\": 1", true},
            {"low_voltage_sample_max", "\"low_voltage_sample_interval_ms\": " + maximum_interval, true},
            {"low_voltage_sample_above_max", "\"low_voltage_sample_interval_ms\": " + above_maximum_interval, false},
            {"brushless_pwm_negative", "\"brushless_debug_fixed_pwm\": -1", false},
            {"brushless_pwm_zero", "\"brushless_debug_fixed_pwm\": 0", true},
            {"brushless_pwm_max", "\"brushless_debug_fixed_pwm\": 1000", true},
            {"brushless_pwm_above_max", "\"brushless_debug_fixed_pwm\": 1001", false},
        };
        for (const OptionalSemanticCase& test_case : optional_semantic_cases) {
            verify_optional_semantic_case(test_case.name, test_case.block, test_case.valid);
        }

        struct AssistantPortCase {
            const char* name;
            const char* value;
            bool valid;
        };
        const std::vector<AssistantPortCase> assistant_port_cases = {
            {"negative_finding", "-1", false},
            {"zero", "0", false},
            {"one", "1", true},
            {"max", "65535", true},
            {"65536_finding", "65536", false},
        };
        for (const AssistantPortCase& test_case : assistant_port_cases) {
            const std::string path =
                base + "_assistant_port_" + test_case.name + ".json";
            WriteText(path, RuntimeParametersJsonWithIntegerValues("5", test_case.value));
            if (test_case.valid) {
                CaptureDiagnostics diagnostics{};
                (void)LoadFixture(path, diagnostics);
            } else {
                ExpectRejectedFixture(path,
                                      "params.validation",
                                      "missing, malformed, or out-of-range required field");
            }
        }
        const std::string disabled_assistant_invalid_port_path =
            base + "_assistant_port_disabled_invalid.json";
        WriteText(disabled_assistant_invalid_port_path,
                  ReplaceFirst(MinimalRuntimeParametersJson("  \"assistant_enabled\": 0"),
                               "\"port\": 8888",
                               "\"port\": 65536"));
        ExpectRejectedFixture(disabled_assistant_invalid_port_path,
                              "params.validation",
                              "missing, malformed, or out-of-range required field");

        const std::array<char, 3> yaw_components = {'P', 'I', 'D'};
        for (char component : yaw_components) {
            const float accepted_boundary = MaximumAcceptedSingleYawGain(component);
            const float rejected_boundary =
                std::nextafter(accepted_boundary, std::numeric_limits<float>::infinity());
            const auto yaw_json = [component](const std::string& value) {
                return RuntimeParametersJsonWithYawPid(component == 'P' ? value : "0",
                                                       component == 'I' ? value : "0",
                                                       component == 'D' ? value : "0");
            };

            const std::string accepted_path =
                base + "_yaw_" + component + "_exact_accepted.json";
            WriteText(accepted_path, yaw_json(JsonNumber(accepted_boundary)));
            CaptureDiagnostics accepted_diagnostics{};
            const ls2k::port::RuntimeParameters accepted =
                LoadFixture(accepted_path, accepted_diagnostics);
            Expect(ls2k::control::YawRatePidArithmeticIsFinite(
                       accepted.yaw_rate_pid_p,
                       accepted.yaw_rate_pid_i,
                       accepted.yaw_rate_pid_d),
                   std::string("exact accepted YAW_RATE_PID.") + component +
                       " boundary must preserve the production arithmetic contract");

            const std::string rejected_path =
                base + "_yaw_" + component + "_adjacent_rejected.json";
            WriteText(rejected_path, yaw_json(JsonNumber(rejected_boundary)));
            ExpectRejectedFixture(rejected_path,
                                  "params.validation",
                                  "malformed or out-of-range optional field");

            const std::string negative_accepted_path =
                base + "_yaw_" + component + "_exact_negative_accepted.json";
            WriteText(negative_accepted_path, yaw_json(JsonNumber(-accepted_boundary)));
            CaptureDiagnostics negative_accepted_diagnostics{};
            const ls2k::port::RuntimeParameters negative_accepted =
                LoadFixture(negative_accepted_path, negative_accepted_diagnostics);
            Expect(ls2k::control::YawRatePidArithmeticIsFinite(
                       negative_accepted.yaw_rate_pid_p,
                       negative_accepted.yaw_rate_pid_i,
                       negative_accepted.yaw_rate_pid_d),
                   std::string("exact negative YAW_RATE_PID.") + component +
                       " boundary must be accepted");

            const std::string negative_rejected_path =
                base + "_yaw_" + component + "_adjacent_negative_rejected.json";
            WriteText(negative_rejected_path, yaw_json(JsonNumber(-rejected_boundary)));
            ExpectRejectedFixture(negative_rejected_path,
                                  "params.validation",
                                  "malformed or out-of-range optional field");

            for (const std::string& rejected_value : {std::string("1e308"),
                                                       std::string("1e999"),
                                                       std::string("-1e999")}) {
                const std::string path =
                    base + "_yaw_" + component + "_rejected_" +
                    std::to_string(rejected_value.size()) + "_" +
                    std::to_string(rejected_value.front() == '-' ? 1 : 0) + ".json";
                WriteText(path, yaw_json(rejected_value));
                ExpectRejectedFixture(path,
                                      "params.validation",
                                      "malformed or out-of-range optional field");
            }
        }

        const std::vector<std::pair<double, bool>> speed_boundaries = {
            {-1.0, false}, {0.0, true}, {5000.0, true}, {5001.0, false},
        };
        for (std::size_t i = 0; i < speed_boundaries.size(); ++i) {
            const std::string path = base + "_speed_boundary_" + std::to_string(i) + ".json";
            WriteText(path,
                      RuntimeParametersJsonWithControlValues(speed_boundaries[i].first,
                                                             0.5,
                                                             0.5));
            if (speed_boundaries[i].second) {
                CaptureDiagnostics diagnostics{};
                const ls2k::port::RuntimeParameters params = LoadFixture(path, diagnostics);
                Expect(params.running_speed_target == speed_boundaries[i].first,
                       "valid RUNNING_SPEED_TARGET boundary must be preserved");
            } else {
                ExpectRejectedFixture(path,
                                      "params.validation",
                                      "missing, malformed, or out-of-range required field");
            }
        }

        struct PwmFloorCase {
            int floor;
            bool valid;
        };
        const std::vector<PwmFloorCase> pwm_floor_boundaries = {
            {-1, false}, {0, true}, {5000, true}, {5001, false},
        };
        for (std::size_t i = 0; i < pwm_floor_boundaries.size(); ++i) {
            const PwmFloorCase& test_case = pwm_floor_boundaries[i];
            const std::string path = base + "_pwm_floor_boundary_" +
                                     std::to_string(i) + ".json";
            WriteText(path,
                      MinimalRuntimeParametersJson(
                          "  \"pwm_limit\": 5000, \"pwm_floor\": " +
                          std::to_string(test_case.floor)));
            if (test_case.valid) {
                CaptureDiagnostics diagnostics{};
                const ls2k::port::RuntimeParameters params = LoadFixture(path, diagnostics);
                Expect(params.pwm_floor == test_case.floor,
                       "valid pwm_floor boundary must be preserved");
            } else {
                ExpectRejectedFixture(path,
                                      "params.validation",
                                      "malformed or out-of-range optional field");
            }
        }

        struct InvalidRequiredPidCase {
            const char* name;
            const char* needle;
            const char* replacement;
        };
        const std::vector<InvalidRequiredPidCase> invalid_required_pid_values = {
            {"left_p_nonfinite", "\"LEFT_WHEEL_PID\": {\"P\": 0",
             "\"LEFT_WHEEL_PID\": {\"P\": 1e999"},
            {"left_i_nonfinite", "\"LEFT_WHEEL_PID\": {\"P\": 0, \"I\": 0",
             "\"LEFT_WHEEL_PID\": {\"P\": 0, \"I\": 1e999"},
            {"left_d_nonfinite", "\"LEFT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0",
             "\"LEFT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 1e999"},
            {"left_integral_nonfinite", "\"LEFT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, \"INTEGRAL_LIMIT\": 1000",
             "\"LEFT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, \"INTEGRAL_LIMIT\": 1e999"},
            {"left_integral_negative", "\"LEFT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, \"INTEGRAL_LIMIT\": 1000",
             "\"LEFT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, \"INTEGRAL_LIMIT\": -1"},
            {"right_p_nonfinite", "\"RIGHT_WHEEL_PID\": {\"P\": 0",
             "\"RIGHT_WHEEL_PID\": {\"P\": 1e999"},
            {"right_i_nonfinite", "\"RIGHT_WHEEL_PID\": {\"P\": 0, \"I\": 0",
             "\"RIGHT_WHEEL_PID\": {\"P\": 0, \"I\": 1e999"},
            {"right_d_nonfinite", "\"RIGHT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0",
             "\"RIGHT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 1e999"},
            {"right_integral_nonfinite", "\"RIGHT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, \"INTEGRAL_LIMIT\": 1000",
             "\"RIGHT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, \"INTEGRAL_LIMIT\": 1e999"},
            {"right_integral_negative", "\"RIGHT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, \"INTEGRAL_LIMIT\": 1000",
             "\"RIGHT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, \"INTEGRAL_LIMIT\": -1"},
        };
        for (const InvalidRequiredPidCase& test_case : invalid_required_pid_values) {
            const std::string path = base + "_" + test_case.name + ".json";
            WriteText(path,
                      ReplaceFirst(MinimalRuntimeParametersJson(""),
                                   test_case.needle,
                                   test_case.replacement));
            ExpectRejectedFixture(path,
                                  "params.validation",
                                  "missing, malformed, or out-of-range required field");
        }

        struct FilterAlphaCase {
            double left;
            double right;
            bool valid;
        };
        const std::vector<FilterAlphaCase> filter_alpha_boundaries = {
            {-0.01, 0.5, false},
            {0.0, 0.5, true},
            {1.0, 0.5, true},
            {1.01, 0.5, false},
            {0.5, -0.01, false},
            {0.5, 0.0, true},
            {0.5, 1.0, true},
            {0.5, 1.01, false},
        };
        for (std::size_t i = 0; i < filter_alpha_boundaries.size(); ++i) {
            const FilterAlphaCase& test_case = filter_alpha_boundaries[i];
            const std::string path = base + "_filter_alpha_boundary_" +
                                     std::to_string(i) + ".json";
            WriteText(path,
                      RuntimeParametersJsonWithControlValues(300.0,
                                                             test_case.left,
                                                             test_case.right));
            if (test_case.valid) {
                CaptureDiagnostics diagnostics{};
                const ls2k::port::RuntimeParameters params = LoadFixture(path, diagnostics);
                Expect(params.left_wheel_pid.measurement_filter_alpha == test_case.left &&
                           params.right_wheel_pid.measurement_filter_alpha == test_case.right,
                       "valid MEASUREMENT_FILTER_ALPHA boundary must be preserved");
            } else {
                ExpectRejectedFixture(path,
                                      "params.validation",
                                      "malformed or out-of-range optional field");
            }
        }

        const std::string nonfinite_left_alpha_path = base + "_nonfinite_left_alpha.json";
        WriteText(nonfinite_left_alpha_path,
                  ReplaceFirst(RuntimeParametersJsonWithControlValues(300.0, 0.5, 0.5),
                               "\"MEASUREMENT_FILTER_ALPHA\": 0.5",
                               "\"MEASUREMENT_FILTER_ALPHA\": 1e999"));
        ExpectRejectedFixture(nonfinite_left_alpha_path,
                              "params.validation",
                              "malformed or out-of-range optional field");

        const std::string nonfinite_right_alpha_path = base + "_nonfinite_right_alpha.json";
        std::string nonfinite_right_alpha = RuntimeParametersJsonWithControlValues(300.0, 0.5, 0.5);
        const std::size_t first_alpha =
            nonfinite_right_alpha.find("\"MEASUREMENT_FILTER_ALPHA\": 0.5");
        Expect(first_alpha != std::string::npos, "left alpha fixture token not found");
        const std::size_t second_alpha =
            nonfinite_right_alpha.find("\"MEASUREMENT_FILTER_ALPHA\": 0.5", first_alpha + 1);
        Expect(second_alpha != std::string::npos, "right alpha fixture token not found");
        nonfinite_right_alpha.replace(second_alpha,
                                      std::string("\"MEASUREMENT_FILTER_ALPHA\": 0.5").size(),
                                      "\"MEASUREMENT_FILTER_ALPHA\": 1e999");
        WriteText(nonfinite_right_alpha_path, nonfinite_right_alpha);
        ExpectRejectedFixture(nonfinite_right_alpha_path,
                              "params.validation",
                              "malformed or out-of-range optional field");

        const std::string invalid_optional_value_path = base + "_invalid_optional_value.json";
        WriteText(invalid_optional_value_path,
                  MinimalRuntimeParametersJson("  \"steering_media_downsample\": 0"));
        ExpectRejectedFixture(invalid_optional_value_path,
                              "params.validation",
                              "malformed or out-of-range optional field");

        const std::vector<std::pair<std::string, std::string>> rejected_optional_values = {
            {"invalid_tflite_identity",
             "  \"ML\": {\"TFLITE_IDENTITY\": {\"MAX_BEST_DISTANCE\": 260101}}"},
            {"invalid_cross_sampleable",
             "  \"BEV_ELEMENT\": {\"CROSS_MIN_SAMPLEABLE_PER_ROW\": 0}"},
            {"invalid_cross_connectivity_index_negative",
             "  \"BEV_ELEMENT\": {\"CROSS_CONNECTIVITY_SAMPLE_INDEX\": -1}"},
            {"invalid_cross_connectivity_index_too_large",
             "  \"BEV_ELEMENT\": {\"CROSS_CONNECTIVITY_SAMPLE_INDEX\": 24}"},
            {"invalid_cross_expansion_distance",
             "  \"BEV_ELEMENT\": {\"CROSS_BOUNDARY_EXPANSION_MIN_M\": 0}"},
            {"invalid_zebra_roi",
             "  \"BEV_ELEMENT\": {\"ZEBRA_FORWARD_MIN_M\": 0.6,"
             "\"ZEBRA_FORWARD_MAX_M\": 0.5}"},
            {"invalid_zebra_jump_count",
             "  \"BEV_ELEMENT\": {\"ZEBRA_MIN_JUMPS_PER_ROW\": 1}"},
            {"invalid_zebra_adjacent_gap",
             "  \"BEV_ELEMENT\": {\"ZEBRA_MAX_ADJACENT_FORWARD_GAP_M\": 0}"},
            {"invalid_zebra_support_span",
             "  \"BEV_ELEMENT\": {\"ZEBRA_MIN_SUPPORT_FORWARD_SPAN_M\": 0}"},
            {"invalid_zebra_absence_ms",
             "  \"BEV_ELEMENT\": {\"ZEBRA_REENTRY_ARM_ABSENCE_MS\": -1}"},
            {"invalid_zebra_stop_delay_ms",
             "  \"BEV_ELEMENT\": {\"ZEBRA_CONTROLLED_STOP_DELAY_MS\": 86400001}"},
            {"invalid_geometry",
             "  \"BEV_GEOMETRY\": {\"NOMINAL_ROAD_HALF_WIDTH_M\": 0}"},
            {"invalid_boundary_trace",
             "  \"BEV_GEOMETRY\": {\"BOUNDARY_TRACE_MAX_ADJACENT_DISTANCE_M\": 0}"},
            {"invalid_classification",
             "  \"BEV_CLASSIFICATION\": {\"WHITE_CONFIDENCE_MIN\": 0.2,"
             "\"UNKNOWN_CONFIDENCE_MIN\": 0.4}"},
            {"invalid_unknown_confidence",
             "  \"BEV_CLASSIFICATION\": {\"UNKNOWN_CONFIDENCE_MIN\": 0}"},
            {"invalid_hold_cycles",
             "  \"BEV_CLASSIFICATION\": {\"HOLD_LAST_MAX_CYCLES\": -1}"},
            {"invalid_v2_angle_order",
             "  \"BEV_ELEMENT\": {\"CIRCLE_V2_NORMAL_TRACE_START_YAW_DEG\": 300,"
             "\"CIRCLE_V2_EXIT_TRACE_START_YAW_DEG\": 200}"},
            {"invalid_v2_calm_time",
             "  \"BEV_ELEMENT\": {\"CIRCLE_V2_CALM_TRACE_MS\": 0}"},
            {"invalid_v2_cooldown_time",
             "  \"BEV_ELEMENT\": {\"CIRCLE_V2_COOLDOWN_MS\": -1}"},
            {"invalid_v2_tangent_span",
             "  \"BEV_ELEMENT\": {\"CIRCLE_V2_EXIT_TANGENT_FIT_SPAN_M\": 0}"},
            {"invalid_v2_stall_timeout",
             "  \"BEV_ELEMENT\": {\"CIRCLE_V2_INNER_TRACE_STALL_TIMEOUT_MS\": 0}"},
            {"invalid_v2_path_offset",
             "  \"BEV_ELEMENT\": {\"CIRCLE_V2_INNER_TRACE_PATH_OFFSET_M\": -0.1}"},
            {"invalid_v2_opposite_confidence",
             "  \"BEV_ELEMENT\": {\"CIRCLE_V2_OPPOSITE_STRAIGHT_CONFIDENCE_MIN\": 1.2}"},
            {"invalid_v2_entry_forward",
             "  \"BEV_ELEMENT\": {\"CIRCLE_V2_ENTRY_FORWARD_MIN_M\": 0.9,"
             "\"CIRCLE_V2_ENTRY_FORWARD_MAX_M\": 0.8}"},
            {"invalid_v2_opening_forward",
             "  \"BEV_ELEMENT\": {\"CIRCLE_V2_OPENING_FORWARD_MIN_M\": 1.0,"
             "\"CIRCLE_V2_OPENING_FORWARD_MAX_M\": 0.5}"},
            {"invalid_v2_inner_geometry_forward",
             "  \"BEV_ELEMENT\": {\"CIRCLE_V2_INNER_GEOMETRY_FORWARD_MIN_M\": 0.8,"
             "\"CIRCLE_V2_INNER_GEOMETRY_FORWARD_MAX_M\": 0.2}"},
            {"invalid_brushless_pwm", "  \"brushless_debug_fixed_pwm\": 1200"},
            {"invalid_gray_bits", "  \"steering_media_gray_bits\": 5"},
            {"invalid_accel_scale", "  \"wheel_turn_accel_delta_scale\": -0.01"},
            {"malformed_decel_scale", "  \"wheel_turn_decel_delta_scale\": \"bad\""},
            {"invalid_drive_step_zero", "  \"drive_pwm_step_limit\": 0"},
            {"invalid_pwm_limit_above_drive_capability", "  \"pwm_limit\": 9001"},
            {"invalid_drive_step_above_pwm_limit",
             "  \"pwm_limit\": 8000, \"drive_pwm_step_limit\": 8001"},
            {"unsupported_camera",
             "  \"CAMERA_SOURCE\": {\"BACKEND\": \"unsupported_backend\"}"},
        };
        for (const auto& rejected : rejected_optional_values) {
            const std::string path = base + "_" + rejected.first + ".json";
            WriteText(path, MinimalRuntimeParametersJson(rejected.second));
            ExpectRejectedFixture(path,
                                  "params.validation",
                                  "malformed or out-of-range optional field");
        }

        const std::string zero_hold_path = base + "_zero_hold.json";
        WriteText(zero_hold_path,
                  MinimalRuntimeParametersJson(
                      "  \"BEV_CLASSIFICATION\": {"
                      "\"HOLD_LAST_MAX_CYCLES\": 0}"));
        CaptureDiagnostics zero_hold_diagnostics{};
        const ls2k::port::RuntimeParameters zero_hold =
            LoadFixture(zero_hold_path, zero_hold_diagnostics);
        Expect(!zero_hold.loaded_from_defaults,
               "zero BEV hold cycles should parse as hold disabled");
        Expect(!zero_hold.parse_failure,
               "zero BEV hold cycles should not set parse_failure");
        Expect(zero_hold.bev_classification.hold_last_max_cycles == 0,
               "zero BEV hold cycles should be preserved");

        const std::string max_drive_pwm_path = base + "_max_drive_pwm.json";
        WriteText(max_drive_pwm_path,
                  MinimalRuntimeParametersJson(
                      "  \"pwm_limit\": 9000, \"drive_pwm_step_limit\": 9000"));
        CaptureDiagnostics max_drive_pwm_diagnostics{};
        const ls2k::port::RuntimeParameters max_drive_pwm =
            LoadFixture(max_drive_pwm_path, max_drive_pwm_diagnostics);
        Expect(max_drive_pwm.pwm_limit == 9000 &&
                   max_drive_pwm.drive_pwm_step_limit == 9000,
               "drive PWM capability boundary should be accepted and preserved");

        const std::string legacy_reference_jump_path =
            base + "_legacy_reference_jump.json";
        WriteText(legacy_reference_jump_path,
                  MinimalRuntimeParametersJson(
                      "  \"BEV_GEOMETRY\": {"
                      "\"REFERENCE_LATERAL_JUMP_GATE_M\": -0.1}"));
        CaptureDiagnostics legacy_reference_jump_diagnostics{};
        const ls2k::port::RuntimeParameters legacy_reference_jump =
            LoadFixture(legacy_reference_jump_path,
                        legacy_reference_jump_diagnostics);
        Expect(!legacy_reference_jump.loaded_from_defaults,
               "removed reference jump gate key should not trigger fallback");
        Expect(!legacy_reference_jump.parse_failure,
               "removed reference jump gate key should be ignored cleanly");
        Expect(std::abs(legacy_reference_jump.bev_geometry
                            .boundary_trace_max_adjacent_distance_m -
                        builtin_defaults.bev_geometry
                            .boundary_trace_max_adjacent_distance_m) < 1.0e-6F,
               "removed reference jump gate key must not alter current geometry defaults");
        Expect(!legacy_reference_jump_diagnostics.SawCode("params.validation"),
               "removed reference jump gate key should not emit params.validation");
        const std::string single_dequeue_path = base + "_single_dequeue.json";
        WriteText(single_dequeue_path,
                  MinimalRuntimeParametersJson(
                      "  \"CAMERA_SOURCE\": {\"BACKEND\": \"v4l2_yuyv\","
                      "\"DRAIN_READY_BUFFERS\": 0}"));
        CaptureDiagnostics single_dequeue_diagnostics{};
        const ls2k::port::RuntimeParameters single_dequeue =
            LoadFixture(single_dequeue_path, single_dequeue_diagnostics);
        Expect(!single_dequeue.loaded_from_defaults && !single_dequeue.parse_failure,
               "disabled camera drain should pass validation");
        Expect(!single_dequeue.camera_source.drain_ready_buffers,
               "DRAIN_READY_BUFFERS=0 should map to single-dequeue policy");

        std::cout << "param_store_load_runtime_parameters_test passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "param_store_load_runtime_parameters_test failed: " << error.what()
                  << "\n";
        return 1;
    }
}
