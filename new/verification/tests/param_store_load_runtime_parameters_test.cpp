#include <fstream>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "platform/bootstrap.hpp"

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
        "\"CIRCLE_ENTRY_",
        "\"CIRCLE_EVIDENCE_",
        "\"CIRCLE_MIN_",
        "\"CIRCLE_OPEN",
        "\"CIRCLE_OPPOSITE_",
        "\"CIRCLE_PRESENT_",
        "\"delta_s_m\"",
        "\"BEV_ELEMENT_RASTER\"",
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
                      "\"MANEUVER\": {\"SPEED_TARGET\": 100,\"MIN_BOUNDARY_SAMPLES\": 3,"
                      "\"EXIT_FORWARD_M\": 0.5,\"EXIT_MAX_ABS_LATERAL_ERROR_M\": 0.1,"
                      "\"EXIT_MAX_ABS_HEADING_ERROR_RAD\": 0.2,\"MAX_DURATION_MS\": 1000,"
                      "\"MAX_INTEGRATION_GAP_MS\": 30,\"COOLDOWN_MS\": 200}},\n"
                      "  \"BEV_GEOMETRY\": {"
                      "\"NOMINAL_ROAD_HALF_WIDTH_M\": 0.33,"
                      "\"SPARSE_ROW_COUNT\": 12,"
                      "\"BOUNDARY_TRACE_MAX_ADJACENT_DISTANCE_M\": 0.37},\n"
                      "  \"BEV_CLASSIFICATION\": {"
                      "\"WHITE_CONFIDENCE_MIN\": 0.66,"
                      "\"UNKNOWN_CONFIDENCE_MIN\": 0.33,"
                      "\"HOLD_LAST_MAX_CYCLES\": 24},\n"
                      "  \"BEV_BOUNDARY\": {"
                      "\"LOCAL_JUMP_MIN_Y\": 41},\n"
                      "  \"BEV_CONTROL_MODEL\": {"
                      "\"LATERAL_OFFSET_TO_WHEEL_DELTA_GAIN\": 321,"
                      "\"HEADING_ERROR_TO_WHEEL_DELTA_GAIN\": 45,"
                      "\"CURVATURE_TO_WHEEL_DELTA_GAIN\": 67,"
                      "\"TRACKING_FIT_MIN_SAMPLES\": 5},\n"
                      "  \"BEV_ELEMENT\": {"
                      "\"CROSS_EXIT_TAKEOVER_ENABLED\": 1,"
                      "\"CROSS_MIN_SAMPLEABLE_PER_ROW\": 11,"
                      "\"CIRCLE_V2_ENABLED\": 1,"
                      "\"CIRCLE_V2_EXIT_YAW_THRESHOLD_DEG\": 300,"
                      "\"CIRCLE_V2_EXIT_HOLD_FRAMES\": 4,"
                      "\"CIRCLE_V2_INNER_TRACE_STALL_TIMEOUT_MS\": 4500,"
                      "\"CIRCLE_V2_INNER_TRACE_STALL_YAW_MIN_DEG\": 12.5,"
                      "\"CIRCLE_V2_INNER_TRACE_PATH_OFFSET_M\": 0.07,"
                      "\"CIRCLE_V2_OPPOSITE_STRAIGHT_CONFIDENCE_MIN\": 0.63,"
                      "\"CIRCLE_V2_ENTRY_BOTTOM_MIN_ROW_COUNT\": 3,"
                      "\"CIRCLE_V2_ENTRY_BOTTOM_FORWARD_MIN_M\": 0.05,"
                      "\"CIRCLE_V2_ENTRY_BOTTOM_FORWARD_MAX_M\": 0.85}"));
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
        Expect(enabled.ml.enabled &&
                   std::abs(enabled.motion_odometry.encoder_ticks_to_meter - 0.001) < 1.0e-9,
               "enabled ML must parse the shared motion-odometry scale");
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
                   std::abs(enabled.ml.maneuver.speed_target - 100.0) < 1.0e-9,
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
        Expect(enabled.bev_boundary.local_jump_min_y == 41,
               "BEV_BOUNDARY.LOCAL_JUMP_MIN_Y should parse");
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
        Expect(enabled.bev_element.cross_exit_takeover_enabled,
               "CROSS_EXIT_TAKEOVER_ENABLED=1 should parse true");
        Expect(enabled.bev_element.cross_min_sampleable_per_row == 11,
               "CROSS_MIN_SAMPLEABLE_PER_ROW should parse");
        Expect(enabled.bev_element.circle_v2_enabled,
               "CIRCLE_V2_ENABLED=1 should parse true");
        Expect(std::abs(enabled.bev_element.circle_v2_exit_yaw_threshold_deg - 300.0F) <
                   1.0e-6F,
               "CIRCLE_V2_EXIT_YAW_THRESHOLD_DEG should parse");
        Expect(enabled.bev_element.circle_v2_exit_hold_frames == 4,
               "CIRCLE_V2_EXIT_HOLD_FRAMES should parse");
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
        Expect(enabled.bev_element.circle_v2_entry_bottom_min_row_count == 3,
               "CIRCLE_V2_ENTRY_BOTTOM_MIN_ROW_COUNT should parse");
        Expect(std::abs(enabled.bev_element.circle_v2_entry_bottom_forward_min_m -
                        0.05F) < 1.0e-6F,
               "CIRCLE_V2_ENTRY_BOTTOM_FORWARD_MIN_M should parse");
        Expect(std::abs(enabled.bev_element.circle_v2_entry_bottom_forward_max_m -
                        0.85F) < 1.0e-6F,
               "CIRCLE_V2_ENTRY_BOTTOM_FORWARD_MAX_M should parse");
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
        Expect(absent.bev_element.cross_exit_takeover_enabled ==
                   builtin_defaults.bev_element.cross_exit_takeover_enabled,
               "missing BEV_ELEMENT should keep takeover enabled");
        Expect(absent.bev_element.cross_min_sampleable_per_row ==
                   builtin_defaults.bev_element.cross_min_sampleable_per_row,
               "missing BEV_ELEMENT should keep cross sampleable threshold default");
        Expect(absent.bev_boundary.local_jump_min_y ==
                   builtin_defaults.bev_boundary.local_jump_min_y,
               "missing BEV_BOUNDARY should keep local jump default");
        Expect(absent.bev_element.circle_v2_enabled ==
                   builtin_defaults.bev_element.circle_v2_enabled,
               "missing BEV_ELEMENT should keep CircleV2 enabled");
        Expect(std::abs(absent.bev_element.circle_v2_exit_yaw_threshold_deg -
                        builtin_defaults.bev_element.circle_v2_exit_yaw_threshold_deg) < 1.0e-6F,
               "missing BEV_ELEMENT should keep CircleV2 yaw threshold default");
        Expect(absent.bev_element.circle_v2_exit_hold_frames ==
                   builtin_defaults.bev_element.circle_v2_exit_hold_frames,
               "missing BEV_ELEMENT should keep CircleV2 hold default");
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
        Expect(absent.bev_element.circle_v2_entry_bottom_min_row_count ==
                   builtin_defaults.bev_element.circle_v2_entry_bottom_min_row_count,
               "missing BEV_ELEMENT should keep CircleV2 entry bottom min-row-count default");
        Expect(std::abs(absent.bev_element.circle_v2_entry_bottom_forward_min_m -
                        builtin_defaults.bev_element.circle_v2_entry_bottom_forward_min_m) < 1.0e-6F,
               "missing BEV_ELEMENT should keep CircleV2 entry bottom min default");
        Expect(std::abs(absent.bev_element.circle_v2_entry_bottom_forward_max_m -
                        builtin_defaults.bev_element.circle_v2_entry_bottom_forward_max_m) < 1.0e-6F,
               "missing BEV_ELEMENT should keep CircleV2 entry bottom max default");
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

        const std::string invalid_tflite_identity_path = base + "_invalid_tflite_identity.json";
        WriteText(invalid_tflite_identity_path,
                  MinimalRuntimeParametersJson(
                      "  \"ML\": {\"TFLITE_IDENTITY\": {\"MAX_BEST_DISTANCE\": 260101}}"));
        CaptureDiagnostics invalid_tflite_identity_diagnostics{};
        const ls2k::port::RuntimeParameters invalid_tflite_identity =
            LoadFixture(invalid_tflite_identity_path, invalid_tflite_identity_diagnostics);
        Expect(invalid_tflite_identity.loaded_from_defaults &&
                   invalid_tflite_identity.parse_failure,
               "TFLITE_IDENTITY distance above d4 squared-L2 maximum must fail startup parsing");
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

        const std::string malformed_path = base + "_malformed.json";
        WriteText(malformed_path,
                  MinimalRuntimeParametersJson(
                      "  \"BEV_ELEMENT\": {\"CROSS_EXIT_TAKEOVER_ENABLED\": {\"bad\": 1}}"));
        CaptureDiagnostics malformed_diagnostics{};
        const ls2k::port::RuntimeParameters malformed =
            LoadFixture(malformed_path, malformed_diagnostics);
        Expect(malformed.loaded_from_defaults,
               "malformed CROSS_EXIT_TAKEOVER_ENABLED should fall back to defaults");
        Expect(malformed.parse_failure,
               "malformed CROSS_EXIT_TAKEOVER_ENABLED should set parse_failure");
        Expect(malformed.bev_element.cross_exit_takeover_enabled,
               "malformed CROSS_EXIT_TAKEOVER_ENABLED should fall back to takeover enabled default");
        Expect(malformed_diagnostics.SawCode("params.parse"),
               "malformed CROSS_EXIT_TAKEOVER_ENABLED should emit params.parse");

        const std::string invalid_cross_sampleable_path = base + "_invalid_cross_sampleable.json";
        WriteText(invalid_cross_sampleable_path,
                  MinimalRuntimeParametersJson(
                      "  \"BEV_ELEMENT\": {\"CROSS_MIN_SAMPLEABLE_PER_ROW\": 0}"));
        CaptureDiagnostics invalid_cross_sampleable_diagnostics{};
        const ls2k::port::RuntimeParameters invalid_cross_sampleable =
            LoadFixture(invalid_cross_sampleable_path, invalid_cross_sampleable_diagnostics);
        Expect(invalid_cross_sampleable.loaded_from_defaults,
               "zero CROSS_MIN_SAMPLEABLE_PER_ROW should fall back to defaults");
        Expect(invalid_cross_sampleable.parse_failure,
               "zero CROSS_MIN_SAMPLEABLE_PER_ROW should set parse_failure");
        Expect(invalid_cross_sampleable.bev_element.cross_min_sampleable_per_row ==
                   builtin_defaults.bev_element.cross_min_sampleable_per_row,
               "invalid cross sampleable threshold should keep default");
        Expect(invalid_cross_sampleable_diagnostics.SawCode("params.parse"),
               "zero CROSS_MIN_SAMPLEABLE_PER_ROW should emit params.parse");

        const std::string malformed_geometry_path = base + "_malformed_geometry.json";
        WriteText(malformed_geometry_path,
                  MinimalRuntimeParametersJson(
                      "  \"BEV_GEOMETRY\": {\"NOMINAL_ROAD_HALF_WIDTH_M\": 0}"));
        CaptureDiagnostics malformed_geometry_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_geometry =
            LoadFixture(malformed_geometry_path, malformed_geometry_diagnostics);
        Expect(malformed_geometry.loaded_from_defaults,
               "zero nominal road half width should fall back to defaults");
        Expect(malformed_geometry.parse_failure,
               "zero nominal road half width should set parse_failure");
        Expect(std::abs(malformed_geometry.bev_geometry.nominal_road_half_width_m -
                        0.225F) < 1.0e-6F,
               "nominal road half-width fallback should keep default");
        Expect(malformed_geometry_diagnostics.SawCode("params.parse"),
               "zero nominal road half width should emit params.parse");

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
        Expect(!legacy_reference_jump_diagnostics.SawCode("params.parse"),
               "removed reference jump gate key should not emit params.parse");

        const std::string malformed_boundary_trace_path =
            base + "_malformed_boundary_trace.json";
        WriteText(malformed_boundary_trace_path,
                  MinimalRuntimeParametersJson(
                      "  \"BEV_GEOMETRY\": {"
                      "\"BOUNDARY_TRACE_MAX_ADJACENT_DISTANCE_M\": 0}"));
        CaptureDiagnostics malformed_boundary_trace_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_boundary_trace =
            LoadFixture(malformed_boundary_trace_path,
                        malformed_boundary_trace_diagnostics);
        Expect(malformed_boundary_trace.loaded_from_defaults,
               "zero boundary trace distance should fall back to defaults");
        Expect(malformed_boundary_trace.parse_failure,
               "zero boundary trace distance should set parse_failure");
        Expect(std::abs(malformed_boundary_trace.bev_geometry
                            .boundary_trace_max_adjacent_distance_m -
                        0.195660427F) < 1.0e-6F,
               "boundary trace distance fallback should keep default");
        Expect(malformed_boundary_trace_diagnostics.SawCode("params.parse"),
               "zero boundary trace distance should emit params.parse");

        const std::string malformed_classification_path =
            base + "_malformed_classification.json";
        WriteText(malformed_classification_path,
                  MinimalRuntimeParametersJson(
                      "  \"BEV_CLASSIFICATION\": {"
                      "\"WHITE_CONFIDENCE_MIN\": 0.2,"
                      "\"UNKNOWN_CONFIDENCE_MIN\": 0.4}"));
        CaptureDiagnostics malformed_classification_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_classification =
            LoadFixture(malformed_classification_path,
                        malformed_classification_diagnostics);
        Expect(malformed_classification.loaded_from_defaults,
               "inverted BEV classification confidence should fall back to defaults");
        Expect(malformed_classification.parse_failure,
               "inverted BEV classification confidence should set parse_failure");
        Expect(std::abs(malformed_classification.bev_classification
                            .white_confidence_min -
                        0.55F) < 1.0e-6F,
               "BEV classification fallback should keep white confidence default");
        Expect(std::abs(malformed_classification.bev_classification
                            .unknown_confidence_min -
                        0.25F) < 1.0e-6F,
               "BEV classification fallback should keep unknown confidence default");
        Expect(malformed_classification_diagnostics.SawCode("params.parse"),
               "inverted BEV classification confidence should emit params.parse");

        const std::string malformed_unknown_confidence_path =
            base + "_malformed_unknown_confidence.json";
        WriteText(malformed_unknown_confidence_path,
                  MinimalRuntimeParametersJson(
                      "  \"BEV_CLASSIFICATION\": {"
                      "\"UNKNOWN_CONFIDENCE_MIN\": 0}"));
        CaptureDiagnostics malformed_unknown_confidence_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_unknown_confidence =
            LoadFixture(malformed_unknown_confidence_path,
                        malformed_unknown_confidence_diagnostics);
        Expect(malformed_unknown_confidence.loaded_from_defaults,
               "zero BEV unknown confidence should fall back to defaults");
        Expect(malformed_unknown_confidence.parse_failure,
               "zero BEV unknown confidence should set parse_failure");
        Expect(std::abs(malformed_unknown_confidence.bev_classification
                            .unknown_confidence_min -
                        0.25F) < 1.0e-6F,
               "zero unknown confidence fallback should keep default");
        Expect(malformed_unknown_confidence_diagnostics.SawCode("params.parse"),
               "zero BEV unknown confidence should emit params.parse");

        const std::string malformed_hold_cycles_path =
            base + "_malformed_hold_cycles.json";
        WriteText(malformed_hold_cycles_path,
                  MinimalRuntimeParametersJson(
                      "  \"BEV_CLASSIFICATION\": {"
                      "\"HOLD_LAST_MAX_CYCLES\": -1}"));
        CaptureDiagnostics malformed_hold_cycles_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_hold_cycles =
            LoadFixture(malformed_hold_cycles_path,
                        malformed_hold_cycles_diagnostics);
        Expect(malformed_hold_cycles.loaded_from_defaults,
               "negative BEV hold cycles should fall back to defaults");
        Expect(malformed_hold_cycles.parse_failure,
               "negative BEV hold cycles should set parse_failure");
        Expect(malformed_hold_cycles.bev_classification.hold_last_max_cycles == 32,
               "negative hold cycles fallback should keep default");
        Expect(malformed_hold_cycles_diagnostics.SawCode("params.parse"),
               "negative BEV hold cycles should emit params.parse");

        const std::string malformed_v2_yaw_path = base + "_malformed_v2_yaw.json";
        WriteText(malformed_v2_yaw_path,
                  MinimalRuntimeParametersJson(
                      "  \"BEV_ELEMENT\": {\"CIRCLE_V2_EXIT_YAW_THRESHOLD_DEG\": 0}"));
        CaptureDiagnostics malformed_v2_yaw_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_v2_yaw =
            LoadFixture(malformed_v2_yaw_path, malformed_v2_yaw_diagnostics);
        Expect(malformed_v2_yaw.loaded_from_defaults,
               "zero CircleV2 yaw threshold should fall back to defaults");
        Expect(malformed_v2_yaw.parse_failure,
               "zero CircleV2 yaw threshold should set parse_failure");
        Expect(std::abs(malformed_v2_yaw.bev_element.circle_v2_exit_yaw_threshold_deg -
                        builtin_defaults.bev_element.circle_v2_exit_yaw_threshold_deg) < 1.0e-6F,
               "CircleV2 yaw fallback should keep nonzero default threshold");
        Expect(malformed_v2_yaw_diagnostics.SawCode("params.parse"),
               "zero CircleV2 yaw threshold should emit params.parse");

        const std::string malformed_v2_hold_path = base + "_malformed_v2_hold.json";
        WriteText(malformed_v2_hold_path,
                  MinimalRuntimeParametersJson(
                      "  \"BEV_ELEMENT\": {\"CIRCLE_V2_EXIT_HOLD_FRAMES\": 1}"));
        CaptureDiagnostics malformed_v2_hold_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_v2_hold =
            LoadFixture(malformed_v2_hold_path, malformed_v2_hold_diagnostics);
        Expect(malformed_v2_hold.loaded_from_defaults,
               "CircleV2 hold below two should fall back to defaults");
        Expect(malformed_v2_hold.parse_failure,
               "CircleV2 hold below two should set parse_failure");
        Expect(malformed_v2_hold.bev_element.circle_v2_exit_hold_frames ==
                   builtin_defaults.bev_element.circle_v2_exit_hold_frames,
               "CircleV2 hold fallback should keep default hold frames");
        Expect(malformed_v2_hold_diagnostics.SawCode("params.parse"),
               "CircleV2 hold below two should emit params.parse");

        const std::string malformed_v2_stall_timeout_path =
            base + "_malformed_v2_stall_timeout.json";
        WriteText(malformed_v2_stall_timeout_path,
                  MinimalRuntimeParametersJson(
                      "  \"BEV_ELEMENT\": {\"CIRCLE_V2_INNER_TRACE_STALL_TIMEOUT_MS\": 0}"));
        CaptureDiagnostics malformed_v2_stall_timeout_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_v2_stall_timeout =
            LoadFixture(malformed_v2_stall_timeout_path,
                        malformed_v2_stall_timeout_diagnostics);
        Expect(malformed_v2_stall_timeout.loaded_from_defaults,
               "CircleV2 stall timeout below one should fall back to defaults");
        Expect(malformed_v2_stall_timeout.parse_failure,
               "CircleV2 stall timeout below one should set parse_failure");
        Expect(malformed_v2_stall_timeout.bev_element
                   .circle_v2_inner_trace_stall_timeout_ms ==
                   builtin_defaults.bev_element.circle_v2_inner_trace_stall_timeout_ms,
               "CircleV2 stall timeout fallback should keep default timeout");
        Expect(malformed_v2_stall_timeout_diagnostics.SawCode("params.parse"),
               "CircleV2 stall timeout below one should emit params.parse");

        const std::string malformed_v2_path_offset_path =
            base + "_malformed_v2_path_offset.json";
        WriteText(malformed_v2_path_offset_path,
                  MinimalRuntimeParametersJson(
                      "  \"BEV_ELEMENT\": {\"CIRCLE_V2_INNER_TRACE_PATH_OFFSET_M\": -0.1}"));
        CaptureDiagnostics malformed_v2_path_offset_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_v2_path_offset =
            LoadFixture(malformed_v2_path_offset_path,
                        malformed_v2_path_offset_diagnostics);
        Expect(malformed_v2_path_offset.loaded_from_defaults,
               "negative CircleV2 inner path offset should fall back to defaults");
        Expect(malformed_v2_path_offset.parse_failure,
               "negative CircleV2 inner path offset should set parse_failure");
        Expect(std::abs(malformed_v2_path_offset.bev_element
                            .circle_v2_inner_trace_path_offset_m -
                        builtin_defaults.bev_element.circle_v2_inner_trace_path_offset_m) < 1.0e-6F,
               "CircleV2 inner path offset fallback should keep default");
        Expect(malformed_v2_path_offset_diagnostics.SawCode("params.parse"),
               "negative CircleV2 inner path offset should emit params.parse");

        const std::string malformed_v2_opposite_confidence_path =
            base + "_malformed_v2_opposite_confidence.json";
        WriteText(malformed_v2_opposite_confidence_path,
                  MinimalRuntimeParametersJson(
                      "  \"BEV_ELEMENT\": {"
                      "\"CIRCLE_V2_OPPOSITE_STRAIGHT_CONFIDENCE_MIN\": 1.2}"));
        CaptureDiagnostics malformed_v2_opposite_confidence_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_v2_opposite_confidence =
            LoadFixture(malformed_v2_opposite_confidence_path,
                        malformed_v2_opposite_confidence_diagnostics);
        Expect(malformed_v2_opposite_confidence.loaded_from_defaults,
               "CircleV2 opposite-straight confidence above one should fall back to defaults");
        Expect(malformed_v2_opposite_confidence.parse_failure,
               "CircleV2 opposite-straight confidence above one should set parse_failure");
        Expect(std::abs(malformed_v2_opposite_confidence.bev_element
                            .circle_v2_opposite_straight_confidence_min -
                        builtin_defaults.bev_element.circle_v2_opposite_straight_confidence_min) < 1.0e-6F,
               "CircleV2 opposite-straight confidence fallback should keep default");
        Expect(malformed_v2_opposite_confidence_diagnostics.SawCode("params.parse"),
               "CircleV2 opposite-straight confidence above one should emit params.parse");

        const std::string malformed_v2_entry_forward_path =
            base + "_malformed_v2_entry_forward.json";
        WriteText(malformed_v2_entry_forward_path,
                  MinimalRuntimeParametersJson(
                      "  \"BEV_ELEMENT\": {"
                      "\"CIRCLE_V2_ENTRY_BOTTOM_FORWARD_MIN_M\": 0.9,"
                      "\"CIRCLE_V2_ENTRY_BOTTOM_FORWARD_MAX_M\": 0.8}"));
        CaptureDiagnostics malformed_v2_entry_forward_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_v2_entry_forward =
            LoadFixture(malformed_v2_entry_forward_path,
                        malformed_v2_entry_forward_diagnostics);
        Expect(malformed_v2_entry_forward.loaded_from_defaults,
               "CircleV2 entry bottom inverted interval should fall back to defaults");
        Expect(malformed_v2_entry_forward.parse_failure,
               "CircleV2 entry bottom inverted interval should set parse_failure");
        Expect(std::abs(malformed_v2_entry_forward.bev_element
                            .circle_v2_entry_bottom_forward_min_m -
                        builtin_defaults.bev_element.circle_v2_entry_bottom_forward_min_m) < 1.0e-6F,
               "CircleV2 entry bottom min fallback should keep default");
        Expect(std::abs(malformed_v2_entry_forward.bev_element
                            .circle_v2_entry_bottom_forward_max_m -
                        builtin_defaults.bev_element.circle_v2_entry_bottom_forward_max_m) < 1.0e-6F,
               "CircleV2 entry bottom max fallback should keep default");
        Expect(malformed_v2_entry_forward_diagnostics.SawCode("params.parse"),
               "CircleV2 entry bottom inverted interval should emit params.parse");

        const std::string malformed_v2_entry_rows_path =
            base + "_malformed_v2_entry_rows.json";
        WriteText(malformed_v2_entry_rows_path,
                  MinimalRuntimeParametersJson(
                      "  \"BEV_ELEMENT\": {\"CIRCLE_V2_ENTRY_BOTTOM_MIN_ROW_COUNT\": 0}"));
        CaptureDiagnostics malformed_v2_entry_rows_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_v2_entry_rows =
            LoadFixture(malformed_v2_entry_rows_path,
                        malformed_v2_entry_rows_diagnostics);
        Expect(malformed_v2_entry_rows.loaded_from_defaults,
               "CircleV2 entry bottom min row count below one should fall back to defaults");
        Expect(malformed_v2_entry_rows.parse_failure,
               "CircleV2 entry bottom min row count below one should set parse_failure");
        Expect(malformed_v2_entry_rows.bev_element.circle_v2_entry_bottom_min_row_count ==
                   builtin_defaults.bev_element.circle_v2_entry_bottom_min_row_count,
               "CircleV2 entry bottom min-row-count fallback should keep default");
        Expect(malformed_v2_entry_rows_diagnostics.SawCode("params.parse"),
               "CircleV2 entry bottom min row count below one should emit params.parse");

        const std::string malformed_boundary_path = base + "_malformed_boundary.json";
        WriteText(malformed_boundary_path,
                  MinimalRuntimeParametersJson(
                      "  \"BEV_BOUNDARY\": {\"LOCAL_JUMP_MIN_Y\": 0}"));
        CaptureDiagnostics malformed_boundary_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_boundary =
            LoadFixture(malformed_boundary_path, malformed_boundary_diagnostics);
        Expect(malformed_boundary.loaded_from_defaults,
               "out-of-range local jump should fall back to defaults");
        Expect(malformed_boundary.parse_failure,
               "out-of-range local jump should set parse_failure");
        Expect(malformed_boundary.bev_boundary.local_jump_min_y ==
                   builtin_defaults.bev_boundary.local_jump_min_y,
               "boundary fallback should keep default local jump");
        Expect(malformed_boundary_diagnostics.SawCode("params.parse"),
               "out-of-range local jump should emit params.parse");

        const std::string malformed_downsample_path = base + "_malformed_downsample.json";
        WriteText(malformed_downsample_path,
                  MinimalRuntimeParametersJson("  \"steering_media_downsample\": 0"));
        CaptureDiagnostics malformed_downsample_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_downsample =
            LoadFixture(malformed_downsample_path, malformed_downsample_diagnostics);
        Expect(malformed_downsample.loaded_from_defaults,
               "out-of-range steering_media_downsample should fall back to defaults");
        Expect(malformed_downsample.parse_failure,
               "out-of-range steering_media_downsample should set parse_failure");
        Expect(malformed_downsample.steering_media_downsample == 1,
               "downsample fallback should keep default downsample");
        Expect(malformed_downsample_diagnostics.SawCode("params.parse"),
               "out-of-range steering_media_downsample should emit params.parse");

        const std::string malformed_brushless_pwm_path = base + "_malformed_brushless_pwm.json";
        WriteText(malformed_brushless_pwm_path,
                  MinimalRuntimeParametersJson("  \"brushless_debug_fixed_pwm\": 1200"));
        CaptureDiagnostics malformed_brushless_pwm_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_brushless_pwm =
            LoadFixture(malformed_brushless_pwm_path, malformed_brushless_pwm_diagnostics);
        Expect(malformed_brushless_pwm.loaded_from_defaults,
               "out-of-range brushless_debug_fixed_pwm should fall back to defaults");
        Expect(malformed_brushless_pwm.parse_failure,
               "out-of-range brushless_debug_fixed_pwm should set parse_failure");
        Expect(malformed_brushless_pwm.brushless_debug_fixed_pwm ==
                   builtin_defaults.brushless_debug_fixed_pwm,
               "brushless PWM fallback should keep default");
        Expect(malformed_brushless_pwm_diagnostics.SawCode("params.parse"),
               "out-of-range brushless_debug_fixed_pwm should emit params.parse");

        const std::string malformed_gray_bits_path = base + "_malformed_gray_bits.json";
        WriteText(malformed_gray_bits_path,
                  MinimalRuntimeParametersJson("  \"steering_media_gray_bits\": 5"));
        CaptureDiagnostics malformed_gray_bits_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_gray_bits =
            LoadFixture(malformed_gray_bits_path, malformed_gray_bits_diagnostics);
        Expect(malformed_gray_bits.loaded_from_defaults,
               "out-of-range steering_media_gray_bits should fall back to defaults");
        Expect(malformed_gray_bits.parse_failure,
               "out-of-range steering_media_gray_bits should set parse_failure");
        Expect(malformed_gray_bits.steering_media_gray_bits == 2,
               "gray_bits fallback should keep default gray2");
        Expect(malformed_gray_bits_diagnostics.SawCode("params.parse"),
               "out-of-range steering_media_gray_bits should emit params.parse");

        const std::string malformed_accel_scale_path = base + "_malformed_accel_scale.json";
        WriteText(malformed_accel_scale_path,
                  MinimalRuntimeParametersJson("  \"wheel_turn_accel_delta_scale\": -0.01"));
        CaptureDiagnostics malformed_accel_scale_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_accel_scale =
            LoadFixture(malformed_accel_scale_path, malformed_accel_scale_diagnostics);
        Expect(malformed_accel_scale.loaded_from_defaults,
               "negative wheel_turn_accel_delta_scale should fall back to defaults");
        Expect(malformed_accel_scale.parse_failure,
               "negative wheel_turn_accel_delta_scale should set parse_failure");
        Expect(std::abs(malformed_accel_scale.wheel_turn_accel_delta_scale -
                        builtin_defaults.wheel_turn_accel_delta_scale) < 1.0e-6,
               "accel scale fallback should keep default");
        Expect(malformed_accel_scale_diagnostics.SawCode("params.parse"),
               "negative wheel_turn_accel_delta_scale should emit params.parse");

        const std::string malformed_decel_scale_path = base + "_malformed_decel_scale.json";
        WriteText(malformed_decel_scale_path,
                  MinimalRuntimeParametersJson("  \"wheel_turn_decel_delta_scale\": \"bad\""));
        CaptureDiagnostics malformed_decel_scale_diagnostics{};
        const ls2k::port::RuntimeParameters malformed_decel_scale =
            LoadFixture(malformed_decel_scale_path, malformed_decel_scale_diagnostics);
        Expect(malformed_decel_scale.loaded_from_defaults,
               "malformed wheel_turn_decel_delta_scale should fall back to defaults");
        Expect(malformed_decel_scale.parse_failure,
               "malformed wheel_turn_decel_delta_scale should set parse_failure");
        Expect(std::abs(malformed_decel_scale.wheel_turn_decel_delta_scale - 1.0) < 1.0e-6,
               "decel scale fallback should keep default");
        Expect(malformed_decel_scale_diagnostics.SawCode("params.parse"),
               "malformed wheel_turn_decel_delta_scale should emit params.parse");

        const std::string unsupported_camera_path = base + "_unsupported_camera.json";
        WriteText(unsupported_camera_path,
                  MinimalRuntimeParametersJson(
                      "  \"CAMERA_SOURCE\": {\"BACKEND\": \"unsupported_backend\"}"));
        CaptureDiagnostics unsupported_camera_diagnostics{};
        const ls2k::port::RuntimeParameters unsupported_camera =
            LoadFixture(unsupported_camera_path, unsupported_camera_diagnostics);
        Expect(unsupported_camera.loaded_from_defaults && unsupported_camera.parse_failure,
               "unsupported camera backend should fail validation instead of aliasing a source");
        Expect(unsupported_camera_diagnostics.SawCode("params.parse"),
               "unsupported camera backend should emit params.parse");

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
