#ifndef LS2K_PORT_RUNTIME_PARAMETER_VALIDATION_HPP
#define LS2K_PORT_RUNTIME_PARAMETER_VALIDATION_HPP

#include <cmath>
#include <string>

#include "runtime_parameter_types.hpp"

namespace ls2k::port {

inline bool FiniteInRange(double value, double min_value, double max_value) {
    return std::isfinite(value) && value >= min_value && value <= max_value;
}

inline bool ValidateReferenceTimeAlignmentParameters(
    const ReferenceTimeAlignmentParameters& params,
    const MotionOdometryParameters& odometry) {
    if (params.max_age_ms < 1 ||
        params.effective_delay_ms < 0 ||
        params.future_prediction_max_ms < 0 ||
        params.max_integration_gap_ms < 1 ||
        params.min_aligned_samples < 1 ||
        !FiniteInRange(odometry.encoder_ticks_to_meter, 0.0, 1.0) ||
        !FiniteInRange(params.wheel_track_m, 0.0, 2.0) ||
        !FiniteInRange(params.turn_output_to_yaw_rate_gain, -100.0, 100.0) ||
        !FiniteInRange(params.actuator_yaw_tau_ms, 0.0, 1000.0) ||
        !FiniteInRange(params.max_delta_forward_m, 0.0, 2.0) ||
        !FiniteInRange(params.max_delta_lateral_m, 0.0, 2.0) ||
        !FiniteInRange(params.max_delta_yaw_rad, 0.0, 6.28319)) {
        return false;
    }
    if (!params.enabled) {
        return true;
    }
    if (params.use_encoder_forward && odometry.encoder_ticks_to_meter <= 0.0) {
        return false;
    }
    if (params.use_wheel_yaw_fallback &&
        (odometry.encoder_ticks_to_meter <= 0.0 || params.wheel_track_m <= 0.0)) {
        return false;
    }
    if (params.command_yaw_prediction_enabled &&
        (!params.future_prediction_enabled || params.turn_output_to_yaw_rate_gain == 0.0)) {
        return false;
    }
    return true;
}

inline bool IsMlActionToken(const std::string& action) {
    return action == "straight" || action == "left" || action == "right" ||
           action == "unmapped";
}

inline bool ValidateMlParameters(const MlParameters& params,
                                 const MotionOdometryParameters& odometry) {
    constexpr int kTfliteIdentitySquaredL2Max = 4 * 255 * 255;
    if (!IsMlActionToken(params.class_mapping.class_0_action) ||
        !IsMlActionToken(params.class_mapping.class_1_action) ||
        !IsMlActionToken(params.class_mapping.class_2_action) ||
        params.v9.min_margin < 0 || params.v9.max_best_distance < 0 ||
        params.v9.max_best_distance > 126 || params.v9.confirm_frames < 1 ||
        params.tflite_identity.min_margin < 0 ||
        params.tflite_identity.min_margin > kTfliteIdentitySquaredL2Max ||
        params.tflite_identity.max_best_distance < 0 ||
        params.tflite_identity.max_best_distance > kTfliteIdentitySquaredL2Max ||
        params.tflite_identity.confirm_frames < 1) {
        return false;
    }
    if (!params.enabled) {
        return !params.maneuver.enabled;
    }

    const MlRoiParameters& roi = params.roi;
    const MlManeuverParameters& maneuver = params.maneuver;
    const double score_total = roi.score_size_weight + roi.score_rectangularity_weight +
                               roi.score_red_fill_weight + roi.score_orientation_weight;
    const bool yuv_valid = roi.red_y_min >= 0 && roi.red_y_min <= roi.red_y_max &&
                           roi.red_y_max <= 255 && roi.red_u_min >= 0 &&
                           roi.red_u_min <= roi.red_u_max && roi.red_u_max <= 255 &&
                           roi.red_v_min >= 0 && roi.red_v_min <= roi.red_v_max &&
                           roi.red_v_max <= 255;
    const bool scores_valid =
        FiniteInRange(roi.score_size_weight, 0.0, 1.0e9) &&
        FiniteInRange(roi.score_rectangularity_weight, 0.0, 1.0e9) &&
        FiniteInRange(roi.score_red_fill_weight, 0.0, 1.0e9) &&
        FiniteInRange(roi.score_orientation_weight, 0.0, 1.0e9) &&
        std::isfinite(score_total) && score_total > 0.0;
    const bool observation_valid = std::isfinite(roi.search_forward_min_m) &&
           std::isfinite(roi.search_forward_max_m) &&
           roi.search_forward_max_m > roi.search_forward_min_m &&
           std::isfinite(roi.search_lateral_limit_m) && roi.search_lateral_limit_m > 0.0 &&
           std::isfinite(roi.grid_forward_step_m) && roi.grid_forward_step_m > 0.0 &&
           std::isfinite(roi.grid_lateral_step_m) && roi.grid_lateral_step_m > 0.0 && yuv_valid &&
           std::isfinite(roi.expected_long_edge_m) && roi.expected_long_edge_m > 0.0 &&
           std::isfinite(roi.expected_short_edge_m) && roi.expected_short_edge_m > 0.0 &&
           roi.expected_long_edge_m >= roi.expected_short_edge_m &&
           std::isfinite(roi.crop_long_offset_m) &&
           std::fabs(roi.crop_long_offset_m) <= roi.expected_long_edge_m &&
           std::isfinite(roi.crop_forward_offset_m) &&
           std::fabs(roi.crop_forward_offset_m) <= roi.expected_long_edge_m &&
           std::isfinite(roi.long_edge_tolerance_m) && roi.long_edge_tolerance_m > 0.0 &&
           std::isfinite(roi.short_edge_tolerance_m) && roi.short_edge_tolerance_m > 0.0 &&
           FiniteInRange(roi.max_long_edge_to_lateral_rad, 1.0e-12, 1.5707963267948966) &&
           roi.min_component_cells >= 1 &&
           FiniteInRange(roi.min_rectangularity, 1.0e-12, 1.0) &&
           FiniteInRange(roi.min_red_fill_ratio, 1.0e-12, 1.0) && scores_valid;
    if (!observation_valid || !params.maneuver.enabled) {
        return observation_valid;
    }
    return std::isfinite(odometry.encoder_ticks_to_meter) &&
           odometry.encoder_ticks_to_meter > 0.0 &&
           std::isfinite(maneuver.speed_target) && maneuver.speed_target > 0.0 &&
           maneuver.min_boundary_samples >= 3 &&
           FiniteInRange(maneuver.path_outward_offset_m, 0.0, 2.0) &&
           std::isfinite(maneuver.exit_forward_m) &&
           maneuver.exit_forward_m > 0.0 &&
           maneuver.max_duration_ms >= 1 && maneuver.max_integration_gap_ms >= 1 &&
           maneuver.cooldown_ms >= 0;
}

}  // namespace ls2k::port

#endif  // LS2K_PORT_RUNTIME_PARAMETER_VALIDATION_HPP
