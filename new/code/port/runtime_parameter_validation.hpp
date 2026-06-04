#ifndef LS2K_PORT_RUNTIME_PARAMETER_VALIDATION_HPP
#define LS2K_PORT_RUNTIME_PARAMETER_VALIDATION_HPP

#include <cmath>

#include "runtime_parameter_types.hpp"

namespace ls2k::port {

inline bool FiniteInRange(double value, double min_value, double max_value) {
    return std::isfinite(value) && value >= min_value && value <= max_value;
}

inline bool ValidateReferenceTimeAlignmentParameters(
    const ReferenceTimeAlignmentParameters& params) {
    if (params.max_age_ms < 1 ||
        params.effective_delay_ms < 0 ||
        params.future_prediction_max_ms < 0 ||
        params.max_integration_gap_ms < 1 ||
        params.min_aligned_samples < 1 ||
        !FiniteInRange(params.encoder_ticks_to_meter, 0.0, 1.0) ||
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
    if (params.use_encoder_forward && params.encoder_ticks_to_meter <= 0.0) {
        return false;
    }
    if (params.use_wheel_yaw_fallback &&
        (params.encoder_ticks_to_meter <= 0.0 || params.wheel_track_m <= 0.0)) {
        return false;
    }
    if (params.command_yaw_prediction_enabled &&
        (!params.future_prediction_enabled || params.turn_output_to_yaw_rate_gain == 0.0)) {
        return false;
    }
    return true;
}

}  // namespace ls2k::port

#endif  // LS2K_PORT_RUNTIME_PARAMETER_VALIDATION_HPP
