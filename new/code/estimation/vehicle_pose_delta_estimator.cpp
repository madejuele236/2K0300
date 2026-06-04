#include "estimation/vehicle_pose_delta_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ls2k::estimation {
namespace {

void Fail(port::VehiclePoseDelta& out, const char* reason) {
    out.valid = false;
    out.reason = reason;
}

void IntegrateBodyStep(port::VehiclePoseDelta& out, double ds_m, double dyaw_rad) {
    const double theta_mid = out.delta_yaw_rad + 0.5 * dyaw_rad;
    out.delta_forward_m += std::cos(theta_mid) * ds_m;
    out.delta_lateral_m += std::sin(theta_mid) * ds_m;
    out.delta_yaw_rad += dyaw_rad;
}

bool LatestUsableCommand(const port::ControlCommandHistory& history,
                         uint64_t now_ms,
                         port::ControlCommandHistorySample& out) {
    for (std::size_t offset = 0; offset < history.count; ++offset) {
        const port::ControlCommandHistorySample& sample = history.NewestOffset(offset);
        if (sample.time_ms > now_ms) {
            continue;
        }
        if (!sample.valid || !sample.actuator_applied || sample.diagnostics_only ||
            sample.hold_disarmed || sample.emergency_stop) {
            continue;
        }
        out = sample;
        return true;
    }
    return false;
}

bool IntegrateMeasuredWindow(uint64_t start_ms,
                             uint64_t end_ms,
                             const port::MotionHistory& history,
                             const port::ReferenceTimeAlignmentParameters& params,
                             port::VehiclePoseDelta& out) {
    if (end_ms <= start_ms) {
        out.measured_until_ms = start_ms;
        return true;
    }
    if (history.count < 2) {
        Fail(out, "motion_history_unavailable");
        return false;
    }
    if (params.use_encoder_forward && params.encoder_ticks_to_meter <= 0.0) {
        Fail(out, "encoder_scale_unavailable");
        return false;
    }

    uint64_t covered_until_ms = start_ms;
    const uint64_t max_gap_ms = static_cast<uint64_t>(std::max(1, params.max_integration_gap_ms));
    double measured_forward_m = 0.0;
    double measured_yaw_rad = 0.0;
    uint64_t measured_dt_ms = 0;

    for (std::size_t index = 1; index < history.count; ++index) {
        const port::MotionHistorySample& prev = history.OldestOffset(index - 1U);
        const port::MotionHistorySample& curr = history.OldestOffset(index);
        if (curr.time_ms <= covered_until_ms) {
            continue;
        }
        if (prev.time_ms > covered_until_ms || prev.time_ms >= end_ms) {
            break;
        }
        if (curr.time_ms <= prev.time_ms) {
            Fail(out, "motion_history_non_monotonic");
            return false;
        }

        const uint64_t full_gap_ms = curr.time_ms - prev.time_ms;
        if (full_gap_ms > max_gap_ms) {
            Fail(out, "motion_history_unavailable");
            return false;
        }

        const uint64_t segment_start_ms = std::max(covered_until_ms, prev.time_ms);
        const uint64_t segment_end_ms = std::min(end_ms, curr.time_ms);
        if (segment_end_ms <= segment_start_ms) {
            continue;
        }

        const double fraction =
            static_cast<double>(segment_end_ms - segment_start_ms) /
            static_cast<double>(full_gap_ms);
        const double dt_s = static_cast<double>(segment_end_ms - segment_start_ms) / 1000.0;

        double ds_m = 0.0;
        if (params.use_encoder_forward) {
            if (!curr.encoder_valid) {
                Fail(out, "encoder_history_unavailable");
                return false;
            }
            const double mean_ticks =
                0.5 * (static_cast<double>(curr.left_encoder_delta) +
                       static_cast<double>(curr.right_encoder_delta));
            ds_m = mean_ticks * params.encoder_ticks_to_meter * fraction;
            out.used_encoder_forward = true;
        }

        double dyaw_rad = 0.0;
        bool yaw_available = false;
        if (params.use_imu_yaw && prev.imu_valid && curr.imu_valid) {
            dyaw_rad = static_cast<double>(prev.gyro_z) * dt_s;
            yaw_available = true;
            out.used_imu_yaw = true;
        } else if (params.use_wheel_yaw_fallback &&
                   curr.encoder_valid &&
                   params.encoder_ticks_to_meter > 0.0 &&
                   params.wheel_track_m > 1.0e-6) {
            const double right_m =
                static_cast<double>(curr.right_encoder_delta) *
                params.encoder_ticks_to_meter *
                fraction;
            const double left_m =
                static_cast<double>(curr.left_encoder_delta) *
                params.encoder_ticks_to_meter *
                fraction;
            dyaw_rad = (right_m - left_m) / params.wheel_track_m;
            yaw_available = true;
            out.used_wheel_yaw = true;
        }
        if (!yaw_available) {
            Fail(out, "yaw_history_unavailable");
            return false;
        }

        IntegrateBodyStep(out, ds_m, dyaw_rad);
        measured_forward_m += ds_m;
        measured_yaw_rad += dyaw_rad;
        measured_dt_ms += segment_end_ms - segment_start_ms;
        covered_until_ms = segment_end_ms;
        ++out.integrated_motion_segments;

        if (covered_until_ms >= end_ms) {
            out.measured_until_ms = end_ms;
            if (measured_dt_ms > 0) {
                const double measured_dt_s = static_cast<double>(measured_dt_ms) / 1000.0;
                out.measured_forward_mps = measured_forward_m / measured_dt_s;
                out.measured_yaw_rate_radps = measured_yaw_rad / measured_dt_s;
            }
            return true;
        }
    }

    Fail(out, "motion_history_unavailable");
    return false;
}

double FirstOrderAverage(double current, double target, double dt_s, double tau_s) {
    if (dt_s <= 0.0) {
        return current;
    }
    if (tau_s <= 1.0e-6) {
        return target;
    }
    const double ratio = tau_s / dt_s;
    const double decay = std::exp(-dt_s / tau_s);
    return target + (current - target) * ratio * (1.0 - decay);
}

bool PredictFutureWindow(uint64_t now_ms,
                         uint64_t end_ms,
                         const port::ControlCommandHistory& command_history,
                         const port::ReferenceTimeAlignmentParameters& params,
                         port::VehiclePoseDelta& out) {
    if (end_ms <= now_ms) {
        return true;
    }
    const uint64_t future_ms = end_ms - now_ms;
    if (!params.future_prediction_enabled) {
        Fail(out, "future_prediction_disabled");
        return false;
    }
    if (future_ms > static_cast<uint64_t>(std::max(0, params.future_prediction_max_ms))) {
        Fail(out, "future_prediction_horizon_exceeded");
        return false;
    }

    const double dt_s = static_cast<double>(future_ms) / 1000.0;
    double future_forward_mps = out.measured_forward_mps;
    double future_yaw_rate_radps = out.measured_yaw_rate_radps;

    port::ControlCommandHistorySample command{};
    if (params.command_yaw_prediction_enabled &&
        LatestUsableCommand(command_history, now_ms, command)) {
        const double target_yaw_rate =
            static_cast<double>(command.applied_turn_output) *
            params.turn_output_to_yaw_rate_gain;
        future_yaw_rate_radps = FirstOrderAverage(out.measured_yaw_rate_radps,
                                                  target_yaw_rate,
                                                  dt_s,
                                                  params.actuator_yaw_tau_ms / 1000.0);
        out.used_command_prediction = true;
    }

    out.predicted_ms = future_ms;
    out.predicted_forward_mps = future_forward_mps;
    out.predicted_yaw_rate_radps = future_yaw_rate_radps;
    IntegrateBodyStep(out, future_forward_mps * dt_s, future_yaw_rate_radps * dt_s);
    return true;
}

}  // namespace

port::VehiclePoseDelta EstimateVehiclePoseDelta(
    uint64_t start_time_ms,
    uint64_t now_time_ms,
    uint64_t end_time_ms,
    const port::MotionHistory& motion_history,
    const port::ControlCommandHistory& command_history,
    const port::ReferenceTimeAlignmentParameters& params) {
    port::VehiclePoseDelta out{};
    out.start_time_ms = start_time_ms;
    out.now_time_ms = now_time_ms;
    out.end_time_ms = end_time_ms;
    out.measured_until_ms = start_time_ms;

    if (start_time_ms == 0 || now_time_ms < start_time_ms || end_time_ms < start_time_ms) {
        Fail(out, "invalid_pose_delta_time");
        return out;
    }

    const uint64_t measured_end_ms = std::min(now_time_ms, end_time_ms);
    if (!IntegrateMeasuredWindow(start_time_ms, measured_end_ms, motion_history, params, out)) {
        return out;
    }
    if (!PredictFutureWindow(now_time_ms, end_time_ms, command_history, params, out)) {
        return out;
    }

    if (!std::isfinite(out.delta_forward_m) ||
        !std::isfinite(out.delta_lateral_m) ||
        !std::isfinite(out.delta_yaw_rad)) {
        Fail(out, "pose_delta_nonfinite");
        return out;
    }
    if (std::fabs(out.delta_forward_m) > params.max_delta_forward_m) {
        Fail(out, "delta_forward_exceeded");
        return out;
    }
    if (std::fabs(out.delta_lateral_m) > params.max_delta_lateral_m) {
        Fail(out, "delta_lateral_exceeded");
        return out;
    }
    if (std::fabs(out.delta_yaw_rad) > params.max_delta_yaw_rad) {
        Fail(out, "delta_yaw_exceeded");
        return out;
    }

    out.valid = true;
    out.reason = out.predicted_ms > 0 ? "estimated_with_prediction" : "estimated_measured_only";
    return out;
}

}  // namespace ls2k::estimation
