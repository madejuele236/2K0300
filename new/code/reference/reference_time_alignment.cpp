#include "reference/reference_time_alignment.hpp"

#include <algorithm>
#include <cmath>

#include "port/perf_counter.hpp"
#include "port/bev_reference_path_utils.hpp"

namespace ls2k::reference {
namespace {

port::BEVReferencePath TransformReferenceSamplesSE2(const port::BEVReferencePath& input,
                                                    double delta_forward_m,
                                                    double delta_lateral_m,
                                                    double delta_yaw_rad,
                                                    std::size_t& aligned_count) {
    port::BEVReferencePath output{};
    output.mode = input.mode;
    const double c = std::cos(-delta_yaw_rad);
    const double s = std::sin(-delta_yaw_rad);
    std::size_t out_index = 0;
    for (const port::BEVPathSample& sample : input.sampled_path) {
        if (!port::IsFiniteReferenceSample(sample)) {
            continue;
        }
        const double x = static_cast<double>(sample.point.forward_m) - delta_forward_m;
        const double y = static_cast<double>(sample.point.lateral_m) - delta_lateral_m;
        const double forward = c * x - s * y;
        const double lateral = s * x + c * y;
        if (!std::isfinite(forward) || !std::isfinite(lateral) || forward <= 0.0) {
            continue;
        }
        if (out_index >= output.sampled_path.size()) {
            break;
        }
        port::BEVPathSample& out = output.sampled_path[out_index++];
        out = sample;
        out.point.forward_m = static_cast<float>(forward);
        out.point.lateral_m = static_cast<float>(lateral);
    }
    aligned_count = out_index;
    if (aligned_count == 0U) {
        output.mode = port::ReferenceMode::kNone;
    }
    return output;
}

}  // namespace

ReferenceTimeAlignmentResult AlignReferencePathToVehiclePoseDelta(
    const port::BEVReferencePath& reference_path,
    uint64_t reference_capture_time_ms,
    uint64_t control_time_ms,
    uint64_t control_effective_time_ms,
    const port::VehiclePoseDelta& pose_delta,
    const port::RuntimeParameters& params) {
    LS2K_PERF_SCOPE(port::PerfStage::kReferenceTimeAlignment);
    ReferenceTimeAlignmentResult result{};
    result.reference_path = reference_path;
    port::ReferenceTimeAlignmentFacts& facts = result.facts;
    facts.enabled = params.reference_time_alignment.enabled;
    facts.reference_capture_time_ms = reference_capture_time_ms;
    facts.control_time_ms = control_time_ms;
    facts.control_effective_time_ms = control_effective_time_ms;
    facts.input_sample_count = port::CountFiniteReferenceSamples(reference_path);
    if (!params.reference_time_alignment.enabled) {
        facts.valid = true;
        facts.reason = "disabled";
        facts.aligned_sample_count = facts.input_sample_count;
        return result;
    }
    if (reference_capture_time_ms == 0 ||
        control_time_ms < reference_capture_time_ms ||
        control_effective_time_ms < reference_capture_time_ms) {
        facts.reason = "invalid_reference_time";
        result.reference_path = {};
        return result;
    }
    facts.age_ms = control_effective_time_ms - reference_capture_time_ms;
    if (facts.age_ms > static_cast<uint64_t>(std::max(1, params.reference_time_alignment.max_age_ms))) {
        facts.reason = "age_exceeded";
        result.reference_path = {};
        return result;
    }

    if (!pose_delta.valid) {
        facts.reason = "pose_delta_" + pose_delta.reason;
        result.reference_path = {};
        return result;
    }

    facts.measured_until_ms = pose_delta.measured_until_ms;
    facts.predicted_ms = pose_delta.predicted_ms;
    facts.delta_forward_m = pose_delta.delta_forward_m;
    facts.delta_lateral_m = pose_delta.delta_lateral_m;
    facts.delta_yaw_rad = pose_delta.delta_yaw_rad;
    facts.measured_forward_mps = pose_delta.measured_forward_mps;
    facts.measured_yaw_rate_radps = pose_delta.measured_yaw_rate_radps;
    facts.predicted_forward_mps = pose_delta.predicted_forward_mps;
    facts.predicted_yaw_rate_radps = pose_delta.predicted_yaw_rate_radps;
    facts.used_encoder_forward = pose_delta.used_encoder_forward;
    facts.used_imu_yaw = pose_delta.used_imu_yaw;
    facts.used_wheel_yaw = pose_delta.used_wheel_yaw;
    facts.used_command_prediction = pose_delta.used_command_prediction;

    if (std::fabs(facts.delta_forward_m) > params.reference_time_alignment.max_delta_forward_m) {
        facts.reason = "delta_forward_exceeded";
        result.reference_path = {};
        return result;
    }
    if (std::fabs(facts.delta_lateral_m) > params.reference_time_alignment.max_delta_lateral_m) {
        facts.reason = "delta_lateral_exceeded";
        result.reference_path = {};
        return result;
    }
    if (std::fabs(facts.delta_yaw_rad) > params.reference_time_alignment.max_delta_yaw_rad) {
        facts.reason = "delta_yaw_exceeded";
        result.reference_path = {};
        return result;
    }

    result.reference_path = TransformReferenceSamplesSE2(reference_path,
                                                         facts.delta_forward_m,
                                                         facts.delta_lateral_m,
                                                         facts.delta_yaw_rad,
                                                         facts.aligned_sample_count);
    if (facts.aligned_sample_count <
        static_cast<std::size_t>(std::max(1, params.reference_time_alignment.min_aligned_samples))) {
        facts.reason = "aligned_samples_insufficient";
        result.reference_path = {};
        return result;
    }
    facts.valid = true;
    facts.reason = facts.predicted_ms > 0 ? "aligned_effective_se2" : "aligned_measured_se2";
    return result;
}

}  // namespace ls2k::reference
