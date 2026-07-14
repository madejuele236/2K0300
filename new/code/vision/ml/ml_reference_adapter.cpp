#include "vision/ml/ml_reference_adapter.hpp"

#include <cmath>

namespace ls2k::vision::ml {

port::MlLockedManeuver LockObservedBoundaryToMarker(
    port::MlAction action,
    uint64_t lock_time_ms,
    const port::MlOrientedRectangle& rectangle,
    const port::BEVReferencePath& observed_boundary) {
    port::MlLockedManeuver out{};
    if (!rectangle.valid || observed_boundary.mode != port::ReferenceMode::kMlObservedBoundary ||
        (action != port::MlAction::kLeft && action != port::MlAction::kRight)) {
        return out;
    }
    // The short axis is the left-hand normal of the reported long axis. Its
    // sign is fixed once, at lock, so marker +forward always points vehicle-forward.
    float axis_f = -rectangle.long_axis_lateral;
    float axis_l = rectangle.long_axis_forward;
    if (axis_f < 0.0F) {
        axis_f = -axis_f;
        axis_l = -axis_l;
    }
    const float norm = std::hypot(axis_f, axis_l);
    if (!std::isfinite(norm) || norm < 1.0e-6F) {
        return out;
    }
    axis_f /= norm;
    axis_l /= norm;
    out.action = action;
    out.lock_time_ms = lock_time_ms;
    out.anchor = rectangle.center;
    out.marker_forward_axis_forward = axis_f;
    out.marker_forward_axis_lateral = axis_l;
    for (const port::BEVPathSample& sample : observed_boundary.sampled_path) {
        if (!sample.present) break;
        port::BEVPathSample& marker = out.marker_path.samples[out.marker_path.sample_count++];
        const float df = sample.point.forward_m - out.anchor.forward_m;
        const float dl = sample.point.lateral_m - out.anchor.lateral_m;
        marker = sample;
        marker.point.forward_m = df * axis_f + dl * axis_l;
        marker.point.lateral_m = -df * axis_l + dl * axis_f;
    }
    out.marker_path.valid = out.marker_path.sample_count > 0U;
    out.valid = out.marker_path.valid;
    return out;
}

port::BEVReferencePath TransformMarkerPathToCurrentVehicle(
    const port::MlLockedManeuver& locked,
    const port::MlPoseDelta& pose_delta) {
    port::BEVReferencePath out{};
    if (!locked.valid || !pose_delta.valid) return out;
    const float c = std::cos(pose_delta.yaw_rad);
    const float s = std::sin(pose_delta.yaw_rad);
    out.mode = port::ReferenceMode::kMlObservedBoundary;
    std::size_t count = 0U;
    for (const port::BEVPathSample& marker : locked.marker_path.samples) {
        if (!marker.present) break;
        const float initial_f = locked.anchor.forward_m +
            marker.point.forward_m * locked.marker_forward_axis_forward -
            marker.point.lateral_m * locked.marker_forward_axis_lateral;
        const float initial_l = locked.anchor.lateral_m +
            marker.point.forward_m * locked.marker_forward_axis_lateral +
            marker.point.lateral_m * locked.marker_forward_axis_forward;
        const float relative_f = initial_f - pose_delta.forward_m;
        const float relative_l = initial_l - pose_delta.lateral_m;
        port::BEVPathSample& current = out.sampled_path[count++];
        current = marker;
        current.point.forward_m = c * relative_f + s * relative_l;
        current.point.lateral_m = -s * relative_f + c * relative_l;
        current.source = port::BEVPathPointSource::kMlObservedBoundary;
    }
    return out;
}

}  // namespace ls2k::vision::ml
