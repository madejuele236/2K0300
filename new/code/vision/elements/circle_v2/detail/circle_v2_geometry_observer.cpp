#include "vision/elements/circle_v2/detail/circle_v2_internal.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "port/bev_reference_path_utils.hpp"

namespace ls2k::vision::detail {
namespace {

constexpr std::size_t kMinimumLinePointCount = 2U;

CircleDir Opposite(CircleDir dir) {
    if (dir == CircleDir::kLeft) {
        return CircleDir::kRight;
    }
    if (dir == CircleDir::kRight) {
        return CircleDir::kLeft;
    }
    return CircleDir::kNone;
}

const BEVWhiteRun* UniqueOriginConnectedRun(const BEVSimpleRowScan& row) {
    const BEVWhiteRun* selected = nullptr;
    for (const BEVWhiteRun& run : row.white_runs) {
        if (run.origin_connectivity != BEVWhiteRunOriginConnectivity::kConnected) {
            continue;
        }
        if (selected != nullptr) {
            return nullptr;
        }
        selected = &run;
    }
    return selected;
}

bool RealBoundaryForSide(const BEVWhiteRun& run,
                         CircleDir side,
                         float& lateral_m) {
    if (side == CircleDir::kLeft &&
        run.left_endpoint == BEVWhiteRunEndpointState::kBoundary) {
        lateral_m = run.left_m;
        return std::isfinite(lateral_m);
    }
    if (side == CircleDir::kRight &&
        run.right_endpoint == BEVWhiteRunEndpointState::kBoundary) {
        lateral_m = run.right_m;
        return std::isfinite(lateral_m);
    }
    return false;
}

BEVWhiteRunEndpointState EndpointForSide(const BEVWhiteRun& run, CircleDir side) {
    return side == CircleDir::kLeft ? run.left_endpoint : run.right_endpoint;
}

void AppendBoundaryPoint(port::BEVReferencePath& path,
                         std::size_t& point_count,
                         const port::BEVPoint& point,
                         float confidence) {
    if (point_count >= path.sampled_path.size()) {
        return;
    }
    port::BEVPathSample& sample = path.sampled_path[point_count++];
    sample.present = true;
    sample.point = point;
    sample.confidence = confidence;
    sample.source = port::BEVPathPointSource::kIntervalCenter;
}

std::size_t BuildLeadingRealBoundaryPath(const SceneFrameView& frame,
                                         CircleDir side,
                                         float forward_min_m,
                                         float forward_max_m,
                                         float max_adjacent_distance_m,
                                         port::BEVReferencePath& edge_path) {
    edge_path.mode = port::ReferenceMode::kIntervalCenter;
    std::size_t point_count = 0U;
    bool segment_started = false;
    float previous_forward_m = 0.0F;

    for (std::size_t row_index = 0; row_index < frame.rows.rows.size(); ++row_index) {
        const BEVSimpleRowScan& row = frame.rows.rows[row_index];
        if (row.forward_m < forward_min_m || row.forward_m > forward_max_m) {
            continue;
        }

        const BEVWhiteRun* run = row.valid ? UniqueOriginConnectedRun(row) : nullptr;
        float lateral_m = 0.0F;
        const bool observable = run != nullptr && RealBoundaryForSide(*run, side, lateral_m);
        const bool adjacent = !segment_started ||
                              row.forward_m - previous_forward_m <= max_adjacent_distance_m;
        if (!observable) {
            continue;
        }
        if (!adjacent) {
            break;
        }
        if (point_count >= edge_path.sampled_path.size()) {
            break;
        }

        port::BEVPathSample& sample = edge_path.sampled_path[point_count];
        sample.present = true;
        sample.point.forward_m = row.forward_m;
        sample.point.lateral_m = lateral_m;
        sample.confidence = 0.8F;
        sample.source = port::BEVPathPointSource::kIntervalCenter;
        previous_forward_m = row.forward_m;
        segment_started = true;
        ++point_count;
    }
    return point_count;
}

bool IsStraightEnough(const port::BEVReferencePath& edge_path,
                      std::size_t point_count,
                      float max_lateral_span_m) {
    float min_lateral = 0.0F;
    float max_lateral = 0.0F;
    std::size_t used_count = 0U;
    for (std::size_t index = 0; index < point_count; ++index) {
        const port::BEVPathSample& sample = edge_path.sampled_path[index];
        if (!port::IsFiniteReferenceSample(sample)) {
            continue;
        }
        if (used_count == 0U) {
            min_lateral = sample.point.lateral_m;
            max_lateral = min_lateral;
        } else {
            min_lateral = std::min(min_lateral, sample.point.lateral_m);
            max_lateral = std::max(max_lateral, sample.point.lateral_m);
        }
        ++used_count;
    }
    return used_count >= kMinimumLinePointCount &&
           max_lateral - min_lateral <= max_lateral_span_m;
}

bool FitTerminalTangent(const std::vector<port::BEVPoint>& points,
                        float fit_span_m,
                        port::BEVPoint& direction) {
    if (points.size() < kMinimumLinePointCount || fit_span_m <= 0.0F) {
        return false;
    }

    std::vector<float> distance_to_end(points.size(), 0.0F);
    std::size_t first = points.size() - 1U;
    float accumulated_m = 0.0F;
    while (first > 0U) {
        const port::BEVPoint& a = points[first - 1U];
        const port::BEVPoint& b = points[first];
        const float segment_m = std::hypot(b.forward_m - a.forward_m,
                                           b.lateral_m - a.lateral_m);
        if (accumulated_m + segment_m > fit_span_m) {
            break;
        }
        accumulated_m += segment_m;
        --first;
        distance_to_end[first] = accumulated_m;
    }
    if (points.size() - first < kMinimumLinePointCount) {
        return false;
    }

    double weight_sum = 0.0;
    double mean_forward = 0.0;
    double mean_lateral = 0.0;
    for (std::size_t index = first; index < points.size(); ++index) {
        const double weight = 1.0 +
            static_cast<double>(fit_span_m - distance_to_end[index]) /
                static_cast<double>(fit_span_m);
        weight_sum += weight;
        mean_forward += weight * points[index].forward_m;
        mean_lateral += weight * points[index].lateral_m;
    }
    mean_forward /= weight_sum;
    mean_lateral /= weight_sum;

    double covariance_ff = 0.0;
    double covariance_fl = 0.0;
    double covariance_ll = 0.0;
    for (std::size_t index = first; index < points.size(); ++index) {
        const double weight = 1.0 +
            static_cast<double>(fit_span_m - distance_to_end[index]) /
                static_cast<double>(fit_span_m);
        const double df = points[index].forward_m - mean_forward;
        const double dl = points[index].lateral_m - mean_lateral;
        covariance_ff += weight * df * df;
        covariance_fl += weight * df * dl;
        covariance_ll += weight * dl * dl;
    }
    const double angle = 0.5 * std::atan2(2.0 * covariance_fl,
                                          covariance_ff - covariance_ll);
    direction.forward_m = static_cast<float>(std::cos(angle));
    direction.lateral_m = static_cast<float>(std::sin(angle));

    const port::BEVPoint& first_point = points[first];
    const port::BEVPoint& last_point = points.back();
    const float orientation =
        direction.forward_m * (last_point.forward_m - first_point.forward_m) +
        direction.lateral_m * (last_point.lateral_m - first_point.lateral_m);
    if (orientation < 0.0F) {
        direction.forward_m = -direction.forward_m;
        direction.lateral_m = -direction.lateral_m;
    }
    return std::isfinite(direction.forward_m) &&
           std::isfinite(direction.lateral_m) &&
           direction.forward_m > 1.0e-4F;
}

bool BuildFovTangentBoundaryPath(const SceneFrameView& frame,
                                 CircleDir side,
                                 const CircleV2Params& params,
                                 port::BEVReferencePath& edge_path) {
    std::vector<port::BEVPoint> observed_points;
    observed_points.reserve(frame.rows.rows.size());
    std::size_t first_fov_row = frame.rows.rows.size();
    float previous_forward_m = 0.0F;

    for (std::size_t row_index = 0; row_index < frame.rows.rows.size(); ++row_index) {
        const BEVSimpleRowScan& row = frame.rows.rows[row_index];
        if (row.forward_m < params.exit_geometry_forward_min_m ||
            row.forward_m > params.exit_geometry_forward_max_m) {
            continue;
        }
        const BEVWhiteRun* run = row.valid ? UniqueOriginConnectedRun(row) : nullptr;
        if (run == nullptr) {
            if (!observed_points.empty()) {
                return false;
            }
            continue;
        }

        const BEVWhiteRunEndpointState endpoint = EndpointForSide(*run, side);
        if (endpoint == BEVWhiteRunEndpointState::kBoundary) {
            float lateral_m = 0.0F;
            if (!RealBoundaryForSide(*run, side, lateral_m)) {
                return false;
            }
            if (!observed_points.empty() &&
                row.forward_m - previous_forward_m > params.max_adjacent_distance_m) {
                return false;
            }
            observed_points.push_back({row.forward_m, lateral_m});
            previous_forward_m = row.forward_m;
            continue;
        }
        if (endpoint == BEVWhiteRunEndpointState::kFovEdge &&
            observed_points.size() >= kMinimumLinePointCount &&
            row.forward_m - previous_forward_m <= params.max_adjacent_distance_m) {
            first_fov_row = row_index;
            break;
        }
        if (!observed_points.empty()) {
            return false;
        }
    }

    if (first_fov_row >= frame.rows.rows.size()) {
        return false;
    }
    port::BEVPoint tangent{};
    if (!FitTerminalTangent(observed_points, params.exit_tangent_fit_span_m, tangent)) {
        return false;
    }

    edge_path.mode = port::ReferenceMode::kIntervalCenter;
    std::size_t point_count = 0U;
    for (const port::BEVPoint& point : observed_points) {
        AppendBoundaryPoint(edge_path, point_count, point, 0.8F);
    }
    const port::BEVPoint cutpoint = observed_points.back();
    const std::size_t observed_count = point_count;
    for (std::size_t row_index = first_fov_row;
         row_index < frame.rows.rows.size() && point_count < edge_path.sampled_path.size();
         ++row_index) {
        const float forward_m = frame.rows.rows[row_index].forward_m;
        if (forward_m <= cutpoint.forward_m ||
            forward_m > params.exit_geometry_forward_max_m) {
            continue;
        }
        const float ray_distance_m =
            (forward_m - cutpoint.forward_m) / tangent.forward_m;
        if (!(ray_distance_m > 0.0F)) {
            continue;
        }
        const port::BEVPoint point{
            forward_m,
            cutpoint.lateral_m + ray_distance_m * tangent.lateral_m,
        };
        if (!std::isfinite(point.lateral_m)) {
            return false;
        }
        AppendBoundaryPoint(edge_path, point_count, point, 0.7F);
    }
    return point_count > observed_count;
}

float ExitTraceOffset(CircleDir dir, float road_half_width_m) {
    if (dir == CircleDir::kLeft) {
        return -road_half_width_m;
    }
    if (dir == CircleDir::kRight) {
        return road_half_width_m;
    }
    return 0.0F;
}

}  // namespace

CircleV2GeometryObservation ObserveCircleV2Geometry(const SceneFrameView& frame,
                                                    CircleDir dir,
                                                    const CircleV2Params& params) {
    CircleV2GeometryObservation observation{};
    if (dir == CircleDir::kNone) {
        return observation;
    }
    const OrdinaryRoadModel* ordinary_road =
        frame.ordinary_road.has_value() ? &*frame.ordinary_road : nullptr;
    CircleV2Geometry& inner = observation.inner;
    const std::size_t inner_point_count = BuildLeadingRealBoundaryPath(
        frame,
        dir,
        params.inner_geometry_forward_min_m,
        params.inner_geometry_forward_max_m,
        params.max_adjacent_distance_m,
        inner.edge_path);
    inner.available = inner_point_count >= kMinimumLinePointCount;
    if (inner.available) {
        inner.source = CircleV2GeometrySource::kObservedBoundary;
    }

    CircleV2Geometry& outer = observation.outer;
    if (ordinary_road == nullptr || ordinary_road->half_width.value_m <= 0.0F) {
        return observation;
    }
    outer.road_half_width_m = ordinary_road->half_width.value_m;
    const CircleDir outer_side = Opposite(dir);
    const std::size_t outer_point_count = BuildLeadingRealBoundaryPath(
        frame,
        outer_side,
        params.exit_geometry_forward_min_m,
        params.exit_geometry_forward_max_m,
        params.max_adjacent_distance_m,
        outer.edge_path);
    port::BEVReferencePath tangent_path{};
    // A real-boundary -> FOV transition means the published path contains inferred
    // geometry, even when the visible prefix is locally straight.
    if (BuildFovTangentBoundaryPath(frame, outer_side, params, tangent_path)) {
        outer.available = true;
        outer.source = CircleV2GeometrySource::kFovTangent;
        outer.edge_path = tangent_path;
    } else if (IsStraightEnough(outer.edge_path,
                                outer_point_count,
                                params.exit_straight_max_lateral_span_m)) {
        outer.available = true;
        outer.source = CircleV2GeometrySource::kObservedBoundary;
    }
    outer.reference_offset_m = ExitTraceOffset(dir, outer.road_half_width_m);
    return observation;
}

const CircleV2Geometry& SelectCircleV2Geometry(
    const CircleV2GeometryObservation& observation,
    CircleV2ReferenceRole role) {
    static const CircleV2Geometry kNone{};
    if (role == CircleV2ReferenceRole::kInnerTrace) {
        return observation.inner;
    }
    if (role == CircleV2ReferenceRole::kExitTrace) {
        return observation.outer;
    }
    return kNone;
}

}  // namespace ls2k::vision::detail
