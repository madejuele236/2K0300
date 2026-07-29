#include "vision/elements/circle_v2/detail/circle_v2_internal.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "port/bev_reference_path_utils.hpp"

namespace ls2k::vision::detail {
namespace {

constexpr std::size_t kMinimumLinePointCount = 2U;
constexpr float kFixedExitRayLateralPerForward = 1.73205080757F;

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

bool IsFartherTowardExit(CircleDir dir, float candidate_m, float current_m) {
    return dir == CircleDir::kRight ? candidate_m > current_m
                                    : candidate_m < current_m;
}

bool BuildFixedExitRayBoundaryPath(const SceneFrameView& frame,
                                   CircleDir dir,
                                   const CircleV2Params& params,
                                   port::BEVReferencePath& edge_path) {
    const CircleDir side = Opposite(dir);
    if (side == CircleDir::kNone) {
        return false;
    }

    std::vector<port::BEVPoint> observed_points;
    observed_points.reserve(frame.rows.rows.size());
    bool ended_at_fov = false;
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
                break;
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
                break;
            }
            observed_points.push_back({row.forward_m, lateral_m});
            previous_forward_m = row.forward_m;
            continue;
        }
        if (!observed_points.empty()) {
            ended_at_fov =
                endpoint == BEVWhiteRunEndpointState::kFovEdge &&
                row.forward_m - previous_forward_m <= params.max_adjacent_distance_m;
            break;
        }
    }

    if (observed_points.size() < kMinimumLinePointCount) {
        return false;
    }

    std::size_t cutpoint_index = 0U;
    for (std::size_t index = 1U; index < observed_points.size(); ++index) {
        if (IsFartherTowardExit(dir,
                                observed_points[index].lateral_m,
                                observed_points[cutpoint_index].lateral_m)) {
            cutpoint_index = index;
        }
    }
    const bool visible_turn = cutpoint_index + 1U < observed_points.size();
    const bool terminal_fov =
        ended_at_fov && cutpoint_index + 1U == observed_points.size();
    if ((!visible_turn && !terminal_fov) ||
        cutpoint_index + 1U < kMinimumLinePointCount) {
        return false;
    }

    edge_path.mode = port::ReferenceMode::kIntervalCenter;
    std::size_t point_count = 0U;
    for (std::size_t index = 0U; index <= cutpoint_index; ++index) {
        AppendBoundaryPoint(edge_path,
                            point_count,
                            observed_points[index],
                            0.8F);
    }
    const port::BEVPoint cutpoint = observed_points[cutpoint_index];
    const std::size_t observed_count = point_count;
    const float lateral_per_forward_m =
        dir == CircleDir::kRight ? kFixedExitRayLateralPerForward
                                 : -kFixedExitRayLateralPerForward;
    for (std::size_t row_index = 0U;
         row_index < frame.rows.rows.size();
         ++row_index) {
        const BEVSimpleRowScan& row = frame.rows.rows[row_index];
        const float forward_m = row.forward_m;
        if (forward_m <= cutpoint.forward_m ||
            forward_m > params.exit_geometry_forward_max_m) {
            continue;
        }
        const port::BEVPoint point{
            forward_m,
            cutpoint.lateral_m +
                lateral_per_forward_m * (forward_m - cutpoint.forward_m),
        };
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
    port::BEVReferencePath exit_ray_path{};
    if (BuildFixedExitRayBoundaryPath(frame, dir, params, exit_ray_path)) {
        outer.available = true;
        outer.source = CircleV2GeometrySource::kFixedExitRay;
        outer.edge_path = exit_ray_path;
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
