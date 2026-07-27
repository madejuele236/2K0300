#include "vision/elements/circle_v2/detail/circle_v2_internal.hpp"

#include <algorithm>
#include <cmath>

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

CircleV2Geometry ObserveCircleV2Geometry(const SceneFrameView& frame,
                                         const CircleV2ReferenceContext& reference,
                                         const CircleSideExpansionObservation& expansion,
                                         const CircleV2Params& params) {
    (void)expansion;
    CircleV2Geometry geometry{};
    const OrdinaryRoadModel* ordinary_road =
        frame.ordinary_road.has_value() ? &*frame.ordinary_road : nullptr;
    if (ordinary_road != nullptr) {
        geometry.road_half_width_m = ordinary_road->half_width.value_m;
    }

    if (reference.role == CircleV2ReferenceRole::kInnerTrace) {
        const std::size_t point_count = BuildLeadingRealBoundaryPath(
            frame,
            reference.dir,
            params.inner_geometry_forward_min_m,
            params.inner_geometry_forward_max_m,
            params.max_adjacent_distance_m,
            geometry.edge_path);
        geometry.available = point_count >= kMinimumLinePointCount;
        return geometry;
    }

    if (reference.role != CircleV2ReferenceRole::kExitTrace || ordinary_road == nullptr ||
        geometry.road_half_width_m <= 0.0F) {
        return geometry;
    }

    const std::size_t point_count = BuildLeadingRealBoundaryPath(
        frame,
        Opposite(reference.dir),
        params.exit_geometry_forward_min_m,
        params.exit_geometry_forward_max_m,
        params.max_adjacent_distance_m,
        geometry.edge_path);
    geometry.available = IsStraightEnough(
        geometry.edge_path, point_count, params.exit_straight_max_lateral_span_m);
    geometry.reference_offset_m =
        ExitTraceOffset(reference.dir, geometry.road_half_width_m);
    return geometry;
}

}  // namespace ls2k::vision::detail
