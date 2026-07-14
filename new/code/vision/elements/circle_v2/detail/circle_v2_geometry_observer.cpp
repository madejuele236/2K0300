#include "vision/elements/circle_v2/detail/circle_v2_internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ls2k::vision::detail {
namespace {

constexpr std::size_t kMinCircleV2LeadingSamples = 3U;
constexpr float kExitTraceMaxLateralSpanM = 0.132345777F;

bool FindOuterRowEdge(const vision::BEVSimpleRowScan& row,
                      float center_lateral_m,
                      CircleDir side,
                      float& edge_lateral_m) {
    float best_distance = std::numeric_limits<float>::infinity();
    bool found = false;
    for (const vision::BEVBoundarySpan& span : row.spans) {
        float near_edge = 0.0F;
        float outer_edge = 0.0F;
        bool on_requested_side = false;
        if (side == CircleDir::kLeft) {
            near_edge = span.right_m;
            outer_edge = span.left_m;
            on_requested_side = near_edge <= center_lateral_m;
        } else if (side == CircleDir::kRight) {
            near_edge = span.left_m;
            outer_edge = span.right_m;
            on_requested_side = near_edge >= center_lateral_m;
        }
        if (!on_requested_side || !std::isfinite(outer_edge)) {
            continue;
        }
        const float distance = std::fabs(near_edge - center_lateral_m);
        if (distance < best_distance) {
            best_distance = distance;
            edge_lateral_m = outer_edge;
            found = true;
        }
    }
    return found;
}

CircleDir Opposite(CircleDir dir) {
    if (dir == CircleDir::kLeft) {
        return CircleDir::kRight;
    }
    if (dir == CircleDir::kRight) {
        return CircleDir::kLeft;
    }
    return CircleDir::kNone;
}

CircleDir ExitEdgeSideForRole(const CircleV2ReferenceContext& reference) {
    if (reference.role == CircleV2ReferenceRole::kExitTrace) {
        return Opposite(reference.dir);
    }
    return CircleDir::kNone;
}

CircleDir InnerEdgeSideForRole(const CircleV2ReferenceContext& reference) {
    if (reference.role == CircleV2ReferenceRole::kInnerTrace) {
        return reference.dir;
    }
    return CircleDir::kNone;
}

bool IsFiniteSample(const port::BEVPathSample& sample) {
    return sample.present &&
           std::isfinite(sample.point.forward_m) &&
           std::isfinite(sample.point.lateral_m);
}

bool HasMinimumLeadingSegment(std::size_t leading_count) {
    return leading_count >= kMinCircleV2LeadingSamples;
}

bool RowHasEdgeFacts(const vision::BEVSimpleRowScan& row) {
    return row.valid && !row.spans.empty();
}

bool IsLeadingSegmentStraightEnough(const port::BEVReferencePath& edge_path,
                                    std::size_t leading_count) {
    if (!HasMinimumLeadingSegment(leading_count)) {
        return false;
    }
    float min_lateral = edge_path.sampled_path[0].point.lateral_m;
    float max_lateral = edge_path.sampled_path[0].point.lateral_m;
    for (std::size_t index = 0; index < leading_count; ++index) {
        const port::BEVPathSample& sample = edge_path.sampled_path[index];
        if (!IsFiniteSample(sample)) {
            return false;
        }
        min_lateral = std::min(min_lateral, sample.point.lateral_m);
        max_lateral = std::max(max_lateral, sample.point.lateral_m);
    }
    return max_lateral - min_lateral <= kExitTraceMaxLateralSpanM;
}

bool FindInnerRowEdge(const vision::BEVSimpleRowScan& row,
                      float center_lateral_m,
                      CircleDir side,
                      float& edge_lateral_m) {
    bool found = false;
    float best_distance = std::numeric_limits<float>::infinity();
    for (const vision::BEVBoundarySpan& span : row.spans) {
        const float candidates[2] = {span.left_m, span.right_m};
        float interval_edge = 0.0F;
        bool have_span_edge = false;
        float interval_edge_distance = 0.0F;
        for (const float candidate : candidates) {
            const bool on_requested_side =
                (side == CircleDir::kLeft && candidate <= center_lateral_m) ||
                (side == CircleDir::kRight && candidate >= center_lateral_m);
            if (!on_requested_side || !std::isfinite(candidate)) {
                continue;
            }
            const float distance = std::fabs(candidate - center_lateral_m);
            if (!have_span_edge || distance < interval_edge_distance) {
                interval_edge = candidate;
                interval_edge_distance = distance;
                have_span_edge = true;
            }
        }
        if (!have_span_edge) {
            continue;
        }
        if (!found || interval_edge_distance < best_distance) {
            best_distance = interval_edge_distance;
            edge_lateral_m = interval_edge;
            found = true;
        }
    }
    return found;
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

bool CenterLateralAtForward(const OrdinaryRoadModel* ordinary_road,
                            float forward_m,
                            float& center_lateral_m,
                            float& confidence) {
    center_lateral_m = 0.0F;
    confidence = 0.8F;
    if (ordinary_road == nullptr) {
        return true;
    }
    bool have_previous = false;
    port::BEVPathSample previous{};
    for (const port::BEVPathSample& sample : ordinary_road->center_path.sampled_path) {
        if (!IsFiniteSample(sample)) {
            break;
        }
        if (std::fabs(sample.point.forward_m - forward_m) <= 1.0e-5F) {
            center_lateral_m = sample.point.lateral_m;
            confidence = sample.confidence;
            return true;
        }
        if (sample.point.forward_m > forward_m) {
            if (!have_previous) {
                return false;
            }
            const float dy = sample.point.forward_m - previous.point.forward_m;
            if (dy <= 1.0e-5F) {
                return false;
            }
            const float t =
                std::clamp((forward_m - previous.point.forward_m) / dy, 0.0F, 1.0F);
            center_lateral_m =
                previous.point.lateral_m +
                t * (sample.point.lateral_m - previous.point.lateral_m);
            confidence = previous.confidence > 0.0F ? previous.confidence : sample.confidence;
            return true;
        }
        previous = sample;
        have_previous = true;
    }
    return false;
}

CircleV2Geometry ObserveInnerTraceGeometry(const SceneFrameView& frame,
                                           const CircleV2ReferenceContext& reference,
                                           const OrdinaryRoadModel* ordinary_road,
                                           const CircleV2Params& /*params*/) {
    CircleV2Geometry geometry{};
    const CircleDir edge_side = InnerEdgeSideForRole(reference);
    if (edge_side == CircleDir::kNone) {
        return geometry;
    }

    port::BEVReferencePath edge_path{};
    edge_path.mode = port::ReferenceMode::kIntervalCenter;
    std::size_t leading_count = 0;
    bool segment_started = false;
    for (std::size_t row_index = 0; row_index < frame.rows.rows.size(); ++row_index) {
        const vision::BEVSimpleRowScan& row = frame.rows.rows[row_index];
        if (!RowHasEdgeFacts(row)) {
            if (segment_started) {
                break;
            }
            continue;
        }
        float center_lateral = 0.0F;
        float confidence = 0.8F;
        if (!CenterLateralAtForward(ordinary_road, row.forward_m, center_lateral, confidence)) {
            if (segment_started) {
                break;
            }
            continue;
        }
        float edge_lateral = 0.0F;
        const bool edge_found =
            FindInnerRowEdge(row, center_lateral, edge_side, edge_lateral);
        if (!edge_found) {
            if (segment_started) {
                break;
            }
            continue;
        }
        if (leading_count >= edge_path.sampled_path.size()) {
            break;
        }
        port::BEVPathSample& edge_sample = edge_path.sampled_path[leading_count];
        edge_sample.present = true;
        edge_sample.point.forward_m = row.forward_m;
        edge_sample.point.lateral_m = edge_lateral;
        edge_sample.confidence = confidence > 0.0F ? confidence : 0.8F;
        edge_sample.source = port::BEVPathPointSource::kIntervalCenter;
        segment_started = true;
        ++leading_count;
    }

    geometry.available = HasMinimumLeadingSegment(leading_count);
    geometry.edge_path = edge_path;
    return geometry;
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
        return ObserveInnerTraceGeometry(frame, reference, ordinary_road, params);
    }

    const CircleDir edge_side = ExitEdgeSideForRole(reference);
    if (ordinary_road == nullptr ||
        edge_side == CircleDir::kNone ||
        geometry.road_half_width_m <= 0.0F) {
        return geometry;
    }

    port::BEVReferencePath edge_path{};
    edge_path.mode = port::ReferenceMode::kIntervalCenter;
    std::size_t leading_count = 0;
    bool segment_started = false;
    for (std::size_t row_index = 0; row_index < frame.rows.rows.size(); ++row_index) {
        const vision::BEVSimpleRowScan& row = frame.rows.rows[row_index];
        if (!RowHasEdgeFacts(row)) {
            if (segment_started) {
                break;
            }
            continue;
        }
        float center_lateral = 0.0F;
        float confidence = 0.8F;
        if (!CenterLateralAtForward(ordinary_road, row.forward_m, center_lateral, confidence)) {
            if (segment_started) {
                break;
            }
            continue;
        }
        float edge_lateral = 0.0F;
        const bool edge_found =
            FindOuterRowEdge(row, center_lateral, edge_side, edge_lateral);
        if (!edge_found) {
            if (segment_started) {
                break;
            }
            continue;
        }
        if (leading_count >= edge_path.sampled_path.size()) {
            break;
        }
        port::BEVPathSample& edge_sample = edge_path.sampled_path[leading_count];
        edge_sample.present = true;
        edge_sample.point.forward_m = row.forward_m;
        edge_sample.point.lateral_m = edge_lateral;
        edge_sample.confidence = confidence > 0.0F ? confidence : 0.8F;
        edge_sample.source = port::BEVPathPointSource::kIntervalCenter;
        segment_started = true;
        ++leading_count;
    }

    geometry.available =
        IsLeadingSegmentStraightEnough(edge_path, leading_count);
    geometry.edge_path = edge_path;
    geometry.reference_offset_m = ExitTraceOffset(reference.dir, geometry.road_half_width_m);
    return geometry;
}

}  // namespace ls2k::vision::detail
