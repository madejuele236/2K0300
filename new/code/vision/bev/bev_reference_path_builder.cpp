#include "vision/bev/bev_reference_path_builder.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "vision/bev/bev_boundary_trace_clip.hpp"

namespace ls2k::vision {
namespace {

void InitializeReferencePath(port::BEVReferencePath& reference,
                             const port::RuntimeParameters& params,
                             port::ReferenceMode mode) {
    reference.mode = mode;
    for (std::size_t index = 0; index < reference.sampled_path.size(); ++index) {
        port::BEVPathSample& sample = reference.sampled_path[index];
        sample.present = false;
        sample.point.forward_m = params.bev_geometry.forward_samples_m[index];
        sample.point.lateral_m = 0.0F;
        sample.confidence = 0.0F;
        sample.source = port::BEVPathPointSource::kNone;
    }
}

struct CenterCandidate {
    float forward_m = 0.0F;
    float lateral_m = 0.0F;
};

enum class SingleEdgeKind {
    kLow,
    kHigh,
};

using CenterCandidateRows =
    std::array<std::vector<CenterCandidate>, port::kBevReferenceSampleCount>;

BEVBoundaryTraceClipOptions BoundaryTraceClipOptionsFromParams(
    const port::RuntimeParameters& params) {
    return BEVBoundaryTraceClipOptions{
        params.bev_geometry.boundary_trace_max_adjacent_distance_m};
}

float EdgeLateral(const BEVBoundarySpan& span, SingleEdgeKind kind) {
    return kind == SingleEdgeKind::kLow ? span.left_m : span.right_m;
}

port::BEVPoint EdgePoint(const BEVSimpleRowScan& row,
                         const BEVBoundarySpan& span,
                         SingleEdgeKind kind) {
    return port::BEVPoint{row.forward_m, EdgeLateral(span, kind)};
}

bool SameBoundaryTracePoint(const BEVBoundaryTracePoint& point,
                            std::size_t row_index,
                            const port::BEVPoint& edge_point) {
    return point.row_index == row_index &&
           point.point.forward_m == edge_point.forward_m &&
           point.point.lateral_m == edge_point.lateral_m;
}

std::vector<BEVBoundaryTracePoint> BuildBoundaryTraceForEdge(
    const std::vector<BEVSimpleRowScan>& rows,
    SingleEdgeKind kind,
    std::size_t current_row_index,
    std::size_t current_span_index,
    float anchor_lateral_m) {
    std::vector<BEVBoundaryTracePoint> trace;
    const std::size_t count =
        std::min(rows.size(), static_cast<std::size_t>(port::kBevReferenceSampleCount));
    trace.reserve(count);
    for (std::size_t row_index = 0U; row_index < count; ++row_index) {
        const BEVSimpleRowScan& row = rows[row_index];
        if (!row.valid) {
            continue;
        }
        const BEVBoundarySpan* selected = nullptr;
        float best_cost = 0.0F;
        for (std::size_t span_index = 0U;
             span_index < row.spans.size();
             ++span_index) {
            const BEVBoundarySpan& span = row.spans[span_index];
            if (row_index == current_row_index &&
                span_index == current_span_index) {
                selected = &span;
                break;
            }
            const float cost =
                std::fabs(EdgeLateral(span, kind) - anchor_lateral_m);
            if (selected == nullptr || cost < best_cost) {
                selected = &span;
                best_cost = cost;
            }
        }
        if (selected != nullptr) {
            trace.push_back(BEVBoundaryTracePoint{
                row_index,
                EdgePoint(row, *selected, kind),
            });
        }
    }
    return trace;
}

struct BoundarySupport {
    bool current_kept = false;
    bool has_neighbor = false;
    port::BEVPoint neighbor{};
};

struct CachedIntervalSupport {
    BoundarySupport low{};
    BoundarySupport high{};
};

using IntervalSupportRows =
    std::array<std::vector<CachedIntervalSupport>, port::kBevReferenceSampleCount>;

BoundarySupport FindBoundarySupport(
    const std::vector<BEVSimpleRowScan>& rows,
    SingleEdgeKind kind,
    const BEVBoundaryTraceClipOptions& clip_options,
    std::size_t row_index,
    std::size_t span_index) {
    BoundarySupport support{};
    if (row_index >= rows.size() ||
        span_index >= rows[row_index].spans.size()) {
        return support;
    }
    const BEVSimpleRowScan& row = rows[row_index];
    const BEVBoundarySpan& span = row.spans[span_index];
    const port::BEVPoint current = EdgePoint(row, span, kind);
    const std::vector<BEVBoundaryTracePoint> raw_trace =
        BuildBoundaryTraceForEdge(rows,
                                  kind,
                                  row_index,
                                  span_index,
                                  current.lateral_m);
    const std::vector<BEVBoundaryTracePoint> clipped_trace =
        ClipBoundaryTraceOutliers(raw_trace, clip_options);

    std::size_t best_row_gap = 0U;
    float best_lateral_gap = 0.0F;
    for (const BEVBoundaryTracePoint& point : clipped_trace) {
        if (SameBoundaryTracePoint(point, row_index, current)) {
            support.current_kept = true;
            continue;
        }
        if (point.row_index <= row_index) {
            continue;
        }
        const std::size_t row_gap = point.row_index - row_index;
        const float lateral_gap = std::fabs(point.point.lateral_m - current.lateral_m);
        if (!support.has_neighbor ||
            row_gap < best_row_gap ||
            (row_gap == best_row_gap && lateral_gap < best_lateral_gap)) {
            support.has_neighbor = true;
            support.neighbor = point.point;
            best_row_gap = row_gap;
            best_lateral_gap = lateral_gap;
        }
    }
    if (!support.current_kept) {
        support.has_neighbor = false;
    }
    return support;
}

const CachedIntervalSupport* CachedSupportAt(const IntervalSupportRows& cache,
                                             std::size_t row_index,
                                             std::size_t interval_index) {
    if (row_index >= cache.size() || interval_index >= cache[row_index].size()) {
        return nullptr;
    }
    return &cache[row_index][interval_index];
}

IntervalSupportRows BuildBoundaryTraceSupport(
    const std::vector<BEVSimpleRowScan>& rows,
    const port::RuntimeParameters& params) {
    IntervalSupportRows cache{};
    const std::size_t count =
        std::min(rows.size(), static_cast<std::size_t>(port::kBevReferenceSampleCount));
    const BEVBoundaryTraceClipOptions clip_options =
        BoundaryTraceClipOptionsFromParams(params);
    for (std::size_t row_index = 0; row_index < count; ++row_index) {
        const BEVSimpleRowScan& row = rows[row_index];
        if (!row.valid || row.spans.empty()) {
            continue;
        }
        std::vector<CachedIntervalSupport>& row_cache = cache[row_index];
        row_cache.reserve(row.spans.size());
        for (std::size_t span_index = 0U;
             span_index < row.spans.size();
             ++span_index) {
            CachedIntervalSupport support{};
            support.low = FindBoundarySupport(rows,
                                              SingleEdgeKind::kLow,
                                              clip_options,
                                              row_index,
                                              span_index);
            support.high = FindBoundarySupport(rows,
                                               SingleEdgeKind::kHigh,
                                               clip_options,
                                               row_index,
                                               span_index);
            row_cache.push_back(support);
        }
    }
    return cache;
}

bool IntervalSupportsMidpointCandidate(
    const std::vector<BEVSimpleRowScan>& rows,
    const IntervalSupportRows& support_cache,
    std::size_t row_index,
    std::size_t span_index) {
    if (row_index >= rows.size() ||
        span_index >= rows[row_index].spans.size()) {
        return false;
    }
    const CachedIntervalSupport* support =
        CachedSupportAt(support_cache, row_index, span_index);
    if (support == nullptr ||
        !support->low.current_kept ||
        !support->high.current_kept) {
        return false;
    }
    return true;
}

bool IsSingleEdgeInterval(
    const IntervalSupportRows& support_cache,
    std::size_t row_index,
    std::size_t interval_index,
    SingleEdgeKind kind) {
    const CachedIntervalSupport* support =
        CachedSupportAt(support_cache, row_index, interval_index);
    if (support == nullptr) {
        return false;
    }
    if (kind == SingleEdgeKind::kLow) {
        return support->low.current_kept && !support->high.current_kept;
    }
    return !support->low.current_kept && support->high.current_kept;
}

float SignedNormalOffset(const port::RuntimeParameters& params, SingleEdgeKind kind) {
    const float nominal = params.bev_geometry.nominal_road_half_width_m;
    return kind == SingleEdgeKind::kLow ? nominal : -nominal;
}

bool BuildSingleEdgeCenterCandidate(const port::BEVPoint& current,
                                    const port::BEVPoint& neighbor,
                                    float signed_normal_offset_m,
                                    CenterCandidate& candidate) {
    if (!std::isfinite(current.forward_m) ||
        !std::isfinite(current.lateral_m) ||
        !std::isfinite(neighbor.forward_m) ||
        !std::isfinite(neighbor.lateral_m) ||
        !std::isfinite(signed_normal_offset_m)) {
        return false;
    }
    const float delta_forward_m = neighbor.forward_m - current.forward_m;
    if (delta_forward_m == 0.0F) {
        return false;
    }
    const float slope =
        (neighbor.lateral_m - current.lateral_m) / delta_forward_m;
    const float center_lateral =
        current.lateral_m +
        signed_normal_offset_m * std::sqrt(1.0F + slope * slope);
    if (!std::isfinite(slope) || !std::isfinite(center_lateral)) {
        return false;
    }
    candidate.forward_m = current.forward_m;
    candidate.lateral_m = center_lateral;
    return true;
}

void AddSingleEdgeCandidates(const std::vector<BEVSimpleRowScan>& rows,
                             SingleEdgeKind kind,
                             const port::RuntimeParameters& params,
                             const IntervalSupportRows& support_cache,
                             CenterCandidateRows& candidate_rows) {
    const std::size_t count =
        std::min(rows.size(), static_cast<std::size_t>(port::kBevReferenceSampleCount));
    for (std::size_t row_index = 0; row_index < count; ++row_index) {
        const BEVSimpleRowScan& row = rows[row_index];
        if (!row.valid) {
            continue;
        }
        for (std::size_t interval_index = 0U;
             interval_index < row.spans.size();
             ++interval_index) {
            const BEVBoundarySpan& span = row.spans[interval_index];
            if (!IsSingleEdgeInterval(support_cache,
                                      row_index,
                                      interval_index,
                                      kind)) {
                continue;
            }
            const CachedIntervalSupport* cached_support =
                CachedSupportAt(support_cache, row_index, interval_index);
            if (cached_support == nullptr) {
                continue;
            }
            const BoundarySupport& support =
                kind == SingleEdgeKind::kLow ? cached_support->low
                                             : cached_support->high;
            if (!support.current_kept || !support.has_neighbor) {
                continue;
            }
            CenterCandidate candidate{};
            if (!BuildSingleEdgeCenterCandidate(EdgePoint(row, span, kind),
                                                support.neighbor,
                                                SignedNormalOffset(params, kind),
                                                candidate)) {
                continue;
            }
            candidate_rows[row_index].push_back(candidate);
        }
    }
}

SingleEdgeKind EdgeKindFromJump(const BEVBoundaryJump& jump) {
    return jump.polarity == BEVBoundaryJumpPolarity::kRisingY
               ? SingleEdgeKind::kLow
               : SingleEdgeKind::kHigh;
}

const BEVBoundaryJump* FindNextConnectedJump(const std::vector<BEVSimpleRowScan>& rows,
                                             std::size_t row_index,
                                             const BEVBoundaryJump& current,
                                             const port::RuntimeParameters& params) {
    if (row_index + 1U >= rows.size()) {
        return nullptr;
    }
    const BEVSimpleRowScan& next_row = rows[row_index + 1U];
    if (!next_row.valid) {
        return nullptr;
    }
    const float max_distance =
        std::max(0.0F, params.bev_geometry.boundary_trace_max_adjacent_distance_m);
    const BEVBoundaryJump* best = nullptr;
    float best_distance = 0.0F;
    for (const BEVBoundaryJump& candidate : next_row.jumps) {
        if (candidate.polarity != current.polarity) {
            continue;
        }
        const float distance =
            std::hypot(candidate.forward_m - current.forward_m,
                       candidate.lateral_m - current.lateral_m);
        if (distance > max_distance) {
            continue;
        }
        if (best == nullptr || distance < best_distance) {
            best = &candidate;
            best_distance = distance;
        }
    }
    return best;
}

bool JumpBelongsToSpan(const BEVSimpleRowScan& row, const BEVBoundaryJump& jump) {
    for (const BEVBoundarySpan& span : row.spans) {
        if (jump.polarity == BEVBoundaryJumpPolarity::kRisingY &&
            span.left_lateral_index == jump.lateral_index &&
            span.left_m == jump.lateral_m) {
            return true;
        }
        if (jump.polarity == BEVBoundaryJumpPolarity::kFallingY &&
            span.right_lateral_index == jump.lateral_index &&
            span.right_m == jump.lateral_m) {
            return true;
        }
    }
    return false;
}

void AddSingleBoundaryJumpCandidates(const std::vector<BEVSimpleRowScan>& rows,
                                     const port::RuntimeParameters& params,
                                     CenterCandidateRows& candidate_rows) {
    const std::size_t count =
        std::min(rows.size(), static_cast<std::size_t>(port::kBevReferenceSampleCount));
    for (std::size_t row_index = 0; row_index < count; ++row_index) {
        const BEVSimpleRowScan& row = rows[row_index];
        if (!row.valid) {
            continue;
        }
        for (const BEVBoundaryJump& jump : row.jumps) {
            if (JumpBelongsToSpan(row, jump)) {
                continue;
            }
            const BEVBoundaryJump* neighbor =
                FindNextConnectedJump(rows, row_index, jump, params);
            if (neighbor == nullptr) {
                continue;
            }
            const SingleEdgeKind kind = EdgeKindFromJump(jump);
            CenterCandidate candidate{};
            if (!BuildSingleEdgeCenterCandidate(port::BEVPoint{row.forward_m, jump.lateral_m},
                                                port::BEVPoint{neighbor->forward_m,
                                                               neighbor->lateral_m},
                                                SignedNormalOffset(params, kind),
                                                candidate)) {
                continue;
            }
            candidate_rows[row_index].push_back(candidate);
        }
    }
}

CenterCandidateRows BuildOrdinaryCenterCandidates(
    const std::vector<BEVSimpleRowScan>& rows,
    const port::RuntimeParameters& params) {
    CenterCandidateRows candidate_rows{};
    const std::size_t count =
        std::min(rows.size(), static_cast<std::size_t>(port::kBevReferenceSampleCount));
    const IntervalSupportRows support_cache =
        BuildBoundaryTraceSupport(rows, params);

    for (std::size_t row_index = 0; row_index < count; ++row_index) {
        const BEVSimpleRowScan& row = rows[row_index];
        if (!row.valid) {
            continue;
        }
        for (std::size_t span_index = 0U;
             span_index < row.spans.size();
             ++span_index) {
            const BEVBoundarySpan& span = row.spans[span_index];
            if (!IntervalSupportsMidpointCandidate(rows,
                                                   support_cache,
                                                   row_index,
                                                   span_index)) {
                continue;
            }
            candidate_rows[row_index].push_back(
                CenterCandidate{row.forward_m,
                                0.5F * (span.left_m + span.right_m)});
        }
    }

    AddSingleEdgeCandidates(rows,
                            SingleEdgeKind::kLow,
                            params,
                            support_cache,
                            candidate_rows);
    AddSingleEdgeCandidates(rows,
                            SingleEdgeKind::kHigh,
                            params,
                            support_cache,
                            candidate_rows);
    AddSingleBoundaryJumpCandidates(rows, params, candidate_rows);
    return candidate_rows;
}

}  // namespace

port::BEVReferencePath BuildConnectedReferencePath(
    const std::vector<BEVSimpleRowScan>& rows,
    const port::RuntimeParameters& params,
    const BEVSegmentConnectivityQuery& connectivity_query) {
    port::BEVReferencePath reference{};
    InitializeReferencePath(reference, params, port::ReferenceMode::kNone);
    const CenterCandidateRows candidate_rows =
        BuildOrdinaryCenterCandidates(rows, params);
    port::BEVPoint predecessor{0.0F, 0.0F};
    bool have_accepted = false;
    std::size_t output_index = 0U;

    for (std::size_t index = 0; index < rows.size() && index < reference.sampled_path.size(); ++index) {
        const CenterCandidate* accepted = nullptr;
        for (const CenterCandidate& candidate : candidate_rows[index]) {
            const BEVSegmentVisibilityPolicy policy =
                have_accepted
                    ? BEVSegmentVisibilityPolicy::kRequireFullSegment
                    : BEVSegmentVisibilityPolicy::kAllowFromEndpointClip;
            const BEVSegmentConnectivityResult connectivity =
                connectivity_query.Evaluate(
                    predecessor,
                    port::BEVPoint{candidate.forward_m, candidate.lateral_m},
                    policy);
            if (connectivity.status == BEVSegmentConnectivityStatus::kConnected) {
                accepted = &candidate;
                break;
            }
        }
        if (accepted == nullptr) {
            continue;
        }

        if (output_index >= reference.sampled_path.size()) {
            break;
        }
        port::BEVPathSample& sample = reference.sampled_path[output_index];
        reference.mode = port::ReferenceMode::kIntervalCenter;
        sample.present = true;
        sample.point.forward_m = accepted->forward_m;
        sample.point.lateral_m = accepted->lateral_m;
        sample.confidence = 1.0F;
        sample.source = port::BEVPathPointSource::kIntervalCenter;
        predecessor = sample.point;
        have_accepted = true;
        ++output_index;
    }
    return reference;
}

port::BEVReferencePath BuildReferencePath(const std::vector<BEVSimpleRowScan>& rows,
                                          const port::RuntimeParameters& params,
                                          const BEVSegmentConnectivityQuery& connectivity_query) {
    return BuildConnectedReferencePath(rows, params, connectivity_query);
}

}  // namespace ls2k::vision
