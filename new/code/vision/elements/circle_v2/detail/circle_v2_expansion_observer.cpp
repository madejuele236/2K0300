#include "vision/elements/circle_v2/detail/circle_v2_internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace ls2k::vision::detail {
namespace {

constexpr float kEpsilon = 1.0e-4F;
constexpr float kFitVarianceEpsilon = 1.0e-8F;
constexpr float kStraightDriftMaxM = 0.066172889F;

struct RowObservation {
    float forward_m = 0.0F;
    float sampleable_width_m = 0.0F;
    const BEVWhiteRun* run = nullptr;
};

struct TracePoint {
    float forward_m = 0.0F;
    float lateral_m = 0.0F;
};

struct LineFit {
    bool valid = false;
    bool straight = false;
    bool shrink = false;
    float slope = 0.0F;
    float intercept = 0.0F;
    float confidence = 0.0F;
};

bool EndpointObserved(BEVWhiteRunEndpointState state) {
    return state == BEVWhiteRunEndpointState::kBoundary;
}

bool EndpointAtFov(BEVWhiteRunEndpointState state) {
    return state == BEVWhiteRunEndpointState::kFovEdge;
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

std::vector<RowObservation> CollectRows(const SceneFrameView& frame,
                                        const CircleV2Params& params) {
    std::vector<RowObservation> rows;
    rows.reserve(frame.rows.rows.size());
    for (std::size_t row_index = 0; row_index < frame.rows.rows.size(); ++row_index) {
        const BEVSimpleRowScan& row = frame.rows.rows[row_index];
        if (row.forward_m < params.opening_forward_min_m ||
            row.forward_m > params.opening_forward_max_m) {
            continue;
        }
        const BEVWhiteRun* run = row.valid &&
                                         row.sampleable_width_m >= params.min_sampleable_width_m
                                     ? UniqueOriginConnectedRun(row)
                                     : nullptr;
        rows.push_back({row.forward_m, row.sampleable_width_m, run});
    }
    std::sort(rows.begin(), rows.end(), [](const RowObservation& lhs,
                                           const RowObservation& rhs) {
        return lhs.forward_m < rhs.forward_m;
    });
    return rows;
}

bool SideObserved(const RowObservation& row, bool left) {
    if (row.run == nullptr) {
        return false;
    }
    return EndpointObserved(left ? row.run->left_endpoint : row.run->right_endpoint);
}

float SideLateral(const RowObservation& row, bool left) {
    return left ? row.run->left_m : row.run->right_m;
}

bool EffectiveOpeningLateral(const RowObservation& row,
                             bool left,
                             float& lateral_m,
                             CircleOpeningSource& source) {
    if (row.run == nullptr) {
        return false;
    }
    const BEVWhiteRunEndpointState side =
        left ? row.run->left_endpoint : row.run->right_endpoint;
    const BEVWhiteRunEndpointState opposite =
        left ? row.run->right_endpoint : row.run->left_endpoint;
    if (EndpointObserved(side)) {
        lateral_m = SideLateral(row, left);
        source = CircleOpeningSource::kObservedBoundary;
        return true;
    }
    if (EndpointAtFov(side) && EndpointObserved(opposite)) {
        lateral_m = SideLateral(row, left);
        source = CircleOpeningSource::kFovEdgeLowerBound;
        return true;
    }
    return false;
}

float Reach(float lateral_m, bool left) {
    return left ? std::max(0.0F, -lateral_m) : std::max(0.0F, lateral_m);
}

LineFit FitLine(const std::vector<TracePoint>& trace,
                bool left,
                const CircleV2Params& params) {
    LineFit fit{};
    if (trace.size() < 2U) {
        return fit;
    }

    float weight_sum = 0.0F;
    float mean_x = 0.0F;
    float mean_y = 0.0F;
    std::vector<float> weights(trace.size(), 1.0F);
    if (trace.size() > 2U) {
        for (std::size_t i = 0; i < trace.size(); ++i) {
            const float left_gap = i == 0U ? trace[1].forward_m - trace[0].forward_m
                                           : trace[i].forward_m - trace[i - 1U].forward_m;
            const float right_gap = i + 1U == trace.size()
                                        ? trace[i].forward_m - trace[i - 1U].forward_m
                                        : trace[i + 1U].forward_m - trace[i].forward_m;
            weights[i] = std::max(kEpsilon, 0.5F * (left_gap + right_gap));
        }
    }
    for (std::size_t i = 0; i < trace.size(); ++i) {
        weight_sum += weights[i];
        mean_x += weights[i] * trace[i].forward_m;
        mean_y += weights[i] * trace[i].lateral_m;
    }
    if (weight_sum <= kEpsilon) {
        return fit;
    }
    mean_x /= weight_sum;
    mean_y /= weight_sum;

    float var_x = 0.0F;
    float cov_xy = 0.0F;
    for (std::size_t i = 0; i < trace.size(); ++i) {
        const float dx = trace[i].forward_m - mean_x;
        var_x += weights[i] * dx * dx;
        cov_xy += weights[i] * dx * (trace[i].lateral_m - mean_y);
    }
    if (var_x <= kFitVarianceEpsilon) {
        return fit;
    }
    fit.slope = cov_xy / var_x;
    fit.intercept = mean_y - fit.slope * mean_x;

    float squared_error_sum = 0.0F;
    for (std::size_t i = 0; i < trace.size(); ++i) {
        const float error = trace[i].lateral_m -
                            (fit.slope * trace[i].forward_m + fit.intercept);
        squared_error_sum += weights[i] * error * error;
    }
    const float rmse = std::sqrt(squared_error_sum / weight_sum);
    const float near_reach = Reach(fit.slope * trace.front().forward_m + fit.intercept, left);
    const float far_reach = Reach(fit.slope * trace.back().forward_m + fit.intercept, left);
    const float reach_delta = near_reach - far_reach;
    fit.shrink = reach_delta > params.opening_distance_min_m &&
                 reach_delta / std::max(kEpsilon, near_reach) >= 0.10F;
    fit.valid = true;
    fit.straight = rmse <= kStraightDriftMaxM && !fit.shrink;
    fit.confidence = std::clamp(1.0F - rmse / kStraightDriftMaxM, 0.0F, 1.0F);
    return fit;
}

bool ReliableStraight(const LineFit& fit, const CircleV2Params& params) {
    return fit.valid && fit.straight &&
           fit.confidence >= params.opposite_straight_confidence_min;
}

float DirectedDistance(const LineFit& baseline,
                       float forward_m,
                       float lateral_m,
                       bool left) {
    const float predicted = baseline.slope * forward_m + baseline.intercept;
    const float normalizer = std::sqrt(1.0F + baseline.slope * baseline.slope);
    return left ? (predicted - lateral_m) / normalizer
                : (lateral_m - predicted) / normalizer;
}

CircleOpeningObservation DetectOpening(const std::vector<RowObservation>& rows,
                                       bool left,
                                       const CircleV2Params& params) {
    CircleOpeningObservation opening{};
    for (std::size_t candidate = 2U; candidate < rows.size(); ++candidate) {
        std::size_t baseline_begin = candidate;
        while (baseline_begin > 0U) {
            const std::size_t previous = baseline_begin - 1U;
            if (!SideObserved(rows[previous], left)) {
                break;
            }
            if (baseline_begin < candidate &&
                rows[baseline_begin].forward_m - rows[previous].forward_m >
                    params.max_adjacent_distance_m) {
                break;
            }
            baseline_begin = previous;
        }
        if (candidate - baseline_begin < 2U ||
            rows[candidate].forward_m - rows[candidate - 1U].forward_m >
                params.max_adjacent_distance_m) {
            continue;
        }

        std::vector<TracePoint> baseline_trace;
        baseline_trace.reserve(candidate - baseline_begin);
        for (std::size_t index = baseline_begin; index < candidate; ++index) {
            baseline_trace.push_back({rows[index].forward_m, SideLateral(rows[index], left)});
        }
        const LineFit baseline = FitLine(baseline_trace, left, params);
        if (!baseline.valid) {
            continue;
        }

        float effective_lateral = 0.0F;
        CircleOpeningSource source = CircleOpeningSource::kNone;
        if (!EffectiveOpeningLateral(rows[candidate], left, effective_lateral, source) ||
            DirectedDistance(baseline, rows[candidate].forward_m, effective_lateral, left) <
                params.opening_distance_min_m) {
            continue;
        }

        std::size_t confirmed_end = candidate;
        for (std::size_t index = candidate + 1U; index < rows.size(); ++index) {
            if (rows[index].forward_m - rows[index - 1U].forward_m >
                params.max_adjacent_distance_m) {
                break;
            }
            float confirm_lateral = 0.0F;
            CircleOpeningSource confirm_source = CircleOpeningSource::kNone;
            if (!EffectiveOpeningLateral(rows[index], left, confirm_lateral, confirm_source) ||
                DirectedDistance(baseline, rows[index].forward_m, confirm_lateral, left) <
                    params.opening_distance_min_m) {
                break;
            }
            confirmed_end = index;
            if (rows[confirmed_end].forward_m - rows[candidate].forward_m >=
                params.opening_confirm_forward_span_m) {
                break;
            }
        }
        const float confirmed_span =
            rows[confirmed_end].forward_m - rows[candidate].forward_m;
        if (confirmed_span < params.opening_confirm_forward_span_m) {
            continue;
        }

        std::vector<TracePoint> opposite_trace;
        opposite_trace.reserve(confirmed_end - baseline_begin + 1U);
        bool opposite_complete = true;
        for (std::size_t index = baseline_begin; index <= confirmed_end; ++index) {
            if (!SideObserved(rows[index], !left)) {
                opposite_complete = false;
                break;
            }
            opposite_trace.push_back({rows[index].forward_m, SideLateral(rows[index], !left)});
        }
        const bool opposite_straight =
            opposite_complete &&
            ReliableStraight(FitLine(opposite_trace, !left, params), params);
        if (!opposite_straight) {
            continue;
        }

        opening.available = true;
        opening.frontier_forward_m = rows[candidate].forward_m;
        opening.effective_lateral_m = effective_lateral;
        opening.source = source;
        opening.outward_distance_m =
            DirectedDistance(baseline, rows[candidate].forward_m, effective_lateral, left);
        opening.confirmed_forward_span_m = confirmed_span;
        opening.origin_connected = true;
        opening.opposite_straight = true;
        return opening;
    }
    return opening;
}

}  // namespace

CircleSideExpansionObservation ObserveCircleSideExpansion(const SceneFrameView& frame,
                                                          const CircleV2Params& params) {
    CircleSideExpansionObservation observation{};
    const std::vector<RowObservation> rows = CollectRows(frame, params);
    observation.openings.left = DetectOpening(rows, true, params);
    observation.openings.right = DetectOpening(rows, false, params);
    if (observation.openings.left.available != observation.openings.right.available) {
        observation.detected_dir = observation.openings.left.available
                                       ? CircleDir::kLeft
                                       : CircleDir::kRight;
    }
    return observation;
}

}  // namespace ls2k::vision::detail
