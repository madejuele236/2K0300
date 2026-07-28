#include "vision/elements/circle_v2/detail/circle_v2_internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "port/perf_counter.hpp"

namespace ls2k::vision::detail {
namespace {

constexpr float kEpsilon = 1.0e-4F;
constexpr float kFitVarianceEpsilon = 1.0e-8F;
constexpr float kStraightDriftMaxM = 0.066172889F;

struct RowObservation {
    std::size_t row_index = 0U;
    float forward_m = 0.0F;
    float sampleable_width_m = 0.0F;
    float white_width_m = 0.0F;
    const BEVWhiteRun* run = nullptr;
};

struct SelectedCircleOpeningRows {
    std::vector<RowObservation> rows{};
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

bool IsLeft(BoundarySide side) {
    return side == BoundarySide::kLeft;
}

BoundarySide Opposite(BoundarySide side) {
    return IsLeft(side) ? BoundarySide::kRight : BoundarySide::kLeft;
}

CircleDir DirectionFor(BoundarySide side) {
    return IsLeft(side) ? CircleDir::kLeft : CircleDir::kRight;
}

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

SelectedCircleOpeningRows SelectCircleOpeningRows(const SceneFrameView& frame,
                                                  const CircleV2Params& params) {
    SelectedCircleOpeningRows selected{};
    selected.rows.reserve(frame.rows.rows.size());
    for (std::size_t source_index = 0U; source_index < frame.rows.rows.size();
         ++source_index) {
        const BEVSimpleRowScan& row = frame.rows.rows[source_index];
        if (row.forward_m < params.opening_forward_min_m ||
            row.forward_m > params.opening_forward_max_m) {
            continue;
        }
        const BEVWhiteRun* run =
            row.valid && row.sampleable_width_m >= params.min_sampleable_width_m
                ? UniqueOriginConnectedRun(row)
                : nullptr;
        selected.rows.push_back({
            source_index,
            row.forward_m,
            row.sampleable_width_m,
            run == nullptr ? 0.0F : run->right_m - run->left_m,
            run,
        });
    }
    std::stable_sort(selected.rows.begin(),
                     selected.rows.end(),
                     [](const RowObservation& lhs, const RowObservation& rhs) {
                         return lhs.forward_m < rhs.forward_m;
                     });
    for (std::size_t index = 0U; index < selected.rows.size(); ++index) {
        selected.rows[index].row_index = index;
    }
    return selected;
}

BEVWhiteRunEndpointState EndpointFor(const RowObservation& row, BoundarySide side) {
    if (row.run == nullptr) {
        return BEVWhiteRunEndpointState::kUnavailableGap;
    }
    return IsLeft(side) ? row.run->left_endpoint : row.run->right_endpoint;
}

bool SideObserved(const RowObservation& row, BoundarySide side) {
    return row.run != nullptr && EndpointObserved(EndpointFor(row, side));
}

float SideLateral(const RowObservation& row, BoundarySide side) {
    return IsLeft(side) ? row.run->left_m : row.run->right_m;
}

bool EffectiveOpeningLateral(const RowObservation& row,
                             BoundarySide side,
                             float& lateral_m,
                             CircleOpeningSource& source) {
    if (row.run == nullptr) {
        return false;
    }
    const BEVWhiteRunEndpointState selected = EndpointFor(row, side);
    const BEVWhiteRunEndpointState opposite = EndpointFor(row, Opposite(side));
    if (EndpointObserved(selected)) {
        lateral_m = SideLateral(row, side);
        source = CircleOpeningSource::kObservedBoundary;
        return true;
    }
    if (EndpointAtFov(selected) && EndpointObserved(opposite)) {
        lateral_m = SideLateral(row, side);
        source = CircleOpeningSource::kFovEdgeLowerBound;
        return true;
    }
    return false;
}

float Reach(float lateral_m, BoundarySide side) {
    return IsLeft(side) ? std::max(0.0F, -lateral_m)
                        : std::max(0.0F, lateral_m);
}

LineFit FitLine(const std::vector<TracePoint>& trace,
                BoundarySide side,
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
        for (std::size_t index = 0U; index < trace.size(); ++index) {
            const float left_gap =
                index == 0U ? trace[1].forward_m - trace[0].forward_m
                            : trace[index].forward_m - trace[index - 1U].forward_m;
            const float right_gap =
                index + 1U == trace.size()
                    ? trace[index].forward_m - trace[index - 1U].forward_m
                    : trace[index + 1U].forward_m - trace[index].forward_m;
            weights[index] = std::max(kEpsilon, 0.5F * (left_gap + right_gap));
        }
    }
    for (std::size_t index = 0U; index < trace.size(); ++index) {
        weight_sum += weights[index];
        mean_x += weights[index] * trace[index].forward_m;
        mean_y += weights[index] * trace[index].lateral_m;
    }
    if (weight_sum <= kEpsilon) {
        return fit;
    }
    mean_x /= weight_sum;
    mean_y /= weight_sum;

    float var_x = 0.0F;
    float cov_xy = 0.0F;
    for (std::size_t index = 0U; index < trace.size(); ++index) {
        const float dx = trace[index].forward_m - mean_x;
        var_x += weights[index] * dx * dx;
        cov_xy +=
            weights[index] * dx * (trace[index].lateral_m - mean_y);
    }
    if (var_x <= kFitVarianceEpsilon) {
        return fit;
    }
    fit.slope = cov_xy / var_x;
    fit.intercept = mean_y - fit.slope * mean_x;

    float squared_error_sum = 0.0F;
    for (std::size_t index = 0U; index < trace.size(); ++index) {
        const float error =
            trace[index].lateral_m -
            (fit.slope * trace[index].forward_m + fit.intercept);
        squared_error_sum += weights[index] * error * error;
    }
    const float rmse = std::sqrt(squared_error_sum / weight_sum);
    const float near_reach =
        Reach(fit.slope * trace.front().forward_m + fit.intercept, side);
    const float far_reach =
        Reach(fit.slope * trace.back().forward_m + fit.intercept, side);
    const float reach_delta = near_reach - far_reach;
    fit.shrink = reach_delta > params.opening_distance_min_m &&
                 reach_delta / std::max(kEpsilon, near_reach) >= 0.10F;
    fit.valid = true;
    fit.straight = rmse <= kStraightDriftMaxM && !fit.shrink;
    fit.confidence =
        std::clamp(1.0F - rmse / kStraightDriftMaxM, 0.0F, 1.0F);
    return fit;
}

float DirectedDistance(const LineFit& baseline,
                       float forward_m,
                       float lateral_m,
                       BoundarySide side) {
    const float predicted = baseline.slope * forward_m + baseline.intercept;
    const float normalizer = std::sqrt(1.0F + baseline.slope * baseline.slope);
    return IsLeft(side) ? (predicted - lateral_m) / normalizer
                        : (lateral_m - predicted) / normalizer;
}

ForwardInterval MakeInterval(const std::vector<RowObservation>& rows,
                             std::size_t begin,
                             std::size_t end) {
    return {
        rows[begin].row_index,
        rows[end].row_index,
        rows[begin].forward_m,
        rows[end].forward_m,
    };
}

bool OpeningRowMatches(const RowObservation& row,
                       BoundarySide side,
                       const LineFit& baseline,
                       const CircleV2Params& params,
                       float& lateral_m,
                       CircleOpeningSource& source,
                       float& outward_distance_m) {
    if (row.white_width_m <= params.nominal_road_width_m ||
        !EffectiveOpeningLateral(row, side, lateral_m, source)) {
        return false;
    }
    outward_distance_m =
        DirectedDistance(baseline, row.forward_m, lateral_m, side);
    return outward_distance_m >= params.opening_distance_min_m;
}

SideOpeningObservation DetectFirstSideOpening(
    const SelectedCircleOpeningRows& selected,
    BoundarySide side,
    const CircleV2Params& params) {
    SideOpeningObservation opening{};
    opening.side = side;
    const std::vector<RowObservation>& rows = selected.rows;

    for (std::size_t candidate = 2U; candidate < rows.size(); ++candidate) {
        std::size_t baseline_begin = candidate;
        while (baseline_begin > 0U) {
            const std::size_t previous = baseline_begin - 1U;
            if (!SideObserved(rows[previous], side)) {
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
            baseline_trace.push_back(
                {rows[index].forward_m, SideLateral(rows[index], side)});
        }
        const LineFit baseline = FitLine(baseline_trace, side, params);
        if (!baseline.valid) {
            continue;
        }

        float effective_lateral = 0.0F;
        float outward_distance = 0.0F;
        CircleOpeningSource source = CircleOpeningSource::kNone;
        if (!OpeningRowMatches(rows[candidate],
                               side,
                               baseline,
                               params,
                               effective_lateral,
                               source,
                               outward_distance)) {
            continue;
        }

        std::size_t confirmed_end = candidate;
        float minimum_white_width = rows[candidate].white_width_m;
        CircleOpeningSource aggregate_source = source;
        for (std::size_t index = candidate + 1U; index < rows.size(); ++index) {
            if (rows[index].forward_m - rows[index - 1U].forward_m >
                params.max_adjacent_distance_m) {
                break;
            }
            float confirm_lateral = 0.0F;
            float confirm_distance = 0.0F;
            CircleOpeningSource confirm_source = CircleOpeningSource::kNone;
            if (!OpeningRowMatches(rows[index],
                                   side,
                                   baseline,
                                   params,
                                   confirm_lateral,
                                   confirm_source,
                                   confirm_distance)) {
                break;
            }
            confirmed_end = index;
            minimum_white_width =
                std::min(minimum_white_width, rows[index].white_width_m);
            if (confirm_source == CircleOpeningSource::kFovEdgeLowerBound) {
                aggregate_source = CircleOpeningSource::kFovEdgeLowerBound;
            }
            if (rows[confirmed_end].forward_m - rows[candidate].forward_m >=
                params.opening_confirm_forward_span_m) {
                break;
            }
        }
        if (rows[confirmed_end].forward_m - rows[candidate].forward_m <
            params.opening_confirm_forward_span_m) {
            continue;
        }

        opening.fact.available = true;
        opening.fact.begin_forward_m = rows[candidate].forward_m;
        opening.fact.end_forward_m = rows[confirmed_end].forward_m;
        opening.fact.effective_lateral_m = effective_lateral;
        opening.fact.source = aggregate_source;
        opening.fact.outward_distance_m = outward_distance;
        opening.fact.minimum_white_width_m = minimum_white_width;
        opening.fact.origin_connected = true;
        opening.opening_range = MakeInterval(rows, candidate, confirmed_end);
        opening.baseline_support_range =
            MakeInterval(rows, baseline_begin, candidate - 1U);
        return opening;
    }
    return opening;
}

bool IntervalsOverlap(const ForwardInterval& left,
                      const ForwardInterval& right) {
    return left.begin_row_index <= right.end_row_index &&
           right.begin_row_index <= left.end_row_index;
}

BoundaryStraightObservation ObserveOppositeBoundaryStraightness(
    const SelectedCircleOpeningRows& selected,
    const SideOpeningObservation& opening,
    const CircleV2Params& params) {
    BoundaryStraightObservation observation{};
    const BoundarySide opposite = Opposite(opening.side);
    const std::vector<RowObservation>& rows = selected.rows;
    const std::size_t begin = opening.baseline_support_range.begin_row_index;
    const std::size_t end = opening.opening_range.end_row_index;
    if (!opening.fact.available || begin > end || end >= rows.size()) {
        return observation;
    }

    std::vector<TracePoint> trace;
    trace.reserve(end - begin + 1U);
    for (std::size_t index = begin; index <= end; ++index) {
        if (!SideObserved(rows[index], opposite)) {
            return observation;
        }
        if (index > begin &&
            rows[index].forward_m - rows[index - 1U].forward_m >
                params.max_adjacent_distance_m) {
            return observation;
        }
        trace.push_back(
            {rows[index].forward_m, SideLateral(rows[index], opposite)});
    }
    observation.observable = trace.size() >= 2U;
    const LineFit fit = FitLine(trace, opposite, params);
    observation.confidence = fit.confidence;
    observation.straight =
        fit.valid && fit.straight &&
        fit.confidence >= params.opposite_straight_confidence_min;
    return observation;
}

}  // namespace

CircleEntryCueObservation ObserveCircleEntryCue(const SceneFrameView& frame,
                                                const CircleV2Params& params) {
    LS2K_PERF_SCOPE(port::PerfStage::kCirclePhase1Rows);
    CircleEntryCueObservation observation{};
    const SelectedCircleOpeningRows rows =
        SelectCircleOpeningRows(frame, params);
    observation.left_opening =
        DetectFirstSideOpening(rows, BoundarySide::kLeft, params);
    observation.right_opening =
        DetectFirstSideOpening(rows, BoundarySide::kRight, params);
    const bool left_available = observation.left_opening.fact.available;
    const bool right_available = observation.right_opening.fact.available;
    if (left_available && right_available &&
        IntervalsOverlap(observation.left_opening.opening_range,
                         observation.right_opening.opening_range)) {
        observation.bilateral_overlap = true;
        return observation;
    }

    const SideOpeningObservation* selected = nullptr;
    if (left_available && right_available) {
        selected = observation.left_opening.fact.begin_forward_m <
                           observation.right_opening.fact.begin_forward_m
                       ? &observation.left_opening
                       : &observation.right_opening;
    } else if (left_available) {
        selected = &observation.left_opening;
    } else if (right_available) {
        selected = &observation.right_opening;
    }
    if (selected == nullptr) {
        return observation;
    }

    const BoundaryStraightObservation opposite =
        ObserveOppositeBoundaryStraightness(rows, *selected, params);
    observation.selected_opposite_boundary = opposite;
    if (!opposite.observable || !opposite.straight) {
        return observation;
    }
    observation.selected = true;
    observation.selected_opening = *selected;
    observation.detected_dir = DirectionFor(selected->side);
    return observation;
}

}  // namespace ls2k::vision::detail
