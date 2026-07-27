#include "vision/elements/cross_exit_element_evidence.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace ls2k::vision {
namespace {

/// 十字出口检测的最小连续开口行数
constexpr std::size_t kCrossMinContiguousOpenRows = 3U;
constexpr float kGeometryEpsilon = 1.0e-6F;
/// 十字出口游程累加器，累计连续开口行的统计数据
struct CrossRunAccumulator {
    std::size_t row_count = 0;             ///< 累加的行数
    float forward_min_m = 0.0F;            ///< 前向距离最小值（米）
    float forward_max_m = 0.0F;            ///< 前向距离最大值（米）
    float lateral_min_m = 0.0F;            ///< 横向最小值（米）
    float lateral_max_m = 0.0F;            ///< 横向最大值（米）
    std::size_t sampleable_count = 0;      ///< 总计可采样数
    std::size_t boundary_jump_count = 0;   ///< 总计边界跳变数
    std::size_t boundary_span_count = 0;   ///< 总计边界 span 数
};

void AddCrossOpenRow(CrossRunAccumulator& run, const BEVSimpleRowScan& row);
bool BetterRun(const CrossRunAccumulator& candidate, const CrossRunAccumulator& best);

struct ConnectedRunObservation {
    const BEVSimpleRowScan* row = nullptr;
    const BEVWhiteRun* run = nullptr;
};

struct MetricLine {
    bool valid = false;
    float slope = 0.0F;
    float intercept = 0.0F;
};

struct CrossOpeningObservation {
    bool present = false;
    const char* reason = "boundary_absence_rows_absent";
    CrossRunAccumulator support{};
};

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

std::vector<ConnectedRunObservation> CollectConnectedRuns(
    const std::vector<BEVSimpleRowScan>& rows,
    std::size_t min_sampleable_per_row) {
    std::vector<ConnectedRunObservation> observations;
    observations.reserve(rows.size());
    for (const BEVSimpleRowScan& row : rows) {
        const bool supported = row.valid && row.sampleable_count >= min_sampleable_per_row;
        observations.push_back(
            ConnectedRunObservation{&row, supported ? UniqueOriginConnectedRun(row) : nullptr});
    }
    return observations;
}

bool SideObserved(const ConnectedRunObservation& observation, bool left) {
    if (observation.run == nullptr) {
        return false;
    }
    const BEVWhiteRunEndpointState endpoint =
        left ? observation.run->left_endpoint : observation.run->right_endpoint;
    return endpoint == BEVWhiteRunEndpointState::kBoundary;
}

bool OppositeAtFov(const ConnectedRunObservation& observation, bool left) {
    if (observation.run == nullptr) {
        return false;
    }
    const BEVWhiteRunEndpointState endpoint =
        left ? observation.run->right_endpoint : observation.run->left_endpoint;
    return endpoint == BEVWhiteRunEndpointState::kFovEdge;
}

float SideLateral(const ConnectedRunObservation& observation, bool left) {
    return left ? observation.run->left_m : observation.run->right_m;
}

float PointDistance(const ConnectedRunObservation& lhs,
                    const ConnectedRunObservation& rhs,
                    bool left) {
    return std::hypot(rhs.row->forward_m - lhs.row->forward_m,
                      SideLateral(rhs, left) - SideLateral(lhs, left));
}

MetricLine FitBoundaryLine(const std::vector<ConnectedRunObservation>& observations,
                           std::size_t begin,
                           std::size_t end,
                           bool left) {
    MetricLine line{};
    if (end - begin < 2U) {
        return line;
    }

    float weight_sum = 0.0F;
    float mean_forward = 0.0F;
    float mean_lateral = 0.0F;
    const auto weight_at = [&observations, begin, end](std::size_t index) {
        if (end - begin == 2U) {
            return 1.0F;
        }
        const float left_gap =
            index == begin
                ? observations[index + 1U].row->forward_m -
                      observations[index].row->forward_m
                : observations[index].row->forward_m -
                      observations[index - 1U].row->forward_m;
        const float right_gap =
            index + 1U == end
                ? observations[index].row->forward_m -
                      observations[index - 1U].row->forward_m
                : observations[index + 1U].row->forward_m -
                      observations[index].row->forward_m;
        return std::max(kGeometryEpsilon, 0.5F * (left_gap + right_gap));
    };
    for (std::size_t index = begin; index < end; ++index) {
        const float weight = weight_at(index);
        weight_sum += weight;
        mean_forward += weight * observations[index].row->forward_m;
        mean_lateral += weight * SideLateral(observations[index], left);
    }
    if (weight_sum <= kGeometryEpsilon) {
        return line;
    }
    mean_forward /= weight_sum;
    mean_lateral /= weight_sum;

    float forward_variance = 0.0F;
    float covariance = 0.0F;
    for (std::size_t index = begin; index < end; ++index) {
        const float weight = weight_at(index);
        const float forward_delta = observations[index].row->forward_m - mean_forward;
        forward_variance += weight * forward_delta * forward_delta;
        covariance += weight * forward_delta *
                      (SideLateral(observations[index], left) - mean_lateral);
    }
    if (forward_variance <= kGeometryEpsilon) {
        return line;
    }
    line.slope = covariance / forward_variance;
    line.intercept = mean_lateral - line.slope * mean_forward;
    line.valid = std::isfinite(line.slope) && std::isfinite(line.intercept);
    return line;
}

float DirectedOutwardDistance(const MetricLine& baseline,
                              const ConnectedRunObservation& observation,
                              bool left) {
    const float predicted = baseline.slope * observation.row->forward_m +
                            baseline.intercept;
    const float normalizer = std::sqrt(1.0F + baseline.slope * baseline.slope);
    return left ? (predicted - SideLateral(observation, left)) / normalizer
                : (SideLateral(observation, left) - predicted) / normalizer;
}

CrossOpeningObservation ObserveSideExpansion(
    const std::vector<ConnectedRunObservation>& observations,
    bool left,
    const port::RuntimeParameters& params) {
    CrossOpeningObservation opening{};
    const float max_adjacent_distance_m =
        params.bev_geometry.boundary_trace_max_adjacent_distance_m;
    const float expansion_min_m = params.bev_element.cross_boundary_expansion_min_m;

    for (std::size_t candidate = 2U; candidate < observations.size(); ++candidate) {
        if (!SideObserved(observations[candidate], left) ||
            !OppositeAtFov(observations[candidate], left)) {
            continue;
        }

        std::size_t baseline_begin = candidate;
        while (baseline_begin > 0U) {
            const std::size_t previous = baseline_begin - 1U;
            if (!SideObserved(observations[previous], left)) {
                break;
            }
            if (baseline_begin < candidate &&
                PointDistance(observations[previous],
                              observations[baseline_begin],
                              left) > max_adjacent_distance_m) {
                break;
            }
            baseline_begin = previous;
        }
        if (candidate - baseline_begin < 2U ||
            observations[candidate].row->forward_m -
                    observations[candidate - 1U].row->forward_m >
                max_adjacent_distance_m) {
            continue;
        }

        const MetricLine baseline =
            FitBoundaryLine(observations, baseline_begin, candidate, left);
        if (!baseline.valid ||
            DirectedOutwardDistance(baseline, observations[candidate], left) <
                expansion_min_m) {
            continue;
        }

        opening.present = true;
        opening.reason = left ? "present_left_boundary_expansion"
                              : "present_right_boundary_expansion";
        opening.support.forward_min_m = observations[candidate].row->forward_m;
        opening.support.forward_max_m = observations[candidate].row->forward_m;
        opening.support.lateral_min_m = observations[candidate].run->left_m;
        opening.support.lateral_max_m = observations[candidate].run->right_m;
        opening.support.row_count = 1U;
        opening.support.sampleable_count = observations[candidate].row->sampleable_count;
        opening.support.boundary_jump_count = observations[candidate].row->jumps.size();
        opening.support.boundary_span_count = observations[candidate].row->spans.size();
        return opening;
    }
    return opening;
}

bool CrossOpenRow(const BEVSimpleRowScan& row, std::size_t min_sampleable_per_row) {
    if (!row.valid || row.sampleable_count < min_sampleable_per_row) {
        return false;
    }
    const BEVWhiteRun* connected_run = UniqueOriginConnectedRun(row);
    // One FOV endpoint is the normal single-boundary/lost-boundary contract.
    // A Cross opening is established only when the connected white run reaches
    // both lateral FOV edges.
    return connected_run != nullptr &&
           connected_run->left_endpoint == BEVWhiteRunEndpointState::kFovEdge &&
           connected_run->right_endpoint == BEVWhiteRunEndpointState::kFovEdge;
}

CrossOpeningObservation ObserveFullFovOpening(
    const std::vector<BEVSimpleRowScan>& rows,
    std::size_t min_sampleable_per_row) {
    CrossRunAccumulator current{};
    CrossRunAccumulator best{};
    const auto finish_run = [&current, &best]() {
        if (current.row_count > 0U && BetterRun(current, best)) {
            best = current;
        }
        current = {};
    };
    for (const BEVSimpleRowScan& row : rows) {
        if (!CrossOpenRow(row, min_sampleable_per_row)) {
            finish_run();
            continue;
        }
        AddCrossOpenRow(current, row);
    }
    finish_run();

    CrossOpeningObservation opening{};
    opening.support = best;
    if (best.row_count >= kCrossMinContiguousOpenRows) {
        opening.present = true;
        opening.reason = "present_full_fov";
    }
    return opening;
}

/// 将一行的数据累加到十字出口运行累加器中
void AddCrossOpenRow(CrossRunAccumulator& run, const BEVSimpleRowScan& row) {
    if (run.row_count == 0U) {
        run.forward_min_m = row.forward_m;
        run.forward_max_m = row.forward_m;
        run.lateral_min_m = row.sampleable_left_m;
        run.lateral_max_m = row.sampleable_right_m;
    } else {
        run.forward_min_m = std::min(run.forward_min_m, row.forward_m);
        run.forward_max_m = std::max(run.forward_max_m, row.forward_m);
        run.lateral_min_m = std::min(run.lateral_min_m, row.sampleable_left_m);
        run.lateral_max_m = std::max(run.lateral_max_m, row.sampleable_right_m);
    }
    ++run.row_count;
    run.sampleable_count += row.sampleable_count;
    run.boundary_jump_count += row.jumps.size();
    run.boundary_span_count += row.spans.size();
}

/// 比较两个连续开口区间，保留支持行数更多的区间。
bool BetterRun(const CrossRunAccumulator& candidate, const CrossRunAccumulator& best) {
    return candidate.row_count > best.row_count;
}

}  // namespace

/// DetectCrossExitEvidence 实现
/// 检测当前帧的十字开口证据：
/// 1. 连通白区连续到达两侧 FOV；或
/// 2. 一侧真实边界相对同侧历史直线向外扩张，且对侧到达 FOV。
/// 两种证据都使用同一个原点连通性门禁，不引入跨帧状态。
port::CrossExitElementEvidence DetectCrossExitEvidence(
    const std::vector<BEVSimpleRowScan>& rows,
    const BEVSegmentConnectivityResult& origin_to_cross_sample_midpoint_connectivity,
    const port::RuntimeParameters& params) {
    port::CrossExitElementEvidence evidence{};
    evidence.reason = "no_sparse_rows";
    if (rows.empty()) {
        return evidence;
    }

    bool saw_supported_row = false;
    const std::size_t min_sampleable_per_row =
        static_cast<std::size_t>(params.bev_element.cross_min_sampleable_per_row);

    for (const BEVSimpleRowScan& row : rows) {
        evidence.sampleable_count += row.sampleable_count;
        evidence.boundary_jump_count += row.jumps.size();
        evidence.boundary_span_count += row.spans.size();

        saw_supported_row = saw_supported_row ||
                            (row.valid && row.sampleable_count >= min_sampleable_per_row);
    }

    CrossOpeningObservation opening =
        ObserveFullFovOpening(rows, min_sampleable_per_row);
    if (!opening.present) {
        const std::vector<ConnectedRunObservation> observations =
            CollectConnectedRuns(rows, min_sampleable_per_row);
        const CrossOpeningObservation left = ObserveSideExpansion(observations, true, params);
        const CrossOpeningObservation right = ObserveSideExpansion(observations, false, params);
        if (left.present && right.present) {
            opening = left.support.forward_min_m <= right.support.forward_min_m ? left : right;
        } else if (left.present) {
            opening = left;
        } else if (right.present) {
            opening = right;
        }
    }

    if (!saw_supported_row) {
        evidence.reason = "insufficient_sampleable_support";
        return evidence;
    }
    evidence.boundary_absent_row_count = opening.support.row_count;
    if (!opening.present) {
        evidence.reason = "boundary_absence_rows_absent";
        return evidence;
    }
    if (origin_to_cross_sample_midpoint_connectivity.status ==
        BEVSegmentConnectivityStatus::kBlocked) {
        evidence.reason = "origin_to_cross_sample_midpoint_blocked";
        return evidence;
    }
    if (origin_to_cross_sample_midpoint_connectivity.status ==
        BEVSegmentConnectivityStatus::kUnobservable) {
        evidence.reason = "origin_to_cross_sample_midpoint_unobservable";
        return evidence;
    }

    evidence.forward_min_m = opening.support.forward_min_m;
    evidence.forward_max_m = opening.support.forward_max_m;
    evidence.lateral_min_m = opening.support.lateral_min_m;
    evidence.lateral_max_m = opening.support.lateral_max_m;
    evidence.sampleable_count = opening.support.sampleable_count;
    evidence.boundary_jump_count = opening.support.boundary_jump_count;
    evidence.boundary_span_count = opening.support.boundary_span_count;
    evidence.present = true;
    evidence.reason = opening.reason;
    return evidence;
}

}  // namespace ls2k::vision
