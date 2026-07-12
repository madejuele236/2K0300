#include "vision/elements/cross_exit_element_evidence.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

namespace ls2k::vision {
namespace {

/// 十字出口检测的最小连续宽行数
constexpr std::size_t kCrossMinContiguousWideRows = 3U;
/// 十字出口游程累加器，累计连续宽行行的统计数据和得分
struct CrossRunAccumulator {
    std::size_t row_count = 0;         ///< 累加的行数
    float forward_min_m = 0.0F;        ///< 前向距离最小值（米）
    float forward_max_m = 0.0F;        ///< 前向距离最大值（米）
    float lateral_min_m = 0.0F;        ///< 横向最小值（米）
    float lateral_max_m = 0.0F;        ///< 横向最大值（米）
    std::size_t sampleable_count = 0;         ///< 总计可采样数
    std::size_t boundary_jump_count = 0;      ///< 总计边界跳变数
    std::size_t boundary_span_count = 0;      ///< 总计边界 span 数
};

bool BoundaryAbsentRow(const BEVSimpleRowScan& row, std::size_t min_sampleable_per_row) {
    return row.valid &&
           row.sampleable_count >= min_sampleable_per_row &&
           row.jumps.empty() &&
           row.spans.empty();
}

/// 将一行的数据累加到十字出口运行累加器中
void AddBoundaryAbsentRow(CrossRunAccumulator& run, const BEVSimpleRowScan& row) {
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

/// 比较两个累加器，判断哪个更优（行数优先，评分其次）
bool BetterRun(const CrossRunAccumulator& candidate, const CrossRunAccumulator& best) {
    return candidate.row_count > best.row_count;
}

/// 检查参考路径是否有有效的前导视觉参考（无间隙、无无限值、首个采样点存在）
bool HasLeadingVisualReference(const port::BEVReferencePath& reference) {
    if (reference.mode != port::ReferenceMode::kIntervalCenter ||
        !reference.sampled_path[0].present) {
        return false;
    }
    bool gap_seen = false;
    for (const port::BEVPathSample& sample : reference.sampled_path) {
        if (!sample.present) {
            gap_seen = true;
            continue;
        }
        if (gap_seen ||
            !std::isfinite(sample.point.forward_m) ||
            !std::isfinite(sample.point.lateral_m)) {
            return false;
        }
    }
    return true;
}

}  // namespace

/// DetectCrossExitEvidence 实现
/// 检测十字出口元素证据：
/// 1. 遍历 sparse boundary rows，识别连续无边界事实行
/// 2. 将连续 absence 行分组为运行累加器
/// 3. 根据连续 absence 行数生成十字出口证据
port::CrossExitElementEvidence DetectCrossExitEvidence(
    const std::vector<BEVSimpleRowScan>& rows,
    const port::RuntimeParameters& params) {
    port::CrossExitElementEvidence evidence{};
    evidence.reason = "no_sparse_rows";
    if (rows.empty()) {
        return evidence;
    }

    bool saw_supported_row = false;
    CrossRunAccumulator current{};
    CrossRunAccumulator best{};
    const std::size_t min_sampleable_per_row =
        static_cast<std::size_t>(params.bev_element.cross_min_sampleable_per_row);

    const auto finish_run = [&current, &best]() {
        if (current.row_count > 0U && BetterRun(current, best)) {
            best = current;
        }
        current = {};
    };

    for (const BEVSimpleRowScan& row : rows) {
        evidence.sampleable_count += row.sampleable_count;
        evidence.boundary_jump_count += row.jumps.size();
        evidence.boundary_span_count += row.spans.size();

        const bool supported = row.valid && row.sampleable_count >= min_sampleable_per_row;
        saw_supported_row = saw_supported_row || supported;
        if (!supported) {
            finish_run();
            continue;
        }

        if (!BoundaryAbsentRow(row, min_sampleable_per_row)) {
            finish_run();
            continue;
        }
        AddBoundaryAbsentRow(current, row);
    }
    finish_run();

    if (!saw_supported_row) {
        evidence.reason = "insufficient_sampleable_support";
        return evidence;
    }
    evidence.boundary_absent_row_count = best.row_count;
    if (best.row_count < kCrossMinContiguousWideRows) {
        evidence.reason = "boundary_absence_rows_absent";
        return evidence;
    }

    evidence.forward_min_m = best.forward_min_m;
    evidence.forward_max_m = best.forward_max_m;
    evidence.lateral_min_m = best.lateral_min_m;
    evidence.lateral_max_m = best.lateral_max_m;
    evidence.sampleable_count = best.sampleable_count;
    evidence.boundary_jump_count = best.boundary_jump_count;
    evidence.boundary_span_count = best.boundary_span_count;
    evidence.present = true;
    evidence.reason = "present";
    return evidence;
}

/// BuildCrossExitVisualReferenceCandidate 实现
/// 从十字出口证据和车道线候选构建视觉参考候选
/// 直接继承车道线候选的参考路径（十字出口不改变路径方向）
port::VisualReferenceCandidate BuildCrossExitVisualReferenceCandidate(
    const port::CrossExitElementEvidence& evidence,
    const port::VisualReferenceCandidate& line_candidate,
    const port::RuntimeParameters& params,
    port::VisualElementCandidateSummary& summary) {
    port::VisualReferenceCandidate candidate{};
    summary = {};
    summary.takeover_enabled = params.bev_element.cross_exit_takeover_enabled;
    if (!evidence.present) {
        summary.reason = evidence.reason.empty() ? "evidence_absent" : evidence.reason;
        return candidate;
    }
    if (!line_candidate.present || !HasLeadingVisualReference(line_candidate.reference_path)) {
        summary.reason = "line_candidate_absent";
        return candidate;
    }

    candidate.present = true;
    candidate.kind = port::VisualReferenceCandidateKind::kCrossExit;
    candidate.reference_path = line_candidate.reference_path;
    candidate.source = "cross_exit";
    candidate.reason = "cross_exit_evidence_candidate";

    summary.built = true;
    summary.included_in_arbitration = summary.takeover_enabled;
    summary.reason = summary.included_in_arbitration ? "included_in_arbitration"
                                                     : "takeover_disabled";
    return candidate;
}

}  // namespace ls2k::vision
