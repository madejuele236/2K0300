#include "reference/visual_reference_orchestration.hpp"

#include "port/bev_reference_path_utils.hpp"

namespace ls2k::reference {
namespace {

/// 候选验证结果
struct CandidateValidation {
    bool accepted = false;                ///< 候选是否通过验证
    const char* rejected_reason = "none"; ///< 拒绝原因
};

/// 判断候选类型是否为特殊类型（非车道线）
bool IsSpecialKind(port::VisualReferenceCandidateKind kind) {
    return kind != port::VisualReferenceCandidateKind::kLine;
}

/// 获取候选类型的优先级（数值越高优先级越高）
int Priority(port::VisualReferenceCandidateKind kind) {
    switch (kind) {
        case port::VisualReferenceCandidateKind::kRoadblockBypass:
            return 50;
        case port::VisualReferenceCandidateKind::kCrossExit:
            return 45;
        case port::VisualReferenceCandidateKind::kCircleLeft:
        case port::VisualReferenceCandidateKind::kCircleRight:
            return 40;
        case port::VisualReferenceCandidateKind::kMlGrounded:
            return 20;
        case port::VisualReferenceCandidateKind::kLine:
            return 10;
    }
    return 0;
}

/// 验证候选是否有效。单个无效槽不代表整条候选无效；选择时会统一紧凑化。
CandidateValidation ValidateCandidate(const port::VisualReferenceCandidate& candidate) {
    if (!candidate.present) {
        return {};
    }
    switch (candidate.reference_path.mode) {
        case port::ReferenceMode::kIntervalCenter:
        case port::ReferenceMode::kMlObservedBoundary:
        case port::ReferenceMode::kMlBoundaryOffset:
            break;
        case port::ReferenceMode::kHoldLast:
            return {false, "hold_candidate_not_visual"};
        case port::ReferenceMode::kNone:
            return {false, "none_candidate_not_visual"};
    }

    if (port::CountFiniteReferenceSamples(candidate.reference_path) == 0U) {
        return {false, "reference_candidate_has_no_valid_samples"};
    }
    return {true, "none"};
}

/// 选中一个候选并将其设置到选择结果中
void SelectCandidate(port::VisualReferenceSelection& selection,
                     const port::VisualReferenceCandidate& candidate,
                     const char* reason) {
    selection.present = true;
    selection.kind_valid = true;
    selection.kind = candidate.kind;
    selection.reference_path = candidate.reference_path;
    port::CompactFiniteReferenceSamples(selection.reference_path);
    selection.source = candidate.source.empty() ? ToString(candidate.kind) : candidate.source;
    selection.reason = reason;
}

}  // namespace

/// ToString 实现
/// 将VisualReferenceCandidateKind枚举值转换为可读字符串
const char* ToString(port::VisualReferenceCandidateKind kind) {
    switch (kind) {
        case port::VisualReferenceCandidateKind::kLine:
            return "line";
        case port::VisualReferenceCandidateKind::kCrossExit:
            return "cross_exit";
        case port::VisualReferenceCandidateKind::kCircleLeft:
            return "circle_left";
        case port::VisualReferenceCandidateKind::kCircleRight:
            return "circle_right";
        case port::VisualReferenceCandidateKind::kRoadblockBypass:
            return "roadblock_bypass";
        case port::VisualReferenceCandidateKind::kMlGrounded:
            return "ml_grounded";
    }
    return "line";
}

/// MakeLineVisualReferenceCandidate 实现
/// 从BEV参考路径创建基础的车道线视觉参考候选
port::VisualReferenceCandidate MakeLineVisualReferenceCandidate(
    const port::BEVReferencePath& reference_path,
    const std::string& source) {
    port::VisualReferenceCandidate candidate{};
    candidate.present = port::CountFiniteReferenceSamples(reference_path) > 0U;
    candidate.kind = port::VisualReferenceCandidateKind::kLine;
    candidate.reference_path = reference_path;
    candidate.source = source.empty() ? "line" : source;
    candidate.reason = candidate.present ? "line_reference_candidate" : "line_reference_absent";
    return candidate;
}

/// SelectVisualReference 实现
/// 从候选列表中选择最佳视觉参考：
/// 1. 验证每个候选的有效性
/// 2. 优先选择特殊候选（按优先级）
/// 3. 若无特殊候选则选择最佳车道线候选
/// 4. 若特殊候选出现同优先级冲突则返回ambiguous
port::VisualReferenceSelection SelectVisualReference(
    const port::VisualReferenceCandidate* candidates,
    std::size_t count) {
    port::VisualReferenceSelection selection{};
    const port::VisualReferenceCandidate* best_line = nullptr;
    const port::VisualReferenceCandidate* best_special = nullptr;
    int best_special_priority = -1;
    bool best_special_tied = false;

    for (std::size_t index = 0; index < count; ++index) {
        const port::VisualReferenceCandidate& candidate = candidates[index];
        if (!candidate.present) {
            continue;
        }
        const CandidateValidation validation = ValidateCandidate(candidate);
        if (!validation.accepted) {
            if (selection.rejected_candidate_reason == "none") {
                selection.rejected_candidate_reason = validation.rejected_reason;
            }
            continue;
        }

        ++selection.candidate_count;
        if (candidate.kind == port::VisualReferenceCandidateKind::kLine) {
            if (best_line == nullptr) {
                best_line = &candidate;
            }
            continue;
        }

        if (!IsSpecialKind(candidate.kind)) {
            continue;
        }
        const int priority = Priority(candidate.kind);
        if (best_special == nullptr || priority > best_special_priority) {
            best_special = &candidate;
            best_special_priority = priority;
            best_special_tied = false;
            continue;
        }
        if (priority < best_special_priority) {
            continue;
        }
        best_special_tied = true;
    }

    if (best_special_tied) {
        selection.present = false;
        selection.reason = "ambiguous_visual_reference_candidates";
        selection.source = "none";
        return selection;
    }
    if (best_special != nullptr) {
        SelectCandidate(selection, *best_special, "special_visual_candidate_selected");
        return selection;
    }
    if (best_line != nullptr) {
        SelectCandidate(selection, *best_line, "line_candidate_selected");
        return selection;
    }
    if (selection.rejected_candidate_reason != "none") {
        selection.reason = "no_valid_visual_reference_candidate";
    }
    return selection;
}

port::VisualReferenceSelection SelectVisualReference(
    const std::vector<port::VisualReferenceCandidate>& candidates) {
    return SelectVisualReference(candidates.data(), candidates.size());
}

}  // namespace ls2k::reference
