#include "vision/bev/bev_boundary_row.hpp"

namespace ls2k::vision {

void ExtractSparseBoundaryRowFacts(const std::vector<BEVRowLumaSample>& samples,
                                   float min_span_width_m,
                                   BEVSimpleRowScan& row) {
    row.jumps.clear();
    row.spans.clear();
    row.white_runs.clear();
    if (samples.empty()) {
        return;
    }

    for (std::size_t index = 1U; index < samples.size(); ++index) {
        const BEVRowLumaSample& previous = samples[index - 1U];
        const BEVRowLumaSample& current = samples[index];
        if (!previous.sampleable || !current.sampleable ||
            !previous.classified || !current.classified ||
            current.lateral_index != previous.lateral_index + 1) {
            continue;
        }
        const int delta_y = static_cast<int>(current.y) - static_cast<int>(previous.y);
        if (previous.white == current.white) {
            continue;
        }
        BEVBoundaryJump jump{};
        jump.forward_m = current.forward_m;
        jump.lateral_m = 0.5F * (previous.lateral_m + current.lateral_m);
        jump.lateral_index = current.lateral_index;
        jump.delta_y = delta_y;
        jump.polarity = current.white ? BEVBoundaryJumpPolarity::kRisingY
                                      : BEVBoundaryJumpPolarity::kFallingY;
        row.jumps.push_back(jump);
    }

    for (std::size_t index = 1U; index < row.jumps.size(); ++index) {
        const BEVBoundaryJump& left = row.jumps[index - 1U];
        const BEVBoundaryJump& right = row.jumps[index];
        if (left.polarity != BEVBoundaryJumpPolarity::kRisingY ||
            right.polarity != BEVBoundaryJumpPolarity::kFallingY ||
            right.lateral_m < left.lateral_m) {
            continue;
        }
        const float width = right.lateral_m - left.lateral_m;
        if (width < min_span_width_m) {
            continue;
        }
        BEVBoundarySpan span{};
        span.forward_m = row.forward_m;
        span.left_m = left.lateral_m;
        span.right_m = right.lateral_m;
        span.center_m = 0.5F * (left.lateral_m + right.lateral_m);
        span.width_m = width;
        span.left_lateral_index = left.lateral_index;
        span.right_lateral_index = right.lateral_index;
        row.spans.push_back(span);
    }

    std::size_t index = 0U;
    while (index < samples.size()) {
        if (!samples[index].sampleable || !samples[index].classified ||
            !samples[index].white) {
            ++index;
            continue;
        }
        const std::size_t begin = index;
        std::size_t end = index;
        while (end + 1U < samples.size() &&
               samples[end + 1U].sampleable &&
               samples[end + 1U].classified &&
               samples[end + 1U].lateral_index == samples[end].lateral_index + 1 &&
               samples[end + 1U].white) {
            ++end;
        }

        BEVWhiteRun run{};
        run.forward_m = samples[begin].forward_m;
        run.left_lateral_index = samples[begin].lateral_index;
        run.right_lateral_index = samples[end].lateral_index;
        run.left_m = samples[begin].lateral_m;
        run.right_m = samples[end].lateral_m;

        if (begin == 0U) {
            run.left_endpoint = BEVWhiteRunEndpointState::kFovEdge;
        } else if (samples[begin - 1U].sampleable &&
                   samples[begin - 1U].classified &&
                   samples[begin].lateral_index == samples[begin - 1U].lateral_index + 1 &&
                   !samples[begin - 1U].white) {
            run.left_endpoint = BEVWhiteRunEndpointState::kBoundary;
            run.left_m = 0.5F * (samples[begin - 1U].lateral_m + samples[begin].lateral_m);
            for (std::size_t jump_index = 0U; jump_index < row.jumps.size(); ++jump_index) {
                const BEVBoundaryJump& jump = row.jumps[jump_index];
                if (jump.lateral_index == samples[begin].lateral_index &&
                    jump.polarity == BEVBoundaryJumpPolarity::kRisingY) {
                    run.left_jump_index = static_cast<int>(jump_index);
                    break;
                }
            }
        }

        if (end + 1U == samples.size()) {
            run.right_endpoint = BEVWhiteRunEndpointState::kFovEdge;
        } else if (samples[end + 1U].sampleable &&
                   samples[end + 1U].classified &&
                   samples[end + 1U].lateral_index == samples[end].lateral_index + 1 &&
                   !samples[end + 1U].white) {
            run.right_endpoint = BEVWhiteRunEndpointState::kBoundary;
            run.right_m = 0.5F * (samples[end].lateral_m + samples[end + 1U].lateral_m);
            for (std::size_t jump_index = 0U; jump_index < row.jumps.size(); ++jump_index) {
                const BEVBoundaryJump& jump = row.jumps[jump_index];
                if (jump.lateral_index == samples[end + 1U].lateral_index &&
                    jump.polarity == BEVBoundaryJumpPolarity::kFallingY) {
                    run.right_jump_index = static_cast<int>(jump_index);
                    break;
                }
            }
        }
        row.white_runs.push_back(run);
        index = end + 1U;
    }
}

}  // namespace ls2k::vision
