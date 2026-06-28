#include "vision/bev/bev_boundary_row.hpp"

#include <cmath>

namespace ls2k::vision {

void ExtractSparseBoundaryRowFacts(const std::vector<BEVRowLumaSample>& samples,
                                   const port::BEVBoundaryParameters& params,
                                   float min_span_width_m,
                                   BEVSimpleRowScan& row) {
    row.jumps.clear();
    row.spans.clear();
    if (!port::IsValidBEVBoundaryParameters(params) || samples.size() < 2U) {
        return;
    }

    for (std::size_t index = 1U; index < samples.size(); ++index) {
        const BEVRowLumaSample& previous = samples[index - 1U];
        const BEVRowLumaSample& current = samples[index];
        if (!previous.sampleable || !current.sampleable) {
            continue;
        }
        const int delta_y = static_cast<int>(current.y) - static_cast<int>(previous.y);
        if (std::abs(delta_y) < params.local_jump_min_y) {
            continue;
        }
        BEVBoundaryJump jump{};
        jump.forward_m = current.forward_m;
        jump.lateral_m = 0.5F * (previous.lateral_m + current.lateral_m);
        jump.lateral_index = current.lateral_index;
        jump.delta_y = delta_y;
        jump.polarity = delta_y > 0 ? BEVBoundaryJumpPolarity::kRisingY
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
}

}  // namespace ls2k::vision
