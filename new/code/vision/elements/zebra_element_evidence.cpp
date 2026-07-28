#include "vision/elements/zebra_element_evidence.hpp"

#include <algorithm>
#include <cstddef>

namespace ls2k::vision {
namespace {

struct StripeRowCandidate {
    bool present = false;
    float forward_m = 0.0F;
    float lateral_min_m = 0.0F;
    float lateral_max_m = 0.0F;
    std::size_t jump_count = 0U;
    std::size_t sampleable_count = 0U;
};

struct StripeBand {
    bool present = false;
    float forward_min_m = 0.0F;
    float forward_max_m = 0.0F;
    float lateral_min_m = 0.0F;
    float lateral_max_m = 0.0F;
    std::size_t jump_count = 0U;
    std::size_t sampleable_count = 0U;
};

bool OppositePolarity(const BEVBoundaryJump& lhs,
                      const BEVBoundaryJump& rhs) {
    return lhs.polarity != rhs.polarity;
}

StripeRowCandidate FindStripeRow(const BEVSimpleRowScan& row,
                                 float lateral_window_width_m,
                                 std::size_t minimum_jump_count) {
    StripeRowCandidate best{};
    if (!row.valid || row.jumps.size() < minimum_jump_count) {
        return best;
    }

    for (std::size_t begin = 0U; begin < row.jumps.size(); ++begin) {
        std::size_t end = begin;
        while (end + 1U < row.jumps.size() &&
               OppositePolarity(row.jumps[end], row.jumps[end + 1U]) &&
               row.jumps[end + 1U].lateral_m -
                       row.jumps[begin].lateral_m <=
                   lateral_window_width_m) {
            ++end;
        }
        const std::size_t jump_count = end - begin + 1U;
        if (jump_count <= best.jump_count) {
            continue;
        }
        best.present = jump_count >= minimum_jump_count;
        best.forward_m = row.forward_m;
        best.lateral_min_m = row.jumps[begin].lateral_m;
        best.lateral_max_m = row.jumps[end].lateral_m;
        best.jump_count = jump_count;
        best.sampleable_count = row.sampleable_count;
    }
    return best;
}

bool IntervalsOverlap(const StripeRowCandidate& lhs,
                      const StripeRowCandidate& rhs) {
    return std::max(lhs.lateral_min_m, rhs.lateral_min_m) <=
           std::min(lhs.lateral_max_m, rhs.lateral_max_m);
}

void StartBand(const StripeRowCandidate& row, StripeBand& band) {
    band.present = true;
    band.forward_min_m = row.forward_m;
    band.forward_max_m = row.forward_m;
    band.lateral_min_m = row.lateral_min_m;
    band.lateral_max_m = row.lateral_max_m;
    band.jump_count = row.jump_count;
    band.sampleable_count = row.sampleable_count;
}

void ExtendBand(const StripeRowCandidate& row, StripeBand& band) {
    band.forward_max_m = row.forward_m;
    band.lateral_min_m = std::min(band.lateral_min_m, row.lateral_min_m);
    band.lateral_max_m = std::max(band.lateral_max_m, row.lateral_max_m);
    band.jump_count += row.jump_count;
    band.sampleable_count += row.sampleable_count;
}

bool BetterBand(const StripeBand& candidate, const StripeBand& current) {
    if (!candidate.present) {
        return false;
    }
    if (!current.present) {
        return true;
    }
    const float candidate_span =
        candidate.forward_max_m - candidate.forward_min_m;
    const float current_span = current.forward_max_m - current.forward_min_m;
    if (candidate_span != current_span) {
        return candidate_span > current_span;
    }
    return candidate.jump_count > current.jump_count;
}

}  // namespace

port::VisualElementEvidenceRecord DetectZebraEvidence(
    const std::vector<BEVSimpleRowScan>& rows,
    const port::RuntimeParameters& params) {
    port::VisualElementEvidenceRecord evidence{};
    evidence.id = "zebra";
    evidence.reason = "periodic_transition_band_absent";
    evidence.candidate.reason = "recognition_only";

    const port::BEVElementParameters& zebra = params.bev_element;
    const float lateral_window_width_m =
        2.0F * params.bev_geometry.nominal_road_half_width_m +
        params.bev_geometry.lateral_step_m;
    const std::size_t minimum_jump_count =
        static_cast<std::size_t>(zebra.zebra_min_jumps_per_row);

    StripeBand current_band{};
    StripeBand best_band{};
    StripeRowCandidate previous_row{};
    auto finish_current_band = [&]() {
        if (BetterBand(current_band, best_band)) {
            best_band = current_band;
        }
        current_band = {};
        previous_row = {};
    };

    for (const BEVSimpleRowScan& row : rows) {
        if (row.forward_m < zebra.zebra_forward_min_m ||
            row.forward_m > zebra.zebra_forward_max_m) {
            finish_current_band();
            continue;
        }
        const StripeRowCandidate candidate =
            FindStripeRow(row, lateral_window_width_m, minimum_jump_count);
        if (!candidate.present) {
            finish_current_band();
            continue;
        }
        if (!current_band.present) {
            StartBand(candidate, current_band);
        } else if (candidate.forward_m - previous_row.forward_m <=
                       zebra.zebra_max_adjacent_forward_gap_m &&
                   IntervalsOverlap(previous_row, candidate)) {
            ExtendBand(candidate, current_band);
        } else {
            finish_current_band();
            StartBand(candidate, current_band);
        }
        previous_row = candidate;
    }
    finish_current_band();

    if (!best_band.present) {
        return evidence;
    }
    evidence.bounds.forward_min_m = best_band.forward_min_m;
    evidence.bounds.forward_max_m = best_band.forward_max_m;
    evidence.bounds.lateral_min_m = best_band.lateral_min_m;
    evidence.bounds.lateral_max_m = best_band.lateral_max_m;
    evidence.support.sampleable_count = best_band.sampleable_count;
    evidence.support.boundary_jump_count = best_band.jump_count;
    if (best_band.forward_max_m - best_band.forward_min_m <
        zebra.zebra_min_support_forward_span_m) {
        evidence.reason = "insufficient_forward_support";
        return evidence;
    }

    evidence.present = true;
    evidence.confidence = 1.0F;
    evidence.reason = "periodic_transition_band";
    return evidence;
}

}  // namespace ls2k::vision
