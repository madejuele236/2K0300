#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "vision/elements/zebra_element_evidence.hpp"

namespace {

using ls2k::vision::BEVBoundaryJump;
using ls2k::vision::BEVBoundaryJumpPolarity;
using ls2k::vision::BEVSimpleRowScan;

void Expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "zebra_element_evidence_test failed: " << message << '\n';
        std::exit(1);
    }
}

BEVSimpleRowScan MakeRow(float forward_m,
                         float lateral_begin_m,
                         std::size_t jump_count,
                         float lateral_step_m = 0.04F) {
    BEVSimpleRowScan row{};
    row.valid = true;
    row.forward_m = forward_m;
    row.sampleable_count = 40U;
    for (std::size_t index = 0U; index < jump_count; ++index) {
        BEVBoundaryJump jump{};
        jump.forward_m = forward_m;
        jump.lateral_m =
            lateral_begin_m + static_cast<float>(index) * lateral_step_m;
        jump.lateral_index = static_cast<int>(index + 3U);
        jump.polarity = index % 2U == 0U
                            ? BEVBoundaryJumpPolarity::kRisingY
                            : BEVBoundaryJumpPolarity::kFallingY;
        row.jumps.push_back(jump);
    }
    return row;
}

ls2k::port::RuntimeParameters Params() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.nominal_road_half_width_m = 0.20F;
    params.bev_geometry.lateral_step_m = 0.022F;
    params.bev_element.zebra_forward_min_m = 0.05F;
    params.bev_element.zebra_forward_max_m = 0.50F;
    params.bev_element.zebra_min_jumps_per_row = 6;
    params.bev_element.zebra_max_adjacent_forward_gap_m = 0.12F;
    params.bev_element.zebra_min_support_forward_span_m = 0.04F;
    return params;
}

void TestPeriodicBandIsDetected() {
    const std::vector<BEVSimpleRowScan> rows{
        MakeRow(0.10F, -0.18F, 8U),
        MakeRow(0.125F, -0.17F, 10U, 0.035F),
        MakeRow(0.15F, -0.16F, 8U),
    };
    const auto evidence = ls2k::vision::DetectZebraEvidence(rows, Params());
    Expect(evidence.id == "zebra", "record id must be zebra");
    Expect(evidence.present, "a sustained periodic transition band must be detected");
    Expect(evidence.reason == "periodic_transition_band",
           "present evidence must expose the owner reason");
    Expect(std::fabs(evidence.bounds.forward_min_m - 0.10F) < 1.0e-6F &&
               std::fabs(evidence.bounds.forward_max_m - 0.15F) < 1.0e-6F,
           "evidence bounds must describe the supporting rows");
    Expect(!evidence.candidate.built &&
               evidence.candidate.reason == "recognition_only",
           "recognizer must not build a takeover candidate");
}

void TestSingleMarkerIsRejected() {
    const std::vector<BEVSimpleRowScan> rows{
        MakeRow(0.10F, -0.18F, 4U),
        MakeRow(0.15F, -0.18F, 4U),
        MakeRow(0.20F, -0.18F, 4U),
    };
    const auto evidence = ls2k::vision::DetectZebraEvidence(rows, Params());
    Expect(!evidence.present,
           "a single rectangular marker must not satisfy zebra periodicity");
    Expect(evidence.reason == "periodic_transition_band_absent",
           "rows below the jump threshold must report no periodic band");
}

void TestInsufficientForwardSupportIsRejected() {
    const std::vector<BEVSimpleRowScan> rows{
        MakeRow(0.10F, -0.18F, 8U),
        MakeRow(0.125F, -0.17F, 8U),
    };
    const auto evidence = ls2k::vision::DetectZebraEvidence(rows, Params());
    Expect(!evidence.present,
           "a short high-frequency row pair must not establish a zebra band");
    Expect(evidence.reason == "insufficient_forward_support",
           "short support must remain observable as a distinct rejection");
}

void TestLateralDiscontinuityBreaksBand() {
    const std::vector<BEVSimpleRowScan> rows{
        MakeRow(0.10F, -0.38F, 6U, 0.03F),
        MakeRow(0.125F, 0.10F, 6U, 0.03F),
        MakeRow(0.15F, -0.38F, 6U, 0.03F),
    };
    const auto evidence = ls2k::vision::DetectZebraEvidence(rows, Params());
    Expect(!evidence.present,
           "non-overlapping transition rows must not compose one zebra band");
}

void TestMetricSupportIsSamplingCountIndependent() {
    const std::vector<BEVSimpleRowScan> dense{
        MakeRow(0.10F, -0.18F, 8U),
        MakeRow(0.12F, -0.18F, 8U),
        MakeRow(0.14F, -0.18F, 8U),
    };
    const std::vector<BEVSimpleRowScan> sparse{
        MakeRow(0.10F, -0.18F, 8U),
        MakeRow(0.14F, -0.18F, 8U),
    };
    Expect(ls2k::vision::DetectZebraEvidence(dense, Params()).present,
           "dense sampling must detect the metric band");
    Expect(ls2k::vision::DetectZebraEvidence(sparse, Params()).present,
           "the same metric band must not require a fixed row count");
}

void TestRoiExcludesFarBackgroundTransitions() {
    const std::vector<BEVSimpleRowScan> rows{
        MakeRow(1.10F, -0.18F, 18U, 0.02F),
        MakeRow(1.20F, -0.18F, 18U, 0.02F),
        MakeRow(1.30F, -0.18F, 18U, 0.02F),
    };
    Expect(!ls2k::vision::DetectZebraEvidence(rows, Params()).present,
           "transitions outside the configured forward ROI must be ignored");
}

}  // namespace

int main() {
    TestPeriodicBandIsDetected();
    TestSingleMarkerIsRejected();
    TestInsufficientForwardSupportIsRejected();
    TestLateralDiscontinuityBreaksBand();
    TestMetricSupportIsSamplingCountIndependent();
    TestRoiExcludesFarBackgroundTransitions();
    std::cout << "zebra_element_evidence_test passed\n";
    return 0;
}
