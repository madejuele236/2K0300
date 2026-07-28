#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "vision/bev/bev_boundary_row.hpp"

namespace {

struct TestFailure {
    std::string message;
};

void Expect(bool condition, const std::string& message) {
    if (!condition) {
        throw TestFailure{message};
    }
}

ls2k::vision::BEVRowLumaSample Sample(float lateral_m,
                                      int lateral_index,
                                      std::uint8_t y) {
    ls2k::vision::BEVRowLumaSample sample{};
    sample.sampleable = true;
    sample.classified = true;
    sample.white = y > 60U;
    sample.forward_m = 0.4F;
    sample.lateral_m = lateral_m;
    sample.lateral_index = lateral_index;
    sample.y = y;
    return sample;
}

void TestDoubleBoundarySpan() {
    std::vector<ls2k::vision::BEVRowLumaSample> samples{
        Sample(-0.04F, 0, 20U),
        Sample(-0.02F, 1, 90U),
        Sample(0.00F, 2, 95U),
        Sample(0.02F, 3, 25U),
    };
    ls2k::vision::BEVSimpleRowScan row{};
    row.valid = true;
    row.forward_m = 0.4F;
    ls2k::vision::ExtractSparseBoundaryRowFacts(samples, 0.01F, row);

    Expect(row.jumps.size() == 2U, "double boundary span must have two jumps");
    Expect(row.spans.size() == 1U, "double boundary span must form one span");
    Expect(row.jumps[0].polarity == ls2k::vision::BEVBoundaryJumpPolarity::kRisingY,
           "first jump must be rising");
    Expect(row.jumps[1].polarity == ls2k::vision::BEVBoundaryJumpPolarity::kFallingY,
           "second jump must be falling");
    Expect(row.spans[0].left_m < row.spans[0].right_m,
           "span must preserve left-to-right metric order");
    Expect(row.white_runs.size() == 1U,
           "one continuous white region must publish one white run");
    const ls2k::vision::BEVWhiteRun& run = row.white_runs.front();
    Expect(run.left_endpoint == ls2k::vision::BEVWhiteRunEndpointState::kBoundary &&
               run.right_endpoint == ls2k::vision::BEVWhiteRunEndpointState::kBoundary,
           "fully observed white run must keep two real boundaries");
    Expect(run.left_jump_index == 0 && run.right_jump_index == 1,
           "white run real endpoints must reference their observed jumps");
}

void TestWhiteRunAtSampleableEdges() {
    {
        std::vector<ls2k::vision::BEVRowLumaSample> samples{
            Sample(-0.04F, 0, 90U),
            Sample(-0.02F, 1, 95U),
            Sample(0.00F, 2, 20U),
        };
        ls2k::vision::BEVSimpleRowScan row{};
        ls2k::vision::ExtractSparseBoundaryRowFacts(samples, 0.01F, row);
        Expect(row.white_runs.size() == 1U, "left-edge white area must publish one run");
        Expect(row.white_runs[0].left_endpoint ==
                   ls2k::vision::BEVWhiteRunEndpointState::kFovEdge &&
                   row.white_runs[0].right_endpoint ==
                       ls2k::vision::BEVWhiteRunEndpointState::kBoundary,
               "left-edge white area must be FOV-open only on the left");
        Expect(row.white_runs[0].left_jump_index == -1 &&
                   row.white_runs[0].right_jump_index == 0,
               "FOV endpoint must not synthesize a jump");
    }
    {
        std::vector<ls2k::vision::BEVRowLumaSample> samples{
            Sample(-0.04F, 0, 20U),
            Sample(-0.02F, 1, 90U),
            Sample(0.00F, 2, 95U),
        };
        ls2k::vision::BEVSimpleRowScan row{};
        ls2k::vision::ExtractSparseBoundaryRowFacts(samples, 0.01F, row);
        Expect(row.white_runs.size() == 1U, "right-edge white area must publish one run");
        Expect(row.white_runs[0].left_endpoint ==
                   ls2k::vision::BEVWhiteRunEndpointState::kBoundary &&
                   row.white_runs[0].right_endpoint ==
                       ls2k::vision::BEVWhiteRunEndpointState::kFovEdge,
               "right-edge white area must be FOV-open only on the right");
    }
}

void TestInternalGapIsNotFov() {
    std::vector<ls2k::vision::BEVRowLumaSample> samples{
        Sample(-0.04F, 0, 20U),
        Sample(-0.02F, 1, 90U),
        Sample(0.00F, 2, 90U),
        Sample(0.02F, 3, 20U),
    };
    samples[2].sampleable = false;
    ls2k::vision::BEVSimpleRowScan row{};
    ls2k::vision::ExtractSparseBoundaryRowFacts(samples, 0.01F, row);
    Expect(row.white_runs.size() == 1U,
           "internal unobservable point must terminate the visible white run");
    Expect(row.white_runs[0].left_endpoint ==
               ls2k::vision::BEVWhiteRunEndpointState::kBoundary &&
               row.white_runs[0].right_endpoint ==
                   ls2k::vision::BEVWhiteRunEndpointState::kUnavailableGap,
           "internal gap must not be relabeled as a FOV edge");
}

void TestAllWhitePublishesTwoFovEndpoints() {
    std::vector<ls2k::vision::BEVRowLumaSample> samples{
        Sample(-0.02F, 0, 90U),
        Sample(0.00F, 1, 95U),
    };
    ls2k::vision::BEVSimpleRowScan row{};
    ls2k::vision::ExtractSparseBoundaryRowFacts(samples, 0.01F, row);
    Expect(row.white_runs.size() == 1U, "all-white row must publish its observable run");
    Expect(row.white_runs[0].left_endpoint ==
               ls2k::vision::BEVWhiteRunEndpointState::kFovEdge &&
               row.white_runs[0].right_endpoint ==
                   ls2k::vision::BEVWhiteRunEndpointState::kFovEdge,
           "all-white row must remain open on both FOV sides");
    Expect(row.jumps.empty() && row.spans.empty(),
           "all-white row must not synthesize boundary or span facts");
}

void TestMultipleSpansAndUnpairedJump() {
    std::vector<ls2k::vision::BEVRowLumaSample> samples{
        Sample(-0.10F, 0, 20U),
        Sample(-0.08F, 1, 90U),
        Sample(-0.06F, 2, 25U),
        Sample(0.02F, 3, 30U),
        Sample(0.04F, 4, 100U),
        Sample(0.06F, 5, 35U),
        Sample(0.10F, 6, 120U),
    };
    ls2k::vision::BEVSimpleRowScan row{};
    row.valid = true;
    row.forward_m = 0.5F;
    ls2k::vision::ExtractSparseBoundaryRowFacts(samples, 0.01F, row);

    Expect(row.jumps.size() == 5U, "row must keep unpaired jump facts");
    Expect(row.spans.size() == 2U, "row must form two paired spans");
}

void TestUnsampleableBreaksLocalConnectivity() {
    std::vector<ls2k::vision::BEVRowLumaSample> samples{
        Sample(-0.04F, 0, 20U),
        Sample(-0.02F, 1, 90U),
        Sample(0.00F, 2, 95U),
        Sample(0.02F, 3, 25U),
    };
    samples[2].sampleable = false;
    ls2k::vision::BEVSimpleRowScan row{};
    row.valid = true;
    row.forward_m = 0.6F;
    ls2k::vision::ExtractSparseBoundaryRowFacts(samples, 0.01F, row);

    Expect(row.jumps.size() == 1U, "unsampleable gap must prevent cross-gap jump");
    Expect(row.spans.empty(), "unsampleable gap must prevent paired span");
}

void TestMissingLateralIndexBreaksConnectivity() {
    std::vector<ls2k::vision::BEVRowLumaSample> samples{
        Sample(-0.04F, 0, 20U),
        Sample(-0.02F, 1, 90U),
        Sample(0.02F, 3, 20U),
    };
    ls2k::vision::BEVSimpleRowScan row{};
    row.valid = true;
    ls2k::vision::ExtractSparseBoundaryRowFacts(samples, 0.01F, row);
    Expect(row.jumps.size() == 1U,
           "a removed unsampleable index must not create a cross-gap transition");
    Expect(row.spans.empty(),
           "a removed unsampleable index must break the white span");
}

void TestThresholdClassOwnsBoundary() {
    std::vector<ls2k::vision::BEVRowLumaSample> samples{
        Sample(-0.06F, 0, 10U),
        Sample(-0.04F, 1, 59U),
        Sample(-0.02F, 2, 61U),
        Sample(0.00F, 3, 250U),
        Sample(0.02F, 4, 60U),
    };
    ls2k::vision::BEVSimpleRowScan row{};
    row.valid = true;
    ls2k::vision::ExtractSparseBoundaryRowFacts(samples, 0.01F, row);
    Expect(row.jumps.size() == 2U,
           "only binary class transitions may create boundaries");
    Expect(row.jumps[0].delta_y == 2,
           "a small luma delta crossing classes must remain a boundary");
    Expect(row.jumps[1].delta_y == -190,
           "Y equal to threshold must be classified as black");
}

}  // namespace

int main() {
    try {
        TestDoubleBoundarySpan();
        TestWhiteRunAtSampleableEdges();
        TestInternalGapIsNotFov();
        TestAllWhitePublishesTwoFovEndpoints();
        TestMultipleSpansAndUnpairedJump();
        TestUnsampleableBreaksLocalConnectivity();
        TestMissingLateralIndexBreaksConnectivity();
        TestThresholdClassOwnsBoundary();
    } catch (const TestFailure& failure) {
        std::cerr << "FAIL: " << failure.message << '\n';
        return 1;
    }
    std::cout << "bev_boundary_row_test: PASS\n";
    return 0;
}
