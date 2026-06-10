#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "vision/bev/bev_simple_perception.hpp"

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
    sample.forward_m = 0.4F;
    sample.lateral_m = lateral_m;
    sample.lateral_index = lateral_index;
    sample.y = y;
    return sample;
}

ls2k::port::BEVBoundaryParameters BoundaryParams() {
    ls2k::port::BEVBoundaryParameters params{};
    params.local_jump_min_y = 30;
    return params;
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
    ls2k::vision::ExtractSparseBoundaryRowFacts(samples, BoundaryParams(), 0.01F, row);

    Expect(row.jumps.size() == 2U, "double boundary span must have two jumps");
    Expect(row.spans.size() == 1U, "double boundary span must form one span");
    Expect(row.jumps[0].polarity == ls2k::vision::BEVBoundaryJumpPolarity::kRisingY,
           "first jump must be rising");
    Expect(row.jumps[1].polarity == ls2k::vision::BEVBoundaryJumpPolarity::kFallingY,
           "second jump must be falling");
    Expect(row.spans[0].left_m < row.spans[0].right_m,
           "span must preserve left-to-right metric order");
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
    ls2k::vision::ExtractSparseBoundaryRowFacts(samples, BoundaryParams(), 0.01F, row);

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
    ls2k::vision::ExtractSparseBoundaryRowFacts(samples, BoundaryParams(), 0.01F, row);

    Expect(row.jumps.size() == 1U, "unsampleable gap must prevent cross-gap jump");
    Expect(row.spans.empty(), "unsampleable gap must prevent paired span");
}

}  // namespace

int main() {
    try {
        TestDoubleBoundarySpan();
        TestMultipleSpansAndUnpairedJump();
        TestUnsampleableBreaksLocalConnectivity();
    } catch (const TestFailure& failure) {
        std::cerr << "FAIL: " << failure.message << '\n';
        return 1;
    }
    std::cout << "bev_boundary_row_test: PASS\n";
    return 0;
}
