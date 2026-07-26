#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "vision/image/otsu_threshold.hpp"

namespace {

struct Failure { std::string message; };

void Expect(bool condition, const std::string& message) {
    if (!condition) {
        throw Failure{message};
    }
}

struct Frame {
    std::vector<std::uint8_t> bytes{};
    ls2k::port::CameraPixelFrameView view{};

    Frame(int width, int height, std::uint8_t fill)
        : bytes(static_cast<std::size_t>(width * height * 2), 128U) {
        for (std::size_t index = 0U; index < bytes.size(); index += 2U) {
            bytes[index] = fill;
        }
        view.valid = true;
        view.format = ls2k::port::CameraFrameFormat::kYuyv;
        view.data = bytes.data();
        view.width = width;
        view.height = height;
        view.stride = width * 2;
    }

    void Set(int row, int col, std::uint8_t y) {
        bytes[static_cast<std::size_t>(row * view.stride + col * 2)] = y;
    }
};

void TestGridCenters() {
    Expect(ls2k::vision::OtsuCellCenterIndex(0, 80, 320) == 2,
           "first 320-wide cell center must be column 2");
    Expect(ls2k::vision::OtsuCellCenterIndex(79, 80, 320) == 318,
           "last 320-wide cell center must be column 318");
    Expect(ls2k::vision::OtsuCellCenterIndex(0, 60, 240) == 2,
           "first 240-high cell center must be row 2");
    Expect(ls2k::vision::OtsuCellCenterIndex(59, 60, 240) == 238,
           "last 240-high cell center must be row 238");
    Expect(ls2k::vision::OtsuCellCenterIndex(0, 80, 79) == -1,
           "an extent smaller than the grid must be rejected");
}

void TestSparseHistogramAndTie() {
    Frame frame(320, 240, 90U);
    for (int row_index = 0; row_index < ls2k::vision::kOtsuSampleRows; ++row_index) {
        const int row = ls2k::vision::OtsuCellCenterIndex(
            row_index, ls2k::vision::kOtsuSampleRows, frame.view.height);
        for (int col_index = 0; col_index < ls2k::vision::kOtsuSampleColumns; ++col_index) {
            const int col = ls2k::vision::OtsuCellCenterIndex(
                col_index, ls2k::vision::kOtsuSampleColumns, frame.view.width);
            frame.Set(row, col, col_index < 40 ? 40U : 200U);
        }
    }
    const auto result = ls2k::vision::ComputeSparseOtsuThreshold(frame.view);
    Expect(result.valid, "two non-empty luma classes must produce a threshold");
    Expect(result.sample_count == ls2k::vision::kOtsuSampleCount,
           "Otsu must use exactly 4800 samples");
    Expect(result.threshold == 40,
           "equal-variance plateau must choose its smallest threshold");
    const ls2k::port::OtsuThresholdState state{
        true, result.threshold, ls2k::port::OtsuThresholdSource::kCurrent, 0U};
    Expect(!ls2k::vision::IsOtsuWhite(40U, state),
           "Y equal to threshold must be black");
    Expect(ls2k::vision::IsOtsuWhite(41U, state),
           "Y above threshold must be white");
}

void TestInvalidInputs() {
    Frame uniform(320, 240, 80U);
    Expect(!ls2k::vision::ComputeSparseOtsuThreshold(uniform.view).valid,
           "a one-class histogram must be invalid");
    Frame small(79, 60, 80U);
    Expect(!ls2k::vision::ComputeSparseOtsuThreshold(small.view).valid,
           "a frame smaller than the sampling grid must be invalid");
    ls2k::port::CameraPixelFrameView invalid{};
    Expect(!ls2k::vision::ComputeSparseOtsuThreshold(invalid).valid,
           "an invalid frame must not produce a threshold");
}

void TestThreeFrameCache() {
    ls2k::vision::OtsuThresholdTracker tracker{};
    const ls2k::vision::OtsuThresholdResult valid{true, 91, 4800U};
    auto state = tracker.Update(valid);
    Expect(state.valid && state.source == ls2k::port::OtsuThresholdSource::kCurrent &&
               state.stale_frames == 0U,
           "a current threshold must seed the tracker");
    for (std::uint8_t stale = 1U; stale <= 3U; ++stale) {
        state = tracker.Update({});
        Expect(state.valid && state.threshold == 91 &&
                   state.source == ls2k::port::OtsuThresholdSource::kCached &&
                   state.stale_frames == stale,
               "the last valid threshold must survive exactly three invalid frames");
    }
    state = tracker.Update({});
    Expect(!state.valid && state.source == ls2k::port::OtsuThresholdSource::kNone,
           "the fourth invalid frame must clear the threshold");
    state = tracker.Update(valid);
    tracker.Reset();
    state = tracker.Update({});
    Expect(!state.valid, "explicit reset must clear the cached threshold");
}

}  // namespace

int main() {
    try {
        TestGridCenters();
        TestSparseHistogramAndTie();
        TestInvalidInputs();
        TestThreeFrameCache();
    } catch (const Failure& failure) {
        std::cerr << "FAIL: " << failure.message << '\n';
        return 1;
    }
    std::cout << "PASS: sparse Otsu threshold tests\n";
    return 0;
}
