#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "vision/image/illumination_binary_model.hpp"

namespace {

struct TestFailure {
    std::string message;
};

void Expect(bool condition, const std::string& message) {
    if (!condition) {
        throw TestFailure{message};
    }
}

struct YuyvFrame {
    explicit YuyvFrame(int width = 320, int height = 240)
        : storage(static_cast<std::size_t>(width * height * 2), 128U) {
        view.valid = true;
        view.data = storage.data();
        view.width = width;
        view.height = height;
        view.stride = width * 2;
        view.format = ls2k::port::CameraFrameFormat::kYuyv;
    }

    void Fill(std::uint8_t y) {
        for (int row = 0; row < view.height; ++row) {
            for (int col = 0; col < view.width; ++col) {
                Set(row, col, y);
            }
        }
    }

    void Set(int row, int col, std::uint8_t y) {
        storage[static_cast<std::size_t>(
            row * view.stride + col * 2)] = y;
    }

    std::vector<std::uint8_t> storage{};
    ls2k::port::CameraPixelFrameView view{};
};

void TestOwnerGeometryAndTwoClassRequirement() {
    YuyvFrame uniform{};
    uniform.Fill(100U);
    Expect(!ls2k::vision::ComputeIlluminationBinaryModel(uniform.view).valid,
           "uniform residual samples must not invent two Otsu classes");

    YuyvFrame small{304, 240};
    small.Fill(100U);
    Expect(!ls2k::vision::ComputeIlluminationBinaryModel(small.view).valid,
           "the fixed 20x15 illumination contract requires 320x240 input");

    ls2k::port::CameraPixelFrameView invalid{};
    Expect(!ls2k::vision::ComputeIlluminationBinaryModel(invalid).valid,
           "invalid input frame must not publish a model");
}

void TestOwnerProducesSpatialModel() {
    YuyvFrame frame{};
    for (int row = 0; row < frame.view.height; ++row) {
        for (int col = 0; col < frame.view.width; ++col) {
            const bool bright_patch = row >= 80 && row < 160 &&
                                      col >= 96 && col < 224;
            frame.Set(row, col, bright_patch ? 220U : 45U);
        }
    }

    const ls2k::vision::BinaryModelResult result =
        ls2k::vision::ComputeIlluminationBinaryModel(frame.view);
    Expect(result.valid, "two-region image must produce a binary model");
    const auto [minimum, maximum] =
        std::minmax_element(result.illumination.begin(),
                            result.illumination.end());
    Expect(*minimum < *maximum,
           "illumination model must retain spatial lighting variation");
}

void TestClassificationUsesOneStrictPredicate() {
    YuyvFrame frame{};
    frame.Fill(50U);
    ls2k::port::BinaryModelState model{};
    model.valid = true;
    model.source = ls2k::port::BinaryModelSource::kCurrent;
    model.illumination.fill(100U);
    model.residual_threshold = 0;

    std::uint8_t y = 0U;
    bool white = true;
    Expect(ls2k::vision::ClassifyImagePixel(
               frame.view, 32, 48, model, y, white),
           "valid pixel must be classified");
    Expect(y == 50U && !white,
           "residual equal to threshold must classify as black");

    frame.Set(32, 48, 51U);
    Expect(ls2k::vision::ClassifyImagePixel(
               frame.view, 32, 48, model, y, white),
           "updated valid pixel must be classified");
    Expect(white, "positive residual must classify as white");

    model.valid = false;
    Expect(!ls2k::vision::ClassifyImagePixel(
               frame.view, 32, 48, model, y, white),
           "invalid model must not produce a classification fact");
}

void TestTrackerContinuityAndReset() {
    ls2k::vision::BinaryModelTracker tracker{};
    ls2k::vision::BinaryModelResult valid{};
    valid.valid = true;
    valid.residual_threshold = 91;
    valid.illumination.fill(73U);

    ls2k::port::BinaryModelState state = tracker.Update(valid);
    Expect(state.valid &&
               state.source == ls2k::port::BinaryModelSource::kCurrent &&
               state.stale_frames == 0U &&
               state.residual_threshold == 91 &&
               state.illumination.front() == 73U,
           "new valid result must become the current complete model");

    for (std::uint8_t stale = 1U; stale <= 3U; ++stale) {
        state = tracker.Update({});
        Expect(state.valid &&
                   state.source == ls2k::port::BinaryModelSource::kCached &&
                   state.stale_frames == stale &&
                   state.illumination.front() == 73U,
               "the complete model must remain cached for three frames");
    }

    state = tracker.Update({});
    Expect(!state.valid &&
               state.source == ls2k::port::BinaryModelSource::kNone,
           "the fourth invalid frame must clear the model");

    tracker.Update(valid);
    tracker.Reset();
    state = tracker.Update({});
    Expect(!state.valid,
           "explicit reset must clear cached model continuity");
}

}  // namespace

int main() {
    try {
        TestOwnerGeometryAndTwoClassRequirement();
        TestOwnerProducesSpatialModel();
        TestClassificationUsesOneStrictPredicate();
        TestTrackerContinuityAndReset();
    } catch (const TestFailure& failure) {
        std::cerr << "FAIL: " << failure.message << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "PASS: illumination-compensated binary model tests\n";
    return EXIT_SUCCESS;
}
