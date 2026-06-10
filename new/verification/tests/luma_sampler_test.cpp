#include <array>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "port/camera_frame_types.hpp"
#include "vision/image/luma_sampler.hpp"

namespace {

struct TestFailure {
    std::string message;
};

void Expect(bool condition, const std::string& message) {
    if (!condition) {
        throw TestFailure{message};
    }
}

void ExpectSample(const ls2k::port::CameraPixelFrameView& frame,
                  float row,
                  float col,
                  std::uint8_t expected,
                  const std::string& message) {
    std::uint8_t y = 0U;
    Expect(ls2k::vision::SampleLumaAt(frame, row, col, y), message + ": sample failed");
    Expect(y == expected, message + ": unexpected luma");
}

void TestGraySampling() {
    std::array<std::uint8_t, 12> gray{
        10U, 20U, 30U, 40U,
        50U, 60U, 70U, 80U,
        90U, 100U, 110U, 120U,
    };
    ls2k::port::CameraPixelFrameView frame{};
    frame.valid = true;
    frame.format = ls2k::port::CameraFrameFormat::kGray;
    frame.data = gray.data();
    frame.width = 4;
    frame.height = 3;
    frame.stride = 4;

    ExpectSample(frame, 1.0F, 2.0F, 70U, "gray integer sample");
    ExpectSample(frame, 0.5F, 0.5F, 35U, "gray bilinear sample");
}

void TestYuyvSamplingWithStride() {
    std::vector<std::uint8_t> yuyv(3U * 10U, 0U);
    const auto set_y = [&yuyv](int row, int col, std::uint8_t y) {
        yuyv[static_cast<std::size_t>(row) * 10U +
             static_cast<std::size_t>(col) * 2U] = y;
    };
    set_y(0, 0, 10U);
    set_y(0, 1, 20U);
    set_y(0, 2, 30U);
    set_y(0, 3, 40U);
    set_y(1, 0, 50U);
    set_y(1, 1, 60U);
    set_y(1, 2, 70U);
    set_y(1, 3, 80U);
    set_y(2, 0, 90U);
    set_y(2, 1, 100U);
    set_y(2, 2, 110U);
    set_y(2, 3, 120U);

    ls2k::port::CameraPixelFrameView frame{};
    frame.valid = true;
    frame.format = ls2k::port::CameraFrameFormat::kYuyv;
    frame.data = yuyv.data();
    frame.width = 4;
    frame.height = 3;
    frame.stride = 10;

    ExpectSample(frame, 1.0F, 2.0F, 70U, "yuyv integer sample");
    ExpectSample(frame, 0.5F, 0.5F, 35U, "yuyv bilinear sample");
}

void TestBoundariesAndLegacyAdapter() {
    std::array<std::uint8_t, 4> gray{1U, 2U, 3U, 4U};
    ls2k::port::CameraPixelFrameView invalid_stride{};
    invalid_stride.valid = true;
    invalid_stride.format = ls2k::port::CameraFrameFormat::kYuyv;
    invalid_stride.data = gray.data();
    invalid_stride.width = 2;
    invalid_stride.height = 1;
    invalid_stride.stride = 2;
    std::uint8_t y = 0U;
    Expect(!ls2k::vision::SampleLumaAt(invalid_stride, 0.0F, 0.0F, y),
           "invalid yuyv stride must fail");

    ls2k::port::LegacyCameraFrame legacy{};
    legacy.width = 2;
    legacy.height = 2;
    legacy.gray[0] = 11U;
    legacy.gray[1] = 22U;
    legacy.gray[2] = 33U;
    legacy.gray[3] = 44U;
    const ls2k::port::CameraPixelFrameView pixel =
        ls2k::port::MakeCameraPixelFrameView(legacy.View(7U, 123U));
    Expect(pixel.Valid(), "legacy adapter must produce valid pixel view");
    Expect(pixel.frame_id == 7U && pixel.capture_time_ms == 123U,
           "legacy adapter must preserve stamp");
    ExpectSample(pixel, 1.0F, 1.0F, 44U, "legacy adapter sample");
    Expect(!ls2k::vision::SampleLumaAt(pixel, -0.1F, 0.0F, y),
           "negative row must fail");
    Expect(!ls2k::vision::SampleLumaAt(pixel, 0.0F, 2.0F, y),
           "outside col must fail");
}

}  // namespace

int main() {
    try {
        TestGraySampling();
        TestYuyvSamplingWithStride();
        TestBoundariesAndLegacyAdapter();
    } catch (const TestFailure& failure) {
        std::cerr << "FAIL: " << failure.message << '\n';
        return 1;
    }
    std::cout << "luma_sampler_test: PASS\n";
    return 0;
}
