#include "vision/image/color_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace ls2k::vision {
namespace {

float ClampByte(float value) { return std::clamp(value, 0.0F, 255.0F); }

CameraColorSample YuyvAt(const port::CameraPixelFrameView& frame, int row, int col) {
    const std::size_t base = static_cast<std::size_t>(row) * frame.stride +
                             static_cast<std::size_t>(col & ~1) * 2U;
    CameraColorSample sample{};
    sample.y = frame.data[base + static_cast<std::size_t>((col & 1) * 2)];
    sample.u = frame.data[base + 1U];
    sample.v = frame.data[base + 3U];
    return sample;
}

std::uint8_t BilinearChannel(std::uint8_t v00,
                             std::uint8_t v01,
                             std::uint8_t v10,
                             std::uint8_t v11,
                             float row_fraction,
                             float col_fraction) {
    const float top = v00 * (1.0F - col_fraction) + v01 * col_fraction;
    const float bottom = v10 * (1.0F - col_fraction) + v11 * col_fraction;
    return static_cast<std::uint8_t>(std::clamp(
        static_cast<int>(top * (1.0F - row_fraction) + bottom * row_fraction + 0.5F),
        0,
        255));
}

}  // namespace

bool SampleColorAt(const port::CameraPixelFrameView& frame,
                   float row_px,
                   float col_px,
                   CameraColorSample& out) {
    if (!frame.Valid() || frame.format != port::CameraFrameFormat::kYuyv ||
        frame.width % 2 != 0 ||
        !std::isfinite(row_px) || !std::isfinite(col_px) || row_px < 0.0F ||
        col_px < 0.0F || row_px > frame.height - 1 || col_px > frame.width - 1) {
        return false;
    }
    const int row0 = static_cast<int>(row_px);
    const int col0 = static_cast<int>(col_px);
    const int row1 = std::min(row0 + 1, frame.height - 1);
    const int col1 = std::min(col0 + 1, frame.width - 1);
    const float rf = row_px - row0;
    const float cf = col_px - col0;
    const CameraColorSample s00 = YuyvAt(frame, row0, col0);
    const CameraColorSample s01 = YuyvAt(frame, row0, col1);
    const CameraColorSample s10 = YuyvAt(frame, row1, col0);
    const CameraColorSample s11 = YuyvAt(frame, row1, col1);
    out.y = BilinearChannel(s00.y, s01.y, s10.y, s11.y, rf, cf);
    out.u = BilinearChannel(s00.u, s01.u, s10.u, s11.u, rf, cf);
    out.v = BilinearChannel(s00.v, s01.v, s10.v, s11.v, rf, cf);
    return true;
}

bool SampleRgbMeanAt(const port::CameraPixelFrameView& frame,
                     float row_px,
                     float col_px,
                     std::uint8_t& rgb_mean) {
    CameraColorSample sample{};
    if (!SampleColorAt(frame, row_px, col_px, sample)) return false;
    const float y = static_cast<float>(sample.y);
    const float u = static_cast<float>(sample.u) - 128.0F;
    const float v = static_cast<float>(sample.v) - 128.0F;
    const float r = ClampByte(y + 1.402F * v);
    const float g = ClampByte(y - 0.344136F * u - 0.714136F * v);
    const float b = ClampByte(y + 1.772F * u);
    rgb_mean = static_cast<std::uint8_t>(std::clamp(
        static_cast<int>((r + g + b) / 3.0F + 0.5F), 0, 255));
    return true;
}

}  // namespace ls2k::vision
