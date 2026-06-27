#include "vision/image/luma_sampler.hpp"

#include <algorithm>
#include <cstddef>

namespace ls2k::vision {
namespace {

std::uint8_t YuyvLumaAtUnchecked(const port::CameraPixelFrameView& frame, int row, int col) {
    const std::size_t row_offset =
        static_cast<std::size_t>(row) * static_cast<std::size_t>(frame.stride);
    const std::size_t index = row_offset + static_cast<std::size_t>(col) * 2U;
    return frame.data[index];
}

bool SampleYuyvLumaAt(const port::CameraPixelFrameView& frame,
                      int row0,
                      int col0,
                      float row_frac,
                      float col_frac,
                      std::uint8_t& y) {
    if (row_frac == 0.0F && col_frac == 0.0F) {
        y = YuyvLumaAtUnchecked(frame, row0, col0);
        return true;
    }

    const int row1 = std::min(row0 + 1, frame.height - 1);
    const int col1 = std::min(col0 + 1, frame.width - 1);

    const std::uint8_t y00 = YuyvLumaAtUnchecked(frame, row0, col0);
    const std::uint8_t y01 = YuyvLumaAtUnchecked(frame, row0, col1);
    const std::uint8_t y10 = YuyvLumaAtUnchecked(frame, row1, col0);
    const std::uint8_t y11 = YuyvLumaAtUnchecked(frame, row1, col1);

    const float top = static_cast<float>(y00) * (1.0F - col_frac) +
                      static_cast<float>(y01) * col_frac;
    const float bottom = static_cast<float>(y10) * (1.0F - col_frac) +
                         static_cast<float>(y11) * col_frac;
    const float value = top * (1.0F - row_frac) + bottom * row_frac;
    const int rounded = std::clamp(static_cast<int>(value + 0.5F), 0, 255);
    y = static_cast<std::uint8_t>(rounded);
    return true;
}

}  // namespace

bool SampleLumaAt(const port::CameraPixelFrameView& frame,
                  float row_px,
                  float col_px,
                  std::uint8_t& y) {
    if (!frame.Valid() || frame.format != port::CameraFrameFormat::kYuyv) {
        return false;
    }
    if (row_px < 0.0F || col_px < 0.0F ||
        row_px > static_cast<float>(frame.height - 1) ||
        col_px > static_cast<float>(frame.width - 1)) {
        return false;
    }

    const int row0 = static_cast<int>(row_px);
    const int col0 = static_cast<int>(col_px);
    const float row_frac = row_px - static_cast<float>(row0);
    const float col_frac = col_px - static_cast<float>(col0);

    return SampleYuyvLumaAt(frame, row0, col0, row_frac, col_frac, y);
}

}  // namespace ls2k::vision
