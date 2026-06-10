#include "vision/image/luma_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace ls2k::vision {
namespace {

bool LumaAtIntegerPixel(const port::CameraPixelFrameView& frame,
                        int row,
                        int col,
                        std::uint8_t& y) {
    if (!frame.Valid() || row < 0 || col < 0 || row >= frame.height || col >= frame.width) {
        return false;
    }
    const std::size_t row_offset =
        static_cast<std::size_t>(row) * static_cast<std::size_t>(frame.stride);
    std::size_t index = row_offset;
    switch (frame.format) {
    case port::CameraFrameFormat::kGray:
        index += static_cast<std::size_t>(col);
        break;
    case port::CameraFrameFormat::kYuyv:
        index += static_cast<std::size_t>(col) * 2U;
        break;
    }
    y = frame.data[index];
    return true;
}

}  // namespace

bool SampleLumaAt(const port::CameraPixelFrameView& frame,
                  float row_px,
                  float col_px,
                  std::uint8_t& y) {
    if (!frame.Valid()) {
        return false;
    }
    if (row_px < 0.0F || col_px < 0.0F ||
        row_px > static_cast<float>(frame.height - 1) ||
        col_px > static_cast<float>(frame.width - 1)) {
        return false;
    }

    const int row0 = static_cast<int>(std::floor(row_px));
    const int col0 = static_cast<int>(std::floor(col_px));
    const int row1 = std::min(row0 + 1, frame.height - 1);
    const int col1 = std::min(col0 + 1, frame.width - 1);
    const float row_frac = row_px - static_cast<float>(row0);
    const float col_frac = col_px - static_cast<float>(col0);

    std::uint8_t y00 = 0U;
    std::uint8_t y01 = 0U;
    std::uint8_t y10 = 0U;
    std::uint8_t y11 = 0U;
    if (!LumaAtIntegerPixel(frame, row0, col0, y00) ||
        !LumaAtIntegerPixel(frame, row0, col1, y01) ||
        !LumaAtIntegerPixel(frame, row1, col0, y10) ||
        !LumaAtIntegerPixel(frame, row1, col1, y11)) {
        return false;
    }

    const float top = static_cast<float>(y00) * (1.0F - col_frac) +
                      static_cast<float>(y01) * col_frac;
    const float bottom = static_cast<float>(y10) * (1.0F - col_frac) +
                         static_cast<float>(y11) * col_frac;
    const float value = top * (1.0F - row_frac) + bottom * row_frac;
    y = static_cast<std::uint8_t>(std::lround(std::clamp(value, 0.0F, 255.0F)));
    return true;
}

}  // namespace ls2k::vision
