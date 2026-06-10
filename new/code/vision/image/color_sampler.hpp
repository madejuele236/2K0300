#ifndef LS2K_VISION_IMAGE_COLOR_SAMPLER_HPP
#define LS2K_VISION_IMAGE_COLOR_SAMPLER_HPP

#include <cstdint>

#include "port/camera_frame_types.hpp"

namespace ls2k::vision {

struct CameraColorSample {
    std::uint8_t y = 0U;
    std::uint8_t u = 0U;
    std::uint8_t v = 0U;
};

bool SampleColorAt(const port::CameraPixelFrameView& frame,
                   float row_px,
                   float col_px,
                   CameraColorSample& out);

}  // namespace ls2k::vision

#endif  // LS2K_VISION_IMAGE_COLOR_SAMPLER_HPP
