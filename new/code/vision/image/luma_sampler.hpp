#ifndef LS2K_VISION_IMAGE_LUMA_SAMPLER_HPP
#define LS2K_VISION_IMAGE_LUMA_SAMPLER_HPP

#include <cstdint>

#include "port/camera_frame_types.hpp"

namespace ls2k::vision {

bool SampleLumaAt(const port::CameraPixelFrameView& frame,
                  float row_px,
                  float col_px,
                  std::uint8_t& y);

}  // namespace ls2k::vision

#endif  // LS2K_VISION_IMAGE_LUMA_SAMPLER_HPP
