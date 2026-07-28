#ifndef LS2K_VISION_IMAGE_ILLUMINATION_BINARY_MODEL_HPP
#define LS2K_VISION_IMAGE_ILLUMINATION_BINARY_MODEL_HPP

#include <cstdint>

#include "port/binary_model_types.hpp"
#include "port/camera_frame_types.hpp"

namespace ls2k::vision {

inline constexpr int kBinaryResidualThresholdMargin = 22;
inline constexpr float kBinaryIlluminationBlurRadiusPx = 35.0F;
inline constexpr std::uint8_t kBinaryModelMaxCachedFrames = 3U;

struct BinaryModelParameters {
    float illumination_blur_radius_px = kBinaryIlluminationBlurRadiusPx;
    int residual_threshold_margin = kBinaryResidualThresholdMargin;
    int illumination_weight = port::kBinaryDefaultIlluminationWeight;
};

struct BinaryModelResult {
    bool valid = false;
    int residual_threshold = 0;
    int illumination_weight = port::kBinaryDefaultIlluminationWeight;
    std::array<std::uint8_t, port::kBinaryIlluminationSampleCount>
        illumination{};
};

BinaryModelResult ComputeIlluminationBinaryModel(
    const port::CameraPixelFrameView& frame);

BinaryModelResult ComputeIlluminationBinaryModel(
    const port::CameraPixelFrameView& frame,
    const BinaryModelParameters& parameters);

bool ClassifyImagePixel(const port::CameraPixelFrameView& frame,
                        int row,
                        int col,
                        const port::BinaryModelState& model,
                        std::uint8_t& y,
                        bool& white);

bool ClassifyImagePoint(const port::CameraPixelFrameView& frame,
                        float row_px,
                        float col_px,
                        const port::BinaryModelState& model,
                        std::uint8_t& y,
                        bool& white);

class BinaryModelTracker final {
public:
    port::BinaryModelState Update(const BinaryModelResult& current);
    void Reset();

private:
    port::BinaryModelState cached_{};
};

}  // namespace ls2k::vision

#endif  // LS2K_VISION_IMAGE_ILLUMINATION_BINARY_MODEL_HPP
