#ifndef LS2K_VISION_IMAGE_OTSU_THRESHOLD_HPP
#define LS2K_VISION_IMAGE_OTSU_THRESHOLD_HPP

#include <cstddef>
#include <cstdint>

#include "port/camera_frame_types.hpp"
#include "port/otsu_threshold_types.hpp"

namespace ls2k::vision {

inline constexpr int kOtsuSampleColumns = 80;
inline constexpr int kOtsuSampleRows = 60;
inline constexpr std::size_t kOtsuSampleCount =
    static_cast<std::size_t>(kOtsuSampleColumns * kOtsuSampleRows);
inline constexpr std::uint8_t kOtsuMaxCachedFrames = 3U;

struct OtsuThresholdResult {
    bool valid = false;
    int threshold = 0;
    std::size_t sample_count = 0U;
};

int OtsuCellCenterIndex(int cell_index, int cell_count, int extent);

OtsuThresholdResult ComputeSparseOtsuThreshold(
    const port::CameraPixelFrameView& frame);

inline bool IsOtsuWhite(std::uint8_t y,
                        const port::OtsuThresholdState& threshold) {
    return threshold.valid && static_cast<int>(y) > threshold.threshold;
}

class OtsuThresholdTracker final {
public:
    port::OtsuThresholdState Update(const OtsuThresholdResult& current);
    void Reset();

private:
    bool have_cached_threshold_ = false;
    int cached_threshold_ = 0;
    std::uint8_t stale_frames_ = 0U;
};

}  // namespace ls2k::vision

#endif  // LS2K_VISION_IMAGE_OTSU_THRESHOLD_HPP
