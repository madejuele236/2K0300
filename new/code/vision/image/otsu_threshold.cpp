#include "vision/image/otsu_threshold.hpp"

#include <array>
#include <cstdint>

#include "vision/image/luma_sampler.hpp"

namespace ls2k::vision {

int OtsuCellCenterIndex(int cell_index, int cell_count, int extent) {
    if (cell_index < 0 || cell_index >= cell_count || cell_count <= 0 ||
        extent < cell_count) {
        return -1;
    }
    const std::int64_t numerator =
        static_cast<std::int64_t>(2 * cell_index + 1) * extent;
    return static_cast<int>(numerator / (2 * cell_count));
}

OtsuThresholdResult ComputeSparseOtsuThreshold(
    const port::CameraPixelFrameView& frame) {
    OtsuThresholdResult result{};
    if (!frame.Valid() || frame.format != port::CameraFrameFormat::kYuyv ||
        frame.width < kOtsuSampleColumns || frame.height < kOtsuSampleRows) {
        return result;
    }

    std::array<std::uint32_t, 256U> histogram{};
    for (int sample_row = 0; sample_row < kOtsuSampleRows; ++sample_row) {
        const int row = OtsuCellCenterIndex(sample_row,
                                            kOtsuSampleRows,
                                            frame.height);
        for (int sample_col = 0; sample_col < kOtsuSampleColumns; ++sample_col) {
            const int col = OtsuCellCenterIndex(sample_col,
                                                kOtsuSampleColumns,
                                                frame.width);
            std::uint8_t y = 0U;
            if (row < 0 || col < 0 ||
                !SampleLumaAt(frame,
                              static_cast<float>(row),
                              static_cast<float>(col),
                              y)) {
                return {};
            }
            ++histogram[y];
            ++result.sample_count;
        }
    }
    if (result.sample_count != kOtsuSampleCount) {
        return {};
    }

    std::uint64_t total_weighted_sum = 0U;
    for (std::size_t value = 0U; value < histogram.size(); ++value) {
        total_weighted_sum +=
            static_cast<std::uint64_t>(value) * histogram[value];
    }

    std::uint32_t background_weight = 0U;
    std::uint64_t background_sum = 0U;
    double best_between_class_variance = -1.0;
    int best_threshold = 0;
    for (int threshold = 0; threshold < 255; ++threshold) {
        const std::uint32_t count = histogram[static_cast<std::size_t>(threshold)];
        background_weight += count;
        background_sum += static_cast<std::uint64_t>(threshold) * count;
        if (background_weight == 0U) {
            continue;
        }
        const std::uint32_t foreground_weight =
            static_cast<std::uint32_t>(result.sample_count) - background_weight;
        if (foreground_weight == 0U) {
            break;
        }
        const double background_mean =
            static_cast<double>(background_sum) / background_weight;
        const double foreground_mean =
            static_cast<double>(total_weighted_sum - background_sum) /
            foreground_weight;
        const double mean_delta = background_mean - foreground_mean;
        const double between_class_variance =
            static_cast<double>(background_weight) * foreground_weight *
            mean_delta * mean_delta;
        if (between_class_variance > best_between_class_variance) {
            best_between_class_variance = between_class_variance;
            best_threshold = threshold;
        }
    }

    if (best_between_class_variance < 0.0) {
        return {};
    }
    result.valid = true;
    result.threshold = best_threshold;
    return result;
}

port::OtsuThresholdState OtsuThresholdTracker::Update(
    const OtsuThresholdResult& current) {
    if (current.valid && current.threshold >= 0 && current.threshold <= 255) {
        have_cached_threshold_ = true;
        cached_threshold_ = current.threshold;
        stale_frames_ = 0U;
        return {true,
                cached_threshold_,
                port::OtsuThresholdSource::kCurrent,
                0U};
    }
    if (have_cached_threshold_ && stale_frames_ < kOtsuMaxCachedFrames) {
        ++stale_frames_;
        return {true,
                cached_threshold_,
                port::OtsuThresholdSource::kCached,
                stale_frames_};
    }
    Reset();
    return {};
}

void OtsuThresholdTracker::Reset() {
    have_cached_threshold_ = false;
    cached_threshold_ = 0;
    stale_frames_ = 0U;
}

}  // namespace ls2k::vision
