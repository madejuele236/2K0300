#include "vision/image/illumination_binary_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "vision/image/luma_sampler.hpp"

namespace ls2k::vision {
namespace {

using Illumination =
    std::array<std::uint8_t, port::kBinaryIlluminationSampleCount>;

constexpr int kBlurPassCount = 3;
constexpr std::uint32_t kBoxWeightOne = 1U << 24U;
constexpr std::uint32_t kBoxWeightHalf = 1U << 23U;
constexpr int kMaximumBlurLineLength =
    port::kBinaryIlluminationColumns > port::kBinaryIlluminationRows
        ? port::kBinaryIlluminationColumns
        : port::kBinaryIlluminationRows;

std::size_t IlluminationIndex(int row, int col) {
    return static_cast<std::size_t>(
        row * port::kBinaryIlluminationColumns + col);
}

struct BoxBlurKernel {
    int whole_radius = 0;
    std::uint32_t whole_weight = 0U;
    std::uint32_t fractional_weight = 0U;
};

BoxBlurKernel MakeBoxBlurKernel(float radius) {
    BoxBlurKernel kernel{};
    kernel.whole_radius = static_cast<int>(radius);
    kernel.whole_weight = static_cast<std::uint32_t>(
        static_cast<float>(kBoxWeightOne) / (2.0F * radius + 1.0F));
    kernel.fractional_weight =
        (kBoxWeightOne -
         static_cast<std::uint32_t>(2 * kernel.whole_radius + 1) *
             kernel.whole_weight) /
        2U;
    return kernel;
}

void BoxBlurLine(const std::uint8_t* input,
                 std::uint8_t* output,
                 int length,
                 const BoxBlurKernel& kernel) {
    std::array<std::uint16_t, kMaximumBlurLineLength + 1> prefix_sum{};
    for (int index = 0; index < length; ++index) {
        prefix_sum[static_cast<std::size_t>(index + 1)] =
            static_cast<std::uint16_t>(
                prefix_sum[static_cast<std::size_t>(index)] + input[index]);
    }

    for (int center = 0; center < length; ++center) {
        const int window_left = center - kernel.whole_radius;
        const int window_right = center + kernel.whole_radius;
        const int first = std::max(window_left, 0);
        const int last = std::min(window_right, length - 1);
        std::uint32_t sum =
            prefix_sum[static_cast<std::size_t>(last + 1)] -
            prefix_sum[static_cast<std::size_t>(first)];
        if (window_left < 0) {
            sum += static_cast<std::uint32_t>(-window_left) * input[0];
        }
        if (window_right >= length) {
            sum += static_cast<std::uint32_t>(
                       window_right - length + 1) *
                   input[length - 1];
        }
        const int left = std::clamp(
            center - kernel.whole_radius - 1, 0, length - 1);
        const int right = std::clamp(
            center + kernel.whole_radius + 1, 0, length - 1);
        const std::uint32_t weighted =
            sum * kernel.whole_weight +
            (static_cast<std::uint32_t>(input[left]) + input[right]) *
                kernel.fractional_weight;
        output[center] =
            static_cast<std::uint8_t>((weighted + kBoxWeightHalf) >> 24U);
    }
}

float PillowGaussianBoxRadius(float radius) {
    const float sigma_squared =
        radius * radius / static_cast<float>(kBlurPassCount);
    const float box_length = std::sqrt(12.0F * sigma_squared + 1.0F);
    const float whole_radius = std::floor((box_length - 1.0F) / 2.0F);
    float fractional =
        (2.0F * whole_radius + 1.0F) *
        (whole_radius * (whole_radius + 1.0F) - 3.0F * sigma_squared);
    fractional /=
        6.0F *
        (sigma_squared -
         (whole_radius + 1.0F) * (whole_radius + 1.0F));
    return whole_radius + fractional;
}

void HorizontalBoxBlur(const Illumination& input,
                       Illumination& output,
                       const BoxBlurKernel& kernel) {
    for (int row = 0; row < port::kBinaryIlluminationRows; ++row) {
        const std::size_t offset =
            static_cast<std::size_t>(row * port::kBinaryIlluminationColumns);
        BoxBlurLine(input.data() + offset,
                    output.data() + offset,
                    port::kBinaryIlluminationColumns,
                    kernel);
    }
}

void VerticalBoxBlur(const Illumination& input,
                     Illumination& output,
                     const BoxBlurKernel& kernel) {
    std::array<std::uint8_t, port::kBinaryIlluminationRows> column{};
    std::array<std::uint8_t, port::kBinaryIlluminationRows> blurred{};
    for (int col = 0; col < port::kBinaryIlluminationColumns; ++col) {
        for (int row = 0; row < port::kBinaryIlluminationRows; ++row) {
            column[static_cast<std::size_t>(row)] =
                input[IlluminationIndex(row, col)];
        }
        BoxBlurLine(column.data(),
                    blurred.data(),
                    port::kBinaryIlluminationRows,
                    kernel);
        for (int row = 0; row < port::kBinaryIlluminationRows; ++row) {
            output[IlluminationIndex(row, col)] =
                blurred[static_cast<std::size_t>(row)];
        }
    }
}

Illumination BlurReducedImage(const Illumination& reduced,
                              float blur_radius_px) {
    const float radius = PillowGaussianBoxRadius(
        blur_radius_px / port::kBinaryModelScale);
    const BoxBlurKernel kernel = MakeBoxBlurKernel(radius);
    Illumination first = reduced;
    Illumination second{};
    for (int pass = 0; pass < kBlurPassCount; ++pass) {
        HorizontalBoxBlur(first, second, kernel);
        first = second;
    }
    for (int pass = 0; pass < kBlurPassCount; ++pass) {
        VerticalBoxBlur(first, second, kernel);
        first = second;
    }
    return first;
}

bool ComputeResidualThreshold(
    const Illumination& reduced,
    const Illumination& illumination,
    int illumination_weight,
    int threshold_margin,
    int& threshold) {
    std::array<int, port::kBinaryIlluminationSampleCount> residuals{};
    std::int64_t total_sum = 0;
    for (std::size_t index = 0; index < reduced.size(); ++index) {
        const int residual =
            port::kBinaryResidualLumaScale *
                static_cast<int>(reduced[index]) -
            illumination_weight *
                static_cast<int>(illumination[index]);
        residuals[index] = residual;
        total_sum += residual;
    }
    std::sort(residuals.begin(), residuals.end());

    std::uint32_t left_weight = 0U;
    std::int64_t left_sum = 0;
    long double best_score = -1.0L;
    int best_threshold = 0;
    std::size_t index = 0U;
    while (index < residuals.size()) {
        const int value = residuals[index];
        std::size_t next = index + 1U;
        while (next < residuals.size() && residuals[next] == value) {
            ++next;
        }
        const std::uint32_t value_count =
            static_cast<std::uint32_t>(next - index);
        left_weight += value_count;
        left_sum += static_cast<std::int64_t>(value) * value_count;
        const std::uint32_t right_weight =
            static_cast<std::uint32_t>(reduced.size()) - left_weight;
        if (right_weight == 0U) {
            break;
        }
        const std::int64_t right_sum = total_sum - left_sum;
        const long double score =
            static_cast<long double>(left_sum) * left_sum / left_weight +
            static_cast<long double>(right_sum) * right_sum / right_weight;
        if (score > best_score) {
            best_score = score;
            best_threshold = value;
        }
        index = next;
    }
    if (best_score < 0.0L) {
        return false;
    }
    threshold = best_threshold + threshold_margin;
    return true;
}

bool ValidGeometry(const port::CameraPixelFrameView& frame) {
    return frame.Valid() &&
           frame.format == port::CameraFrameFormat::kYuyv &&
           frame.width ==
               port::kBinaryIlluminationColumns * port::kBinaryModelScale &&
           frame.height ==
               port::kBinaryIlluminationRows * port::kBinaryModelScale;
}

}  // namespace

BinaryModelResult ComputeIlluminationBinaryModel(
    const port::CameraPixelFrameView& frame) {
    return ComputeIlluminationBinaryModel(frame, {});
}

BinaryModelResult ComputeIlluminationBinaryModel(
    const port::CameraPixelFrameView& frame,
    const BinaryModelParameters& parameters) {
    BinaryModelResult result{};
    if (!ValidGeometry(frame) ||
        !std::isfinite(parameters.illumination_blur_radius_px) ||
        parameters.illumination_blur_radius_px <= 0.0F ||
        parameters.illumination_weight < 0 ||
        parameters.illumination_weight >
            port::kBinaryMaximumIlluminationWeight) {
        return result;
    }

    Illumination reduced{};
    for (int row = 0; row < port::kBinaryIlluminationRows; ++row) {
        for (int col = 0; col < port::kBinaryIlluminationColumns; ++col) {
            std::uint8_t y = 0U;
            const int image_row =
                row * port::kBinaryModelScale + port::kBinaryModelScale / 2;
            const int image_col =
                col * port::kBinaryModelScale + port::kBinaryModelScale / 2;
            if (!SampleLumaPixelAt(frame, image_row, image_col, y)) {
                return {};
            }
            reduced[IlluminationIndex(row, col)] = y;
        }
    }

    result.illumination =
        BlurReducedImage(reduced, parameters.illumination_blur_radius_px);
    result.illumination_weight = parameters.illumination_weight;
    if (!ComputeResidualThreshold(
            reduced,
            result.illumination,
            result.illumination_weight,
            parameters.residual_threshold_margin,
            result.residual_threshold)) {
        return {};
    }
    result.valid = true;
    return result;
}

bool ClassifyImagePixel(const port::CameraPixelFrameView& frame,
                        int row,
                        int col,
                        const port::BinaryModelState& model,
                        std::uint8_t& y,
                        bool& white) {
    if (!ValidGeometry(frame) || !model.valid ||
        row < 0 || row >= frame.height || col < 0 || col >= frame.width ||
        !SampleLumaPixelAt(frame, row, col, y)) {
        return false;
    }
    const int illumination_row = row / port::kBinaryModelScale;
    const int illumination_col = col / port::kBinaryModelScale;
    const int residual =
        port::kBinaryResidualLumaScale * static_cast<int>(y) -
        model.illumination_weight *
            static_cast<int>(
                model.illumination[IlluminationIndex(illumination_row,
                                                     illumination_col)]);
    white = residual > model.residual_threshold;
    return true;
}

bool ClassifyImagePoint(const port::CameraPixelFrameView& frame,
                        float row_px,
                        float col_px,
                        const port::BinaryModelState& model,
                        std::uint8_t& y,
                        bool& white) {
    if (!std::isfinite(row_px) || !std::isfinite(col_px)) {
        return false;
    }
    const int row = static_cast<int>(std::floor(row_px + 0.5F));
    const int col = static_cast<int>(std::floor(col_px + 0.5F));
    return ClassifyImagePixel(frame, row, col, model, y, white);
}

port::BinaryModelState BinaryModelTracker::Update(
    const BinaryModelResult& current) {
    if (current.valid) {
        cached_.valid = true;
        cached_.residual_threshold = current.residual_threshold;
        cached_.illumination_weight = current.illumination_weight;
        cached_.source = port::BinaryModelSource::kCurrent;
        cached_.stale_frames = 0U;
        cached_.illumination = current.illumination;
        return cached_;
    }
    if (cached_.valid &&
        cached_.stale_frames < kBinaryModelMaxCachedFrames) {
        ++cached_.stale_frames;
        cached_.source = port::BinaryModelSource::kCached;
        return cached_;
    }
    Reset();
    return {};
}

void BinaryModelTracker::Reset() {
    cached_ = {};
}

}  // namespace ls2k::vision
