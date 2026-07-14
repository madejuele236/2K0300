#include "vision/bev/bev_sample_projection_lut.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace ls2k::vision {
namespace {

float LateralAtIndex(std::size_t index, float lateral_limit, float lateral_step) {
    return -lateral_limit + static_cast<float>(index) * lateral_step;
}

std::size_t ComputeLateralSampleCount(float lateral_limit, float lateral_step) {
    if (lateral_limit <= 0.0F || lateral_step <= 1.0e-5F) {
        return 0;
    }
    return static_cast<std::size_t>(std::floor((2.0F * lateral_limit) / lateral_step + 1.0e-4F)) + 1U;
}

std::size_t ActiveSparseRowCount(const port::RuntimeParameters& params) {
    return static_cast<std::size_t>(
        std::clamp(params.bev_geometry.sparse_row_count,
                   1,
                   static_cast<int>(port::kBevReferenceSampleCount)));
}

bool SameCalibration(const port::BEVProjectorCalibration& lhs,
                     const port::BEVProjectorCalibration& rhs) {
    if (lhs.valid != rhs.valid ||
        lhs.debug_grid_width != rhs.debug_grid_width ||
        lhs.debug_grid_height != rhs.debug_grid_height ||
        lhs.projector_id != rhs.projector_id ||
        lhs.projector_hash != rhs.projector_hash) {
        return false;
    }
    for (std::size_t index = 0; index < port::kBevCalibrationPointCount; ++index) {
        if (lhs.source_points[index].row_px != rhs.source_points[index].row_px ||
            lhs.source_points[index].col_px != rhs.source_points[index].col_px ||
            lhs.target_points[index].forward_m != rhs.target_points[index].forward_m ||
            lhs.target_points[index].lateral_m != rhs.target_points[index].lateral_m) {
            return false;
        }
    }
    return true;
}

bool SameForwardSamples(const std::array<float, port::kBevReferenceSampleCount>& lhs,
                        const std::array<float, port::kBevReferenceSampleCount>& rhs) {
    for (std::size_t index = 0; index < port::kBevReferenceSampleCount; ++index) {
        if (lhs[index] != rhs[index]) {
            return false;
        }
    }
    return true;
}

bool LutMatches(const BEVSampleProjectionLut& lut,
                const port::CameraPixelFrameView& frame,
                const port::RuntimeParameters& params,
                const BEVProjector& projector,
                std::size_t lateral_count,
                float lateral_limit,
                float lateral_step,
                std::size_t active_sparse_rows) {
    return lut.valid &&
           lut.frame_width == frame.width &&
           lut.frame_height == frame.height &&
           lut.frame_stride == frame.stride &&
           lut.sparse_row_count == active_sparse_rows &&
           lut.lateral_sample_count == lateral_count &&
           lut.lateral_limit_m == lateral_limit &&
           lut.lateral_step_m == lateral_step &&
           SameForwardSamples(lut.forward_samples_m, params.bev_geometry.forward_samples_m) &&
           SameCalibration(lut.calibration, projector.Calibration()) &&
           lut.entries.size() == active_sparse_rows * lateral_count;
}

}  // namespace

bool EnsureBEVSampleProjectionLut(BEVSampleProjectionLut& lut,
                                  const port::CameraPixelFrameView& frame,
                                  const port::RuntimeParameters& params,
                                  const BEVProjector& projector) {
    const float lateral_limit =
        std::max(0.110288148F, params.bev_geometry.search_lateral_limit_m);
    const float lateral_step =
        std::max(0.005514407F, params.bev_geometry.lateral_step_m);
    const std::size_t lateral_count = ComputeLateralSampleCount(lateral_limit, lateral_step);
    const std::size_t active_sparse_rows = ActiveSparseRowCount(params);
    if (!projector.Valid() || !frame.Valid() || lateral_count == 0) {
        lut = {};
        return false;
    }
    if (LutMatches(lut,
                   frame,
                   params,
                   projector,
                   lateral_count,
                   lateral_limit,
                   lateral_step,
                   active_sparse_rows)) {
        return true;
    }

    BEVSampleProjectionLut rebuilt{};
    rebuilt.valid = true;
    rebuilt.calibration = projector.Calibration();
    rebuilt.frame_width = frame.width;
    rebuilt.frame_height = frame.height;
    rebuilt.frame_stride = frame.stride;
    rebuilt.forward_samples_m = params.bev_geometry.forward_samples_m;
    rebuilt.sparse_row_count = active_sparse_rows;
    rebuilt.lateral_limit_m = lateral_limit;
    rebuilt.lateral_step_m = lateral_step;
    rebuilt.lateral_sample_count = lateral_count;
    rebuilt.entries.resize(active_sparse_rows * lateral_count);

    for (std::size_t row_index = 0; row_index < active_sparse_rows; ++row_index) {
        const float forward_m = params.bev_geometry.forward_samples_m[row_index];
        for (std::size_t lateral_index = 0; lateral_index < lateral_count; ++lateral_index) {
            BEVSampleProjectionEntry& entry =
                rebuilt.entries[row_index * lateral_count + lateral_index];
            entry.forward_m = forward_m;
            entry.lateral_m = LateralAtIndex(lateral_index, lateral_limit, lateral_step);
            port::ImagePoint image_point{};
            if (!projector.ProjectVehicleToImage({entry.forward_m, entry.lateral_m}, image_point)) {
                entry.state = BEVSampleProjectionState::kProjectionFailed;
                continue;
            }
            entry.image_row_px = image_point.row_px;
            entry.image_col_px = image_point.col_px;
            if (image_point.row_px < 0.0F || image_point.col_px < 0.0F ||
                image_point.row_px > static_cast<float>(frame.height - 1) ||
                image_point.col_px > static_cast<float>(frame.width - 1)) {
                entry.state = BEVSampleProjectionState::kOutsideFrame;
            } else {
                entry.state = BEVSampleProjectionState::kSampleable;
            }
        }
    }

    lut = std::move(rebuilt);
    return true;
}

const char* ToString(BEVSampleProjectionState state) {
    switch (state) {
        case BEVSampleProjectionState::kSampleable:
            return "sampleable";
        case BEVSampleProjectionState::kOutsideFrame:
            return "outside_frame";
        case BEVSampleProjectionState::kProjectionFailed:
            return "projection_failed";
    }
    return "projection_failed";
}

}  // namespace ls2k::vision
