#include "vision/bev/bev_sparse_row_scanner.hpp"

#include <algorithm>
#include <cstdint>

#include "vision/bev/bev_boundary_row.hpp"
#include "vision/image/luma_sampler.hpp"

namespace ls2k::vision {
namespace {

std::size_t ActiveSparseRowCount(const port::RuntimeParameters& params) {
    return static_cast<std::size_t>(
        std::clamp(params.bev_geometry.sparse_row_count,
                   1,
                   static_cast<int>(port::kBevReferenceSampleCount)));
}

// 扫描单条稀疏 BEV 行，产出该行的黑/白/未知/不可用计数以及白色连续区间。
// 这些 row facts 同时服务基础 line reference 和 element evidence，是当前视觉事实的公共输入。
// 这里不做 cross/circle 的语义判断，只描述这一行本身看到了什么。
BEVSimpleRowScan ScanSparseRow(const port::CameraPixelFrameView& frame,
                               const port::RuntimeParameters& params,
                               const BEVSampleProjectionLut& lut,
                               std::size_t row_index) {
    BEVSimpleRowScan row{};
    if (!lut.valid || row_index >= port::kBevReferenceSampleCount || lut.lateral_sample_count == 0) {
        return row;
    }

    row.valid = true;
    row.forward_m = params.bev_geometry.forward_samples_m[row_index];
    row.row_px = static_cast<int>(row_index);
    const float min_width_m =
        std::max(0.022057630F, params.bev_geometry.lateral_step_m * 1.5F);
    bool have_sampleable_lateral = false;
    std::vector<BEVRowLumaSample> luma_samples;
    luma_samples.reserve(lut.lateral_sample_count);
    for (std::size_t lateral_index = 0; lateral_index < lut.lateral_sample_count; ++lateral_index) {
        const BEVSampleProjectionEntry& entry =
            lut.entries[row_index * lut.lateral_sample_count + lateral_index];
        if (entry.state != BEVSampleProjectionState::kSampleable) {
            ++row.unavailable_count;
            continue;
        }
        std::uint8_t gray = 0;
        if (!SampleLumaAt(frame, entry.image_row_px, entry.image_col_px, gray)) {
            ++row.unavailable_count;
            continue;
        }
        const float lateral = entry.lateral_m;
        BEVRowLumaSample sample{};
        sample.sampleable = true;
        sample.forward_m = row.forward_m;
        sample.lateral_m = lateral;
        sample.lateral_index = static_cast<int>(lateral_index);
        sample.y = gray;
        luma_samples.push_back(sample);
        ++row.sampleable_count;
        if (!have_sampleable_lateral) {
            row.sampleable_left_m = lateral;
            row.sampleable_right_m = lateral;
            have_sampleable_lateral = true;
        } else {
            row.sampleable_left_m = std::min(row.sampleable_left_m, lateral);
            row.sampleable_right_m = std::max(row.sampleable_right_m, lateral);
        }
    }
    if (have_sampleable_lateral) {
        row.sampleable_width_m = std::max(0.0F, row.sampleable_right_m - row.sampleable_left_m);
    }
    ExtractSparseBoundaryRowFacts(luma_samples,
                                  params.bev_boundary,
                                  min_width_m,
                                  row);
    return row;
}

}  // namespace

std::vector<BEVSimpleRowScan> ScanSparseRows(const port::CameraPixelFrameView& frame,
                                             const port::RuntimeParameters& params,
                                             const BEVSampleProjectionLut& lut) {
    std::vector<BEVSimpleRowScan> rows;
    const std::size_t active_sparse_rows = ActiveSparseRowCount(params);
    rows.reserve(active_sparse_rows);
    for (std::size_t index = 0; index < active_sparse_rows; ++index) {
        rows.push_back(ScanSparseRow(frame, params, lut, index));
    }
    return rows;
}

}  // namespace ls2k::vision
