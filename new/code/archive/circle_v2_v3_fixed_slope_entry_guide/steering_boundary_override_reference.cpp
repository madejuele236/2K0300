#include "runtime/steering_boundary_override_reference.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace ls2k::runtime {
namespace {

constexpr std::size_t kMinLeadingReferenceSamples = 3U;
constexpr float kWhiteContainmentToleranceM = 1.0e-4F;

bool IsFiniteSample(const port::BEVPathSample& sample) {
    return sample.present &&
           std::isfinite(sample.point.forward_m) &&
           std::isfinite(sample.point.lateral_m);
}

bool LateralInsideInterval(const vision::BEVSimpleWhiteInterval& interval,
                           float lateral_m) {
    return lateral_m + kWhiteContainmentToleranceM >= interval.left_m &&
           lateral_m - kWhiteContainmentToleranceM <= interval.right_m;
}

bool LateralInsideAnyWhiteInterval(const vision::BEVSimpleRowScan& row,
                                   float lateral_m) {
    for (const vision::BEVSimpleWhiteInterval& interval : row.intervals) {
        if (LateralInsideInterval(interval, lateral_m)) {
            return true;
        }
    }
    return false;
}

const vision::BEVSimpleWhiteInterval* FindBaseInterval(
    const vision::BEVSimpleRowScan& row,
    float ordinary_lateral_m) {
    const vision::BEVSimpleWhiteInterval* best = nullptr;
    float best_cost = 0.0F;
    for (const vision::BEVSimpleWhiteInterval& interval : row.intervals) {
        if (LateralInsideInterval(interval, ordinary_lateral_m)) {
            return &interval;
        }
        const float cost = std::fabs(interval.center_m - ordinary_lateral_m);
        if (best == nullptr || cost < best_cost) {
            best = &interval;
            best_cost = cost;
        }
    }
    return best;
}

bool BuildPatchedInterval(const vision::BEVSimpleWhiteInterval& base,
                          BoundaryOverrideSide side,
                          float boundary_lateral_m,
                          float min_width_m,
                          vision::BEVSimpleWhiteInterval& patched) {
    patched = base;
    if (side == BoundaryOverrideSide::kLeft) {
        patched.left_m = boundary_lateral_m;
    } else {
        patched.right_m = boundary_lateral_m;
    }
    patched.width_m = patched.right_m - patched.left_m;
    if (!std::isfinite(patched.width_m) || patched.width_m < min_width_m) {
        return false;
    }
    patched.center_m = 0.5F * (patched.left_m + patched.right_m);
    return std::isfinite(patched.center_m);
}

void StopReferenceAt(std::vector<vision::BEVSimpleRowScan>& rows,
                     std::size_t index) {
    if (index < rows.size()) {
        rows[index].intervals.clear();
    }
}

std::size_t ApplyBoundaryOverride(std::vector<vision::BEVSimpleRowScan>& patched_rows,
                                  const port::BEVReferencePath& ordinary_reference,
                                  const BoundaryOverrideRequest& request,
                                  const port::RuntimeParameters& params) {
    const float min_width_m = std::max(0.02F, params.bev_geometry.lateral_step_m * 1.5F);
    std::size_t patched_count = 0;
    for (std::size_t index = 0;
         index < patched_rows.size() && index < request.boundary_path.sampled_path.size() &&
         index < ordinary_reference.sampled_path.size();
         ++index) {
        vision::BEVSimpleRowScan& row = patched_rows[index];
        const port::BEVPathSample& boundary_sample =
            request.boundary_path.sampled_path[index];
        const port::BEVPathSample& ordinary_sample =
            ordinary_reference.sampled_path[index];
        if (!row.valid || row.intervals.empty() ||
            !IsFiniteSample(boundary_sample) ||
            !IsFiniteSample(ordinary_sample)) {
            StopReferenceAt(patched_rows, index);
            break;
        }

        const vision::BEVSimpleWhiteInterval* base =
            FindBaseInterval(row, ordinary_sample.point.lateral_m);
        if (base == nullptr) {
            StopReferenceAt(patched_rows, index);
            break;
        }

        vision::BEVSimpleWhiteInterval patched{};
        if (!BuildPatchedInterval(*base,
                                  request.side,
                                  boundary_sample.point.lateral_m,
                                  min_width_m,
                                  patched) ||
            !LateralInsideAnyWhiteInterval(row, patched.center_m)) {
            StopReferenceAt(patched_rows, index);
            break;
        }

        row.intervals.clear();
        row.intervals.push_back(patched);
        ++patched_count;
    }
    return patched_count;
}

std::size_t KeepWhiteLeadingSegment(port::BEVReferencePath& reference,
                                    const std::vector<vision::BEVSimpleRowScan>& rows) {
    std::size_t leading_count = 0;
    for (std::size_t index = 0; index < reference.sampled_path.size(); ++index) {
        port::BEVPathSample& sample = reference.sampled_path[index];
        if (!IsFiniteSample(sample) ||
            index >= rows.size() ||
            !LateralInsideAnyWhiteInterval(rows[index], sample.point.lateral_m)) {
            for (std::size_t clear_index = index;
                 clear_index < reference.sampled_path.size();
                 ++clear_index) {
                reference.sampled_path[clear_index].present = false;
            }
            break;
        }
        ++leading_count;
    }
    if (leading_count == 0U) {
        reference.mode = port::ReferenceMode::kNone;
    }
    return leading_count;
}

}  // namespace

std::optional<port::BEVReferencePath> BuildReferencePathWithBoundaryOverride(
    const std::vector<vision::BEVSimpleRowScan>& rows,
    const BoundaryOverrideRequest& request,
    const port::RuntimeParameters& params) {
    std::vector<vision::BEVSimpleRowScan> patched_rows = rows;
    const port::BEVReferencePath ordinary_reference =
        vision::BuildReferencePath(rows, params);
    const std::size_t patched_count =
        ApplyBoundaryOverride(patched_rows, ordinary_reference, request, params);
    if (patched_count < kMinLeadingReferenceSamples) {
        return std::nullopt;
    }

    port::BEVReferencePath reference =
        vision::BuildReferencePath(patched_rows, params);
    const std::size_t white_count = KeepWhiteLeadingSegment(reference, rows);
    if (white_count < kMinLeadingReferenceSamples) {
        return std::nullopt;
    }
    return reference;
}

}  // namespace ls2k::runtime
