#include "reference/reference_usability.hpp"

#include <algorithm>
#include <cstddef>

#include "port/bev_reference_path_utils.hpp"

namespace ls2k::reference {
namespace {

/// 从运行时参数中获取配置的最小前导参考采样点数（钳制在有效范围内）
std::size_t ConfiguredMinLeadingSamples(const port::RuntimeParameters& params) {
    constexpr int kMinSamplesForControl = 3;
    const int bounded =
        std::clamp(params.bev_control_model.min_leading_reference_samples,
                   kMinSamplesForControl,
                   static_cast<int>(port::kBevReferenceSampleCount));
    return static_cast<std::size_t>(bounded);
}

}  // namespace

/// EvaluateReferenceUsability 实现
/// 统计整条有序参考中的有限有效样本及其前向范围。
/// 任一坏点只从集合中退出；剩余点数不足才使路径不可用。
port::ReferenceUsability EvaluateReferenceUsability(const port::BEVReferencePath& reference_path,
                                                    const port::RuntimeParameters& params) {
    port::ReferenceUsability usability{};
    const std::size_t min_leading_samples = ConfiguredMinLeadingSamples(params);
    for (const port::BEVPathSample& sample : reference_path.sampled_path) {
        if (!port::IsFiniteReferenceSample(sample)) {
            continue;
        }
        if (usability.leading_usable_samples == 0) {
            usability.leading_min_forward_m = sample.point.forward_m;
            usability.leading_max_forward_m = sample.point.forward_m;
        } else {
            usability.leading_min_forward_m =
                std::min(usability.leading_min_forward_m, sample.point.forward_m);
            usability.leading_max_forward_m =
                std::max(usability.leading_max_forward_m, sample.point.forward_m);
        }
        ++usability.leading_usable_samples;
    }

    if (usability.leading_usable_samples == 0) {
        usability.reason = "no_reference_facts";
        return usability;
    }
    if (usability.leading_usable_samples < min_leading_samples) {
        usability.reason = "insufficient_leading_reference";
        return usability;
    }

    usability.usable = true;
    usability.reason = "ok";
    return usability;
}

}  // namespace ls2k::reference
