#ifndef LS2K_PORT_BEV_REFERENCE_PATH_UTILS_HPP
#define LS2K_PORT_BEV_REFERENCE_PATH_UTILS_HPP

#include <cmath>
#include <cstddef>

#include "port/bev_reference_types.hpp"

namespace ls2k::port {

inline bool IsFiniteReferenceSample(const BEVPathSample& sample) {
    return sample.present &&
           std::isfinite(sample.point.forward_m) &&
           std::isfinite(sample.point.lateral_m);
}

inline std::size_t CountFiniteReferenceSamples(const BEVReferencePath& path) {
    std::size_t count = 0U;
    for (const BEVPathSample& sample : path.sampled_path) {
        count += IsFiniteReferenceSample(sample) ? 1U : 0U;
    }
    return count;
}

inline const BEVPathSample* FirstFiniteReferenceSample(const BEVReferencePath& path) {
    for (const BEVPathSample& sample : path.sampled_path) {
        if (IsFiniteReferenceSample(sample)) {
            return &sample;
        }
    }
    return nullptr;
}

// A reference path is an ordered set of valid samples. Invalid slots carry no
// ordering semantics, so normalize them out before handing the path to a
// consumer that stores samples as a compact fixed-capacity array.
inline std::size_t CompactFiniteReferenceSamples(BEVReferencePath& path) {
    std::size_t output_index = 0U;
    for (std::size_t index = 0U; index < path.sampled_path.size(); ++index) {
        const BEVPathSample sample = path.sampled_path[index];
        if (!IsFiniteReferenceSample(sample)) {
            continue;
        }
        path.sampled_path[output_index++] = sample;
    }
    for (std::size_t index = output_index; index < path.sampled_path.size(); ++index) {
        path.sampled_path[index] = {};
    }
    if (output_index == 0U) {
        path.mode = ReferenceMode::kNone;
    }
    return output_index;
}

}  // namespace ls2k::port

#endif  // LS2K_PORT_BEV_REFERENCE_PATH_UTILS_HPP
