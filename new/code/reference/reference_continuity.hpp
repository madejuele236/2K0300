#ifndef LS2K_REFERENCE_REFERENCE_CONTINUITY_HPP
#define LS2K_REFERENCE_REFERENCE_CONTINUITY_HPP

#include <cstdint>
#include <string>

#include "port/bev_reference_types.hpp"
#include "port/reference_tracking_geometry_types.hpp"
#include "port/reference_usability_types.hpp"
#include "port/runtime_parameter_types.hpp"

namespace ls2k::reference {

port::ReferenceHoldState MakeReferenceHoldState(
    const port::BEVReferencePath& current_visual_reference,
    uint64_t reference_capture_time_ms,
    const port::RuntimeParameters& params);

inline port::ReferenceHoldState MakeReferenceHoldState(
    const port::BEVReferencePath& current_visual_reference,
    const port::RuntimeParameters& params) {
    return MakeReferenceHoldState(current_visual_reference, 0, params);
}

port::ReferenceContinuityResult BuildReferenceHoldCandidate(
    const port::ReferenceHoldState& prior_hold,
    const port::RuntimeParameters& params);

struct ReferenceContinuitySelection {
    port::ReferenceContinuityResult continuity{};          ///< 最终选中的 current/hold reference
    port::ReferenceUsability usability{};                  ///< 最终 reference 的可用性
    port::ReferenceTrackingGeometry tracking_geometry{};  ///< 最终 reference 的跟踪几何
};

/// 以 tracking geometry 是否可计算为提交边界，选择当前参考或最近有效 HOLD。
ReferenceContinuitySelection ResolveReferenceContinuity(
    const port::BEVReferencePath& current_visual_reference,
    const std::string& current_source,
    uint64_t reference_capture_time_ms,
    const port::ReferenceHoldState& prior_hold,
    const port::RuntimeParameters& params);

}  // namespace ls2k::reference

#endif  // LS2K_REFERENCE_REFERENCE_CONTINUITY_HPP
