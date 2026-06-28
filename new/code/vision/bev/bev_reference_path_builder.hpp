#ifndef LS2K_VISION_BEV_REFERENCE_PATH_BUILDER_HPP
#define LS2K_VISION_BEV_REFERENCE_PATH_BUILDER_HPP

#include <vector>

#include "port/bev_reference_types.hpp"
#include "port/runtime_parameter_types.hpp"
#include "vision/bev/bev_row_facts.hpp"

namespace ls2k::vision {

/// 从行扫描结果中提取第一个连续参考段（允许近端缺失，输出点保留真实 forward_m）
/// @param rows 行扫描结果数组
/// @param params 运行时参数
/// @return 提取的参考路径
port::BEVReferencePath ExtractStrictLeadingReferenceSegment(
    const std::vector<BEVSimpleRowScan>& rows,
    const port::RuntimeParameters& params);

/// 从行扫描结果构建完整的参考路径（目前委托给ExtractStrictLeadingReferenceSegment）
port::BEVReferencePath BuildReferencePath(const std::vector<BEVSimpleRowScan>& rows,
                                          const port::RuntimeParameters& params);

}  // namespace ls2k::vision

#endif  // LS2K_VISION_BEV_REFERENCE_PATH_BUILDER_HPP
