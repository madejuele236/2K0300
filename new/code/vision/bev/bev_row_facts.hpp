#ifndef LS2K_VISION_BEV_ROW_FACTS_HPP
#define LS2K_VISION_BEV_ROW_FACTS_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ls2k::vision {

enum class BEVBoundaryJumpPolarity {
    kRisingY,
    kFallingY,
};

struct BEVRowLumaSample {
    bool sampleable = false;
    float forward_m = 0.0F;
    float lateral_m = 0.0F;
    int lateral_index = 0;
    std::uint8_t y = 0U;
};

struct BEVBoundaryJump {
    float forward_m = 0.0F;
    float lateral_m = 0.0F;
    int lateral_index = 0;
    int delta_y = 0;
    BEVBoundaryJumpPolarity polarity = BEVBoundaryJumpPolarity::kRisingY;
};

struct BEVBoundarySpan {
    float forward_m = 0.0F;
    float left_m = 0.0F;
    float right_m = 0.0F;
    float center_m = 0.0F;
    float width_m = 0.0F;
    int left_lateral_index = 0;
    int right_lateral_index = 0;
};

/// BEV稀疏行扫描结果，包含 V9 局部 Y 边界事实
struct BEVSimpleRowScan {
    bool valid = false;               ///< 扫描行是否有效
    float forward_m = 0.0F;           ///< 此行对应的前向距离（米）
    int row_px = 0;                   ///< 在采样行数组中的索引
    std::size_t sampleable_count = 0; ///< 可采样的横向样本总数
    std::size_t unavailable_count = 0;///< 不可用像素计数
    float sampleable_left_m = 0.0F;   ///< 可采样区域横向低坐标边界（米）
    float sampleable_right_m = 0.0F;  ///< 可采样区域横向高坐标边界（米）
    float sampleable_width_m = 0.0F;  ///< 可采样区域宽度（米）
    std::vector<BEVBoundaryJump> jumps{};      ///< V9 局部 Y 边界跳变事实
    std::vector<BEVBoundarySpan> spans{};      ///< V9 同行配对边界 span 事实
};

}  // namespace ls2k::vision

#endif  // LS2K_VISION_BEV_ROW_FACTS_HPP
