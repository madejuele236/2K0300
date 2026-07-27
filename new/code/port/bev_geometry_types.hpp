/**
 * @file bev_geometry_types.hpp
 * @brief BEV几何类型定义
 *
 * 定义BEV（鸟瞰视角）投影所需的几何数据类型，包括：
 * - 图像坐标点与BEV坐标点的结构
 * - 投影标定参数（四点透视变换）
 * - 前向采样网格参数
 * - 分类与控制模型参数
 */

#ifndef LS2K_PORT_BEV_GEOMETRY_TYPES_HPP
#define LS2K_PORT_BEV_GEOMETRY_TYPES_HPP

#include <array>
#include <cmath>
#include <cstddef>
#include <string>

namespace ls2k::port {

/**
 * @struct ImagePoint
 * @brief 图像坐标系中的点（行/列坐标，单位为像素）
 */
struct ImagePoint {
    float row_px = 0.0F;  ///< 行坐标（像素）
    float col_px = 0.0F;  ///< 列坐标（像素）
};

/**
 * @struct BEVPoint
 * @brief BEV坐标系中的点（前向/横向，单位为米）
 */
struct BEVPoint {
    float forward_m = 0.0F;  ///< 前向距离（米），沿车辆纵轴方向
    float lateral_m = 0.0F;  ///< 横向距离（米），沿车辆横轴方向（右正左负）
};

constexpr std::size_t kBevCalibrationPointCount = 4;  ///< 透视变换标定点数量
constexpr std::size_t kBevReferenceSampleCount = 24;   ///< 参考路径采样点数量

/**
 * @struct BEVProjectorCalibration
 * @brief BEV投影器标定参数
 *
 * 包含从图像到BEV平面的四点透视变换标定数据，
 * 以及投影器的标识信息和调试网格尺寸。
 */
struct BEVProjectorCalibration {
    bool valid = true;  ///< 标定是否有效
    std::array<ImagePoint, kBevCalibrationPointCount> source_points{  ///< 源图像上的四个标定点（像素坐标）
        {ImagePoint{219.0F, 55.5F},
         ImagePoint{219.0F, 307.5F},
         ImagePoint{56.0F, 137.5F},
         ImagePoint{56.0F, 226.5F}}};
    std::array<BEVPoint, kBevCalibrationPointCount> target_points{  ///< BEV平面上的对应目标点（米坐标）
        {BEVPoint{0.0795685735F, -0.2316051101F},
         BEVPoint{0.0795685735F, 0.2316051101F},
         BEVPoint{0.7834243479F, -0.2316051101F},
         BEVPoint{0.7834243479F, 0.2316051101F}}};
    int debug_grid_width = 160;   ///< 调试栅格宽度（像素）
    int debug_grid_height = 128;  ///< 调试栅格高度（像素）
    std::string projector_id = "bev_projector_red_marker_metric_20260713T171917Z";  ///< 投影器唯一标识
    std::string projector_hash = "bev-projector-red-marker-0p12x0p05-20260713T171917Z";   ///< 投影器哈希版本
};

/**
 * @struct BEVGeometryParameters
 * @brief BEV几何参数
 *
 * 定义参考路径的前向采样网格和横向搜索范围。
 * forward_samples_m 数组定义了 BEV 投影后的车辆坐标系前向采样位置。
 * 消费方直接按米制 forward_m 使用这些值，不需要再额外做 BEV 转换。
 * sparse_row_count 只控制启用前缀长度，不重新分布采样行。
 */
struct BEVGeometryParameters {
    std::array<float, kBevReferenceSampleCount> forward_samples_m{  ///< 24个前向采样位置（米），从近到远
        {0.130440284F,
         0.215509525F,
         0.300580069F,
         0.385649310F,
         0.470719854F,
         0.555789094F,
         0.640858335F,
         0.725928879F,
         0.810998120F,
         0.896068664F,
         0.981137905F,
         1.066207145F,
         1.151277689F,
         1.236346930F,
         1.321416170F,
         1.406486715F,
         1.491555955F,
         1.576626500F,
         1.661695740F,
         1.746764980F,
         1.831835525F,
         1.916904765F,
         2.001975310F,
         2.087044550F}};
    int sparse_row_count = static_cast<int>(kBevReferenceSampleCount);  ///< 启用原始前向采样行的前 N 行
    float search_lateral_limit_m = 1.764610363F;  ///< 横向搜索范围限制（米）
    float lateral_step_m = 0.022057630F;          ///< 横向搜索步长（米）
    float boundary_trace_max_adjacent_distance_m = 0.195660427F;  ///< 边界 trace 相邻保留点最大距离（米）
    float nominal_road_half_width_m = 0.225F;  ///< 实测赛道半宽（米）
};

/**
 * @struct BEVClassificationParameters
 * @brief BEV分类参数
 *
 * 控制视觉元素分类的置信度阈值和历史保持策略。
 */
struct BEVClassificationParameters {
    float white_confidence_min = 0.55F;  ///< 白色元素的最小置信度阈值
    float unknown_confidence_min = 0.25F;  ///< 无法确定类别的置信度阈值
    int hold_last_max_cycles = 32;  ///< 最近一次识别结果的最大保持周期数；0 表示禁用 hold
};

inline bool IsValidBEVClassificationParameters(
    const BEVClassificationParameters& params) {
    return std::isfinite(params.unknown_confidence_min) &&
           std::isfinite(params.white_confidence_min) &&
           params.unknown_confidence_min > 0.0F &&
           params.white_confidence_min >= params.unknown_confidence_min &&
           params.white_confidence_min <= 1.0F &&
           params.hold_last_max_cycles >= 0;
}

/**
 * @struct BEVControlModelParameters
 * @brief BEV控制模型参数
 *
 * 将从感知到控制的映射参数化，包括横向误差的加权和PID增益等。
 */
struct BEVControlModelParameters {
    double lateral_offset_to_wheel_delta_gain = 136.007362;  ///< 横向位置项到轮速差值的增益系数
    double heading_error_to_wheel_delta_gain = 59.136130;  ///< 航向误差项到轮速差值的增益系数
    double curvature_to_wheel_delta_gain = 23.141201;  ///< nominal speed下曲率前馈项到轮速差值的增益系数
    int min_leading_reference_samples = 3;  ///< 最小有限有效参考采样点数量（字段名保留兼容）
    int tracking_fit_min_samples = 3;       ///< 跟踪几何拟合最小采样点数量
};

}  // namespace ls2k::port

#endif  // LS2K_PORT_BEV_GEOMETRY_TYPES_HPP
