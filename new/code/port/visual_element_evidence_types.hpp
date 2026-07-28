/**
 * @file visual_element_evidence_types.hpp
 * @brief 视觉元素证据类型定义
 *
 * 定义BEV（鸟瞰视角）下的视觉元素检测证据类型。
 * 包含十字路口出口检测、圆形元素（转弯）检测的证据结构，
 * 以及元素候选摘要、边界框、统计支持和运行时参数。
 */

#ifndef LS2K_PORT_VISUAL_ELEMENT_EVIDENCE_TYPES_HPP
#define LS2K_PORT_VISUAL_ELEMENT_EVIDENCE_TYPES_HPP

#include <cstddef>
#include <string>
#include <vector>

namespace ls2k::port {

/**
 * @struct VisualElementCandidateSummary
 * @brief 视觉元素候选摘要
 *
 * 描述一个视觉元素候选是否已构建、是否启用接管、是否被纳入仲裁。
 */
struct VisualElementCandidateSummary {
    bool built = false;                    ///< 是否已构建候选
    bool takeover_enabled = false;         ///< 是否启用接管功能
    bool included_in_arbitration = false;  ///< 是否已纳入仲裁
    std::string reason = "not_built";      ///< 未构建/未纳入的原因
};

/**
 * @struct CrossExitElementEvidence
 * @brief 十字路口出口元素证据
 *
 * 描述 Cross 开口检测的位置范围、稀疏边界事实统计和判定原因。
 */
struct CrossExitElementEvidence {
    bool present = false;              ///< 元素是否存在
    float forward_min_m = 0.0F;        ///< 元素前向最小距离（米）
    float forward_max_m = 0.0F;        ///< 元素前向最大距离（米）
    float lateral_min_m = 0.0F;        ///< 元素横向最小位置（米）
    float lateral_max_m = 0.0F;        ///< 元素横向最大位置（米）
    std::size_t sampleable_count = 0;      ///< 可采样的栅格单元数
    std::size_t boundary_jump_count = 0;    ///< 边界跳变事实数量
    std::size_t boundary_span_count = 0;    ///< 同行边界 span 事实数量
    std::size_t boundary_absent_row_count = 0; ///< opening 支撑行数（字段名保留协议兼容）
    std::string reason = "not_evaluated";  ///< 未评估的原因
};

/**
 * @struct VisualElementEvidenceBounds
 * @brief 视觉元素证据边界
 *
 * 定义元素在BEV坐标系中的空间范围（前向和横向的min/max）。
 */
struct VisualElementEvidenceBounds {
    float forward_min_m = 0.0F;  ///< 前向最小距离（米）
    float forward_max_m = 0.0F;  ///< 前向最大距离（米）
    float lateral_min_m = 0.0F;  ///< 横向最小位置（米）
    float lateral_max_m = 0.0F;  ///< 横向最大位置（米）
};

/**
 * @struct VisualElementEvidenceSupport
 * @brief 视觉元素证据统计支持
 *
 * 统计可采样单元数和 V9 边界事实数量。
 */
struct VisualElementEvidenceSupport {
    std::size_t sampleable_count = 0;       ///< 可采样单元总数
    std::size_t boundary_jump_count = 0;     ///< V9 边界跳变事实数量
    std::size_t boundary_span_count = 0;     ///< V9 边界 span 事实数量
};

/**
 * @struct VisualElementEvidenceRecord
 * @brief 单个视觉元素证据记录
 *
 * 包含元素的完整证据信息：ID、存在性、置信度、边界、统计支持和候选状态。
 */
struct VisualElementEvidenceRecord {
    std::string id{};                     ///< 元素ID
    bool present = false;                 ///< 元素是否存在
    float confidence = 0.0F;              ///< 检测置信度
    std::string reason = "not_evaluated";  ///< 评估结果原因
    VisualElementEvidenceBounds bounds{};  ///< 元素空间边界
    VisualElementEvidenceSupport support{};  ///< 统计支持数据
    VisualElementCandidateSummary candidate{};  ///< 候选摘要
};

/**
 * @struct VisualElementEvidenceFrame
 * @brief 一帧中的所有视觉元素证据
 *
 * 包含十字路口出口证据和通用元素记录列表。
 */
struct VisualElementEvidenceFrame {
    CrossExitElementEvidence cross_exit{};          ///< 十字路口出口证据
    std::vector<VisualElementEvidenceRecord> records{};  ///< 通用元素记录列表
};

/**
 * @struct BEVElementParameters
 * @brief BEV元素检测的运行参数
 *
 * 控制十字路口出口检测和 Circle V2 场景注册/退出门限。
 */
struct BEVElementParameters {
    // 十字路口出口检测参数
    int cross_min_sampleable_per_row = 8;       ///< cross 判定每行最少可采样点数
    int cross_connectivity_sample_index = 9;    ///< 原点连通性目标的 BEV 前向采样点索引，[0,23]
    float cross_boundary_expansion_min_m = 0.055F;  ///< 单侧边界相对前段基线的最小向外垂距

    // Circle V2 场景状态机参数
    bool circle_v2_enabled = true;                  ///< 是否注册 CircleV2Scene
    float circle_v2_normal_trace_start_yaw_deg = 90.0F;
    float circle_v2_exit_trace_start_yaw_deg = 270.0F;
    float circle_v2_calm_fallback_yaw_deg = 340.0F;
    int circle_v2_calm_trace_ms = 1000;
    int circle_v2_cooldown_ms = 3000;
    int circle_v2_inner_trace_stall_timeout_ms = 2000; ///< InnerTrace 无明显 yaw 积分退回 Idle 超时
    float circle_v2_inner_trace_stall_yaw_min_deg = 60.0F; ///< InnerTrace 超时退回 Idle 的最小明显 yaw 积分
    float circle_v2_inner_trace_path_offset_m = 0.0F; ///< InnerTrace 从内圆边线向道路内部偏移的距离
    float circle_v2_opposite_straight_confidence_min = 0.70F; ///< CircleV2 对侧直线最低拟合置信度
    float circle_v2_min_sampleable_width_m = 0.35F;
    float circle_v2_opening_forward_min_m = 0.05F;
    float circle_v2_opening_forward_max_m = 1.50F;
    float circle_v2_opening_distance_min_m = 0.055F;
    float circle_v2_opening_confirm_forward_span_m = 0.10F;
    float circle_v2_entry_forward_min_m = 0.10F;
    float circle_v2_entry_forward_max_m = 0.50F;
    float circle_v2_inner_geometry_forward_min_m = 0.05F;
    float circle_v2_inner_geometry_forward_max_m = 0.50F;
    float circle_v2_exit_geometry_forward_min_m = 0.05F;
    float circle_v2_exit_geometry_forward_max_m = 0.50F;
    float circle_v2_exit_straight_max_lateral_span_m = 0.13F;
    float circle_v2_exit_tangent_fit_span_m = 0.10F;
};

}  // namespace ls2k::port

#endif  // LS2K_PORT_VISUAL_ELEMENT_EVIDENCE_TYPES_HPP
