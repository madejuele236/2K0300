#ifndef LS2K_RUNTIME_STEERING_FRAME_PERCEPTION_PIPELINE_HPP
#define LS2K_RUNTIME_STEERING_FRAME_PERCEPTION_PIPELINE_HPP

#include "vision/bev/bev_projector.hpp"
#include "vision/bev/bev_sample_projection_lut.hpp"
#include "vision/ml/red_rectangle_detector.hpp"
#include "vision/ml/selected_ml_classifier.hpp"
#include "vision/image/illumination_binary_model.hpp"
#include "port/diagnostics.hpp"
#include "port/motion_history_types.hpp"
#include "port/perception_result.hpp"
#include "port/platform_adapter.hpp"
#include "port/steering_state_types.hpp"

namespace ls2k::runtime {

/// 转向帧感知管线 —— 单帧图像感知处理管线。
/// 包含 BEV 边界事实、视觉元素检测、视觉参考选择与连续性跟踪。
class SteeringFramePipeline {
public:
    /// 配置感知管线：初始化 BEV 投影器、重置采样 LUT
    /// @param params       运行时参数
    /// @param diagnostics  诊断输出接口
    /// @return             配置是否成功
    bool Configure(const port::RuntimeParameters& params,
                   port::DiagnosticSink& diagnostics);
    /// 重置参考连续性与 ML scene 记忆（CircleV2 contract 保持不变）
    void ResetReferenceMemory();
    /// 处理一帧图像：BEV 边界事实 → 元素检测 → 参考选择 → 横向误差计算
    /// @param capture   相机捕获数据
    /// @param params    运行时参数
    /// @param motion_history  控制侧运动历史快照
    /// @return          处理后的感知结果
    port::PerceptionResult ProcessFrame(const port::CameraCapture& capture,
                                        const port::RuntimeParameters& params,
                                        const port::MotionHistory& motion_history);

private:
    vision::BEVProjector projector_{};                          ///< BEV 投影器
    vision::BEVSampleProjectionLut sample_lut_{};               ///< 采样投影查找表
    vision::ml::MlRedRectangleProjectionLut ml_rectangle_lut_{}; ///< ML 红框网格投影表
    vision::ml::SelectedMlClassifier ml_classifier_{};          ///< 编译期选中的唯一 ML 分类器
    vision::BinaryModelTracker binary_model_tracker_{};          ///< 最多跨3个无效帧的统一空间二值模型 owner
    port::SteeringPerceptionMemory perception_memory_{};        ///< 感知记忆（参考连续性）
    bool projector_configured_ = false;                         ///< 投影器是否已配置
    bool ml_classifier_ready_ = false;                          ///< 选中分类器初始化状态
};

}  // namespace ls2k::runtime

#endif  // LS2K_RUNTIME_STEERING_FRAME_PERCEPTION_PIPELINE_HPP
