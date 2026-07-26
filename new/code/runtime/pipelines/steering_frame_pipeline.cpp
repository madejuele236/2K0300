#include "runtime/pipelines/steering_frame_pipeline.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <string>

#include "reference/reference_control_readiness.hpp"
#include "reference/reference_continuity.hpp"
#include "reference/reference_lateral_error.hpp"
#include "reference/reference_tracking_geometry.hpp"
#include "reference/reference_usability.hpp"
#include "vision/elements/visual_element_pipeline.hpp"
#include "reference/visual_reference_orchestration.hpp"
#include "port/perf_counter.hpp"
#include "runtime/runtime_state.hpp"
#include "vision/bev/bev_simple_perception.hpp"
#include "vision/elements/circle_v2/circle_v2_reference_adapter.hpp"
#include "vision/elements/circle_v2/circle_v2_scene.hpp"
#include "vision/ml/ml_scene.hpp"

namespace ls2k::runtime {

using vision::AdaptCircleV2ReferencePlan;
using vision::CircleV2Params;
using vision::CircleV2Scene;
using vision::CircleV2StepResult;
using vision::CircleV2Telemetry;
using vision::CirclePhase;
using vision::ConstArrayView;
using vision::MotionArcView;
using vision::OrdinaryRoadModel;
using vision::ResetCircleV2Memory;
using vision::SceneFrameView;

namespace {

constexpr float kPi = 3.14159265358979323846F;

/// 构建感知结果结构：从各感知子阶段的结果组装最终的 PerceptionResult
/// @param capture              相机捕获数据
/// @param threshold             Otsu 二值化阈值
/// @param health                感知健康状态
/// @param element_evidence      视觉元素证据帧
/// @param visual_selection      视觉参考选择结果
/// @param continuity            参考连续性结果
/// @param selected_usability    选中参考的可用性
/// @param lateral_error         参考横向误差估计
/// @param reference_tracking_geometry     参考跟踪几何事实
/// @param reference_control     参考控制就绪状态
/// @param publish_time_ms       发布时间戳
/// @return                      组装好的 PerceptionResult
port::PerceptionResult BuildPerceptionResult(
    const port::CameraCapture& capture,
    const vision::BEVSimplePerceptionResult& boundary_facts,
    const port::PerceptionHealth& health,
    const port::VisualElementEvidenceFrame& element_evidence,
    const port::CircleV2TelemetrySnapshot& circle_v2,
    const port::MlTelemetrySnapshot& ml,
    const port::VisualReferenceCandidatePathSet& candidate_paths,
    const port::VisualReferenceSelection& visual_selection,
    const port::ReferenceContinuityResult& continuity,
    const port::ReferenceUsability& selected_usability,
    const port::ReferenceLateralErrorEstimate& lateral_error,
    const port::ReferenceTrackingGeometry& reference_tracking_geometry,
    const port::ReferenceControlReadiness& reference_control,
    uint64_t publish_time_ms) {
    port::PerceptionResult perception{};
    perception.published = true;
    perception.fresh = true;
    perception.frame_id = capture.frame_id;
    perception.capture_time_ms = capture.capture_time_ms;
    perception.publish_time_ms = publish_time_ms;
    perception.otsu = boundary_facts.otsu;
    perception.perception_tag = "bev_sparse_otsu_binary";
    perception.boundary_row_count = boundary_facts.rows.size();
    perception.boundary_jump_count = boundary_facts.boundary_jump_count;
    perception.boundary_span_count = boundary_facts.boundary_span_count;
    perception.reference_mode = vision::ToString(continuity.mode);
    perception.reference_source = continuity.source;
    perception.reference_capture_time_ms = continuity.reference_capture_time_ms;
    perception.reference_path = continuity.reference_path;
    perception.perception_health = health;
    perception.element_evidence = element_evidence;
    perception.circle_v2 = circle_v2;
    perception.ml = ml;
    perception.visual_reference_candidate_paths = candidate_paths;
    perception.visual_reference_selection = visual_selection;
    perception.reference_usability = selected_usability;
    perception.reference_lateral_error = lateral_error;
    perception.reference_tracking_geometry = reference_tracking_geometry;
    perception.reference_control = reference_control;
    return perception;
}

bool HasLeadingCenterPath(const port::BEVReferencePath& path) {
    if (path.mode == port::ReferenceMode::kNone || !path.sampled_path[0].present) {
        return false;
    }
    for (const port::BEVPathSample& sample : path.sampled_path) {
        if (!sample.present) {
            break;
        }
        if (!std::isfinite(sample.point.forward_m) ||
            !std::isfinite(sample.point.lateral_m)) {
            return false;
        }
        return true;
    }
    return false;
}

std::optional<OrdinaryRoadModel> BuildOrdinaryRoadModel(
    const vision::BEVSimplePerceptionResult& facts,
    float nominal_road_half_width_m) {
    OrdinaryRoadModel model{};
    model.center_path = facts.reference_path;
    if (!HasLeadingCenterPath(model.center_path)) {
        return std::nullopt;
    }
    model.half_width.value_m = nominal_road_half_width_m;
    return model;
}

bool IntegrateYawForCircle(const port::MotionHistory& history,
                           uint64_t start_ms,
                           uint64_t end_ms,
                           int max_gap_ms,
                           float& delta_yaw_rad) {
    delta_yaw_rad = 0.0F;
    if (end_ms <= start_ms) {
        return end_ms >= start_ms;
    }
    if (history.count < 2U) {
        return false;
    }
    uint64_t covered_until_ms = start_ms;
    const uint64_t max_gap = static_cast<uint64_t>(std::max(1, max_gap_ms));
    for (std::size_t index = 1; index < history.count; ++index) {
        const port::MotionHistorySample& prev = history.OldestOffset(index - 1U);
        const port::MotionHistorySample& curr = history.OldestOffset(index);
        if (curr.time_ms <= covered_until_ms) {
            continue;
        }
        if (prev.time_ms > covered_until_ms || prev.time_ms >= end_ms) {
            if (covered_until_ms == start_ms && prev.time_ms < end_ms) {
                covered_until_ms = prev.time_ms;
            } else {
                break;
            }
        }
        if (!prev.imu_valid || !curr.imu_valid || curr.time_ms <= prev.time_ms) {
            return false;
        }
        const uint64_t gap_ms = curr.time_ms - prev.time_ms;
        if (gap_ms > max_gap) {
            return false;
        }
        const uint64_t segment_start = std::max(covered_until_ms, prev.time_ms);
        const uint64_t segment_end = std::min(end_ms, curr.time_ms);
        if (segment_end <= segment_start) {
            continue;
        }
        const float dt_s = static_cast<float>(segment_end - segment_start) / 1000.0F;
        delta_yaw_rad += prev.gyro_z * dt_s;
        covered_until_ms = segment_end;
        if (covered_until_ms >= end_ms) {
            return true;
        }
    }
    return covered_until_ms >= end_ms;
}

struct MotionArcQueryContext {
    const port::MotionHistory* history = nullptr;
    int max_gap_ms = 30;
};

bool QueryMotionArcYawDelta(void* context, uint64_t from_ms, uint64_t to_ms, float& out_delta_rad) {
    const MotionArcQueryContext* query = static_cast<const MotionArcQueryContext*>(context);
    if (query == nullptr || query->history == nullptr) {
        return false;
    }
    return IntegrateYawForCircle(*query->history,
                                 from_ms,
                                 to_ms,
                                 query->max_gap_ms,
                                 out_delta_rad);
}

CircleV2Params BuildCircleV2Params(const port::RuntimeParameters& params) {
    CircleV2Params circle_params{};
    circle_params.exit_yaw_threshold_rad =
        params.bev_element.circle_v2_exit_yaw_threshold_deg * kPi / 180.0F;
    circle_params.exit_hold_frames =
        std::max(2, params.bev_element.circle_v2_exit_hold_frames);
    circle_params.inner_trace_stall_timeout_ms =
        std::max(1, params.bev_element.circle_v2_inner_trace_stall_timeout_ms);
    circle_params.inner_trace_stall_yaw_min_rad =
        params.bev_element.circle_v2_inner_trace_stall_yaw_min_deg * kPi / 180.0F;
    circle_params.inner_trace_path_offset_m =
        params.bev_element.circle_v2_inner_trace_path_offset_m;
    circle_params.opposite_straight_confidence_min =
        params.bev_element.circle_v2_opposite_straight_confidence_min;
    circle_params.max_adjacent_distance_m =
        params.bev_geometry.boundary_trace_max_adjacent_distance_m;
    circle_params.min_sampleable_width_m = params.bev_element.circle_v2_min_sampleable_width_m;
    circle_params.opening_forward_min_m = params.bev_element.circle_v2_opening_forward_min_m;
    circle_params.opening_forward_max_m = params.bev_element.circle_v2_opening_forward_max_m;
    circle_params.opening_distance_min_m = params.bev_element.circle_v2_opening_distance_min_m;
    circle_params.opening_confirm_forward_span_m =
        params.bev_element.circle_v2_opening_confirm_forward_span_m;
    circle_params.entry_forward_min_m = params.bev_element.circle_v2_entry_forward_min_m;
    circle_params.entry_forward_max_m = params.bev_element.circle_v2_entry_forward_max_m;
    circle_params.inner_geometry_forward_min_m =
        params.bev_element.circle_v2_inner_geometry_forward_min_m;
    circle_params.inner_geometry_forward_max_m =
        params.bev_element.circle_v2_inner_geometry_forward_max_m;
    circle_params.exit_geometry_forward_min_m =
        params.bev_element.circle_v2_exit_geometry_forward_min_m;
    circle_params.exit_geometry_forward_max_m =
        params.bev_element.circle_v2_exit_geometry_forward_max_m;
    circle_params.exit_straight_max_lateral_span_m =
        params.bev_element.circle_v2_exit_straight_max_lateral_span_m;
    return circle_params;
}

port::CircleV2TelemetrySnapshot BuildCircleV2TelemetrySnapshot(bool enabled,
                                                               const CircleV2Telemetry& telemetry) {
    port::CircleV2TelemetrySnapshot snapshot{};
    snapshot.enabled = enabled;
    snapshot.frame_phase = vision::ToString(telemetry.frame_phase);
    snapshot.next_phase = vision::ToString(telemetry.next_phase);
    snapshot.dir = vision::ToString(telemetry.dir);
    snapshot.reference_role = vision::ToString(telemetry.reference_role);
    snapshot.reason = vision::ToString(telemetry.reason);
    snapshot.motion_arc_available = telemetry.motion_arc_available;
    snapshot.geometry_available = telemetry.geometry_available;
    snapshot.inner_trace_elapsed_ms = telemetry.inner_trace_elapsed_ms;
    snapshot.directed_turn_angle_rad = telemetry.directed_turn_angle_rad;
    snapshot.openings = telemetry.openings;
    return snapshot;
}

bool CrossExitSuppressesCircleV2(const vision::VisualElementPipelineResult& element_result,
                                 const port::RuntimeParameters& params) {
    return params.bev_element.cross_exit_takeover_enabled &&
           element_result.evidence.cross_exit.present;
}

}  // namespace

/// 配置感知管线：初始化 BEV 投影器、重置采样 LUT
/// @param params       运行时参数
/// @param diagnostics  诊断输出接口
/// @return             投影器是否配置成功
bool SteeringFramePipeline::Configure(const port::RuntimeParameters& params,
                                      port::DiagnosticSink& diagnostics) {
    projector_configured_ = projector_.Configure(params.bev_projector);
    sample_lut_ = {};
    ml_rectangle_lut_ = {};
    otsu_tracker_.Reset();
    ml_classifier_ready_ = !params.ml.enabled || ml_classifier_.Initialize();
    const std::string ml_classifier_identity =
        !params.ml.enabled
            ? std::string("disabled backend=") + ml_classifier_.BackendName()
            : (ml_classifier_ready_
                   ? std::string("backend=") + ml_classifier_.BackendName() +
                         " artifact_id=" + ml_classifier_.ArtifactId() +
                         " artifact_sha256=" + ml_classifier_.ArtifactSha256() +
                         " artifact_items=" + std::to_string(ml_classifier_.ArtifactItemCount()) +
                         " working_memory_bytes=" + std::to_string(ml_classifier_.WorkingMemoryBytes())
                   : "selected ML classifier initialization failed");
    diagnostics.Emit({projector_configured_ ? port::DiagnosticLevel::kInfo
                                            : port::DiagnosticLevel::kFailSafe,
                      projector_configured_ ? "perception.projector.configured"
                                            : "perception.projector.invalid",
                      projector_configured_ ? "BEV projector configured once for runtime perception"
                                            : "BEV projector configuration failed; perception will publish fail-safe fallback",
                      port::NowMs()});
    diagnostics.Emit({ml_classifier_ready_ ? port::DiagnosticLevel::kInfo
                                           : port::DiagnosticLevel::kFailSafe,
                      ml_classifier_ready_ ? "perception.ml_classifier.configured"
                                           : "perception.ml_classifier.invalid",
                      ml_classifier_identity,
                      port::NowMs()});
    return projector_configured_ && ml_classifier_ready_;
}

/// 重置普通参考连续性记忆（清空 reference hold，不触碰 scene-owned 记忆）
void SteeringFramePipeline::ResetReferenceMemory() {
    ResetSteeringReferenceHoldMemory(perception_memory_);
    vision::ml::ResetMlSceneMemory(perception_memory_.ml_scene);
    otsu_tracker_.Reset();
}

/// 处理一帧图像：V9 BEV 边界事实 → 元素检测 → 视觉参考选择 → 横向误差计算 → 参考控制就绪评估
/// @param capture   相机捕获数据
/// @param params    运行时参数
/// @return          处理后的感知结果
// NOLINTNEXTLINE(readability-function-size): perception pipeline stays as one ordered stage owner to avoid cross-layer authority leakage.
port::PerceptionResult SteeringFramePipeline::ProcessFrame(
    const port::CameraCapture& capture,
    const port::RuntimeParameters& params,
    const port::MotionHistory& motion_history) {
    port::ReferenceContinuityResult continuity{};
    port::ReferenceUsability selected_usability{};
    port::ReferenceLateralErrorEstimate lateral_error{};
    port::ReferenceTrackingGeometry reference_tracking_geometry{};
    port::ReferenceControlReadiness reference_control{};
    port::PerceptionHealth health{};
    port::VisualElementEvidenceFrame element_evidence{};
    port::CircleV2TelemetrySnapshot circle_v2_snapshot{};
    port::MlTelemetrySnapshot ml_snapshot{};
    port::VisualReferenceCandidatePathSet candidate_paths{};
    port::VisualReferenceSelection visual_selection{};
    vision::BEVSimplePerceptionResult current_facts{};
    port::OtsuThresholdState otsu_state{};
    {
        LS2K_PERF_SCOPE(port::PerfStage::kPerceptionOtsu);
        otsu_state = otsu_tracker_.Update(
            vision::ComputeSparseOtsuThreshold(capture.pixel_view));
    }
    {
        LS2K_PERF_SCOPE(port::PerfStage::kPerceptionBev);
        health.projector_ok = projector_.Valid();
        health.reason = health.projector_ok ? "ok" : "projector_invalid";
        const port::SteeringPerceptionMemory prior_memory = perception_memory_;
        {
            LS2K_PERF_SCOPE(port::PerfStage::kBevSimple);
            current_facts =
                vision::RunBEVSimplePerception(capture.pixel_view,
                                               otsu_state,
                                               params,
                                               projector_,
                                               &sample_lut_);
        }
        port::VisualReferenceCandidate line_candidate{};
        line_candidate =
            reference::MakeLineVisualReferenceCandidate(current_facts.reference_path,
                                                        current_facts.reference_source);

        vision::VisualElementPipelineResult element_result{};
        vision::ml::MlSceneInput ml_input{};
        ml_input.frame = &capture.pixel_view;
        ml_input.projector = &projector_;
        ml_input.road_path_facts = &current_facts.road_path_facts;
        ml_input.motion_history = &motion_history;
        ml_input.rectangle_projection_lut = &ml_rectangle_lut_;
        ml_input.classifier = ml_classifier_ready_ ? &ml_classifier_ : nullptr;
        ml_input.capture_time_ms = capture.capture_time_ms;
        const vision::ml::MlSceneResult ml_result =
            vision::ml::StepMlScene(ml_input, params, prior_memory.ml_scene);
        perception_memory_.ml_scene = ml_result.next_memory;
        ml_snapshot = ml_result.telemetry;

        vision::VisualElementPipelineInput element_input{};
        element_input.sparse_rows = &current_facts.rows;
        element_input.origin_to_last_row_midpoint_connectivity =
            current_facts.origin_to_last_row_midpoint_connectivity;
        element_input.line_candidate = line_candidate;
        element_result = vision::RunVisualElementPipeline(element_input, params);
        element_evidence = element_result.evidence;

        std::optional<port::VisualReferenceCandidate> circle_candidate{};
        const bool cross_exit_takeover_active =
            CrossExitSuppressesCircleV2(element_result, params);
        const bool circle_v2_should_step =
            params.bev_element.circle_v2_enabled && !cross_exit_takeover_active;
        if (circle_v2_should_step) {
            MotionArcQueryContext motion_query{};
            motion_query.history = &motion_history;
            motion_query.max_gap_ms = params.reference_time_alignment.max_integration_gap_ms;
            SceneFrameView scene_frame{};
            scene_frame.rows.rows =
                ConstArrayView<vision::BEVSimpleRowScan>(current_facts.rows.data(),
                                                         current_facts.rows.size());
            scene_frame.ordinary_road =
                BuildOrdinaryRoadModel(current_facts,
                                       params.bev_geometry.nominal_road_half_width_m);
            scene_frame.motion_arc = MotionArcView(&motion_query, QueryMotionArcYawDelta);
            scene_frame.stamp.capture_time_ms = capture.capture_time_ms;
            const CircleV2StepResult circle_result =
                CircleV2Scene{}.Step(scene_frame,
                                      prior_memory.circle_v2,
                                      BuildCircleV2Params(params));
            perception_memory_.circle_v2 = circle_result.next_memory;
            circle_v2_snapshot =
                BuildCircleV2TelemetrySnapshot(true, circle_result.telemetry);
            circle_candidate = AdaptCircleV2ReferencePlan(circle_result.reference_plan);
        } else {
            if (prior_memory.circle_v2.phase != CirclePhase::kIdle) {
                ResetCircleV2Memory(perception_memory_.circle_v2);
            }
            CircleV2Telemetry idle_telemetry{};
            circle_v2_snapshot =
                BuildCircleV2TelemetrySnapshot(params.bev_element.circle_v2_enabled,
                                               idle_telemetry);
        }

        port::ReferenceUsability current_usability{};
        {
            std::array<port::VisualReferenceCandidate,
                       port::kVisualReferenceCandidatePathCapacity>
                candidates{};
            std::size_t candidate_count = 0;
            candidates[candidate_count++] = line_candidate;
            if (!ml_result.suppress_other_scene_candidates) {
                for (const port::VisualReferenceCandidate& candidate :
                     element_result.candidates) {
                    if (candidate_count < candidates.size()) {
                        candidates[candidate_count++] = candidate;
                    }
                }
                if (circle_candidate.has_value() && candidate_count < candidates.size()) {
                    candidates[candidate_count++] = *circle_candidate;
                }
            }
            if (ml_result.candidate.present && candidate_count < candidates.size()) {
                candidates[candidate_count++] = ml_result.candidate;
            }
            for (std::size_t index = 0; index < candidate_count; ++index) {
                const port::VisualReferenceCandidate& candidate = candidates[index];
                if (candidate.present) {
                    port::AppendVisualReferenceCandidatePath(candidate_paths, candidate);
                }
            }
            visual_selection = reference::SelectVisualReference(candidates.data(),
                                                                candidate_count);
        }
        current_usability =
            reference::EvaluateReferenceUsability(visual_selection.reference_path, params);
        if (current_usability.usable) {
            continuity.reference_path = visual_selection.reference_path;
            continuity.mode = visual_selection.reference_path.mode;
            continuity.source = visual_selection.source;
            continuity.hold_selected = false;
            continuity.reference_capture_time_ms = capture.capture_time_ms;
            continuity.next_hold_state =
                reference::MakeReferenceHoldState(visual_selection.reference_path,
                                                  capture.capture_time_ms,
                                                  params);
            selected_usability = current_usability;
        } else {
            port::ReferenceContinuityResult hold_candidate{};
            port::ReferenceUsability hold_usability{};
            {
                LS2K_PERF_SCOPE(port::PerfStage::kReferenceHold);
                hold_candidate =
                    reference::BuildReferenceHoldCandidate(prior_memory.reference_hold, params);
                hold_usability =
                    reference::EvaluateReferenceUsability(hold_candidate.reference_path, params);
            }
            if (hold_usability.usable) {
                continuity = hold_candidate;
                selected_usability = hold_usability;
            } else {
                continuity = {};
                selected_usability =
                    reference::EvaluateReferenceUsability(continuity.reference_path, params);
            }
        }
        lateral_error = reference::ComputeReferenceLateralError(continuity.reference_path,
                                                                selected_usability,
                                                                params);
        reference_tracking_geometry =
            reference::ComputeReferenceTrackingGeometry(continuity.reference_path,
                                                        selected_usability,
                                                        params.bev_control_model);
        reference_control = reference::EvaluateReferenceControlReadiness(selected_usability,
                                                                         reference_tracking_geometry,
                                                                         continuity.hold_selected);
        perception_memory_.reference_hold = continuity.next_hold_state;
    }

    return BuildPerceptionResult(capture,
                                 current_facts,
                                 health,
                                 element_evidence,
                                 circle_v2_snapshot,
                                 ml_snapshot,
                                 candidate_paths,
                                 visual_selection,
                                 continuity,
                                 selected_usability,
                                 lateral_error,
                                 reference_tracking_geometry,
                                 reference_control,
                                 port::NowMs());
}

}  // namespace ls2k::runtime
