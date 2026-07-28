#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
GENERATED_DIR="${REPO_ROOT}/new/out/generated/v9"
OUT_BIN="${SCRIPT_DIR}/current_pipeline_path_replay"

if [[ ! -f "${GENERATED_DIR}/generated_v9_artifact.cpp" ]]; then
  echo "missing production-generated V9 artifact; build new/out/new first" >&2
  exit 1
fi

# This is a native-host build of the production perception owners. Only the
# final PNG drawing and replay input adapter live in the verification target.
# shellcheck disable=SC2046
g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread \
  -DLS2K_ML_CLASSIFIER_BACKEND=1 \
  -DLS2K_PERF_ENABLED=1 \
  -I"${REPO_ROOT}/new/code" \
  -I"${GENERATED_DIR}" \
  $(pkg-config --cflags opencv4) \
  "${SCRIPT_DIR}/current_pipeline_path_replay.cpp" \
  "${REPO_ROOT}/new/code/platform/param_store.cpp" \
  "${REPO_ROOT}/new/code/port/perf_counter.cpp" \
  "${REPO_ROOT}/new/code/vision/image/luma_sampler.cpp" \
  "${REPO_ROOT}/new/code/vision/image/illumination_binary_model.cpp" \
  "${REPO_ROOT}/new/code/vision/image/color_sampler.cpp" \
  "${REPO_ROOT}/new/code/vision/ml/red_rectangle_detector.cpp" \
  "${REPO_ROOT}/new/code/vision/ml/roi_sampler.cpp" \
  "${REPO_ROOT}/new/code/vision/ml/ml_class_mapping.cpp" \
  "${REPO_ROOT}/new/code/vision/ml/selected_ml_classifier.cpp" \
  "${REPO_ROOT}/new/code/vision/ml/selected_ml_classifier_v9.cpp" \
  "${REPO_ROOT}/new/code/vision/ml/v9_descriptor.cpp" \
  "${REPO_ROOT}/new/code/vision/ml/v9_replay.cpp" \
  "${GENERATED_DIR}/generated_v9_artifact.cpp" \
  "${REPO_ROOT}/new/code/vision/ml/ml_observer.cpp" \
  "${REPO_ROOT}/new/code/vision/ml/ml_path_generator.cpp" \
  "${REPO_ROOT}/new/code/vision/ml/ml_scene.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_projector.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_image_segment_connectivity.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/single_boundary_offset.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_boundary_row.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_sample_projection_lut.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_sparse_row_scanner.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_reference_path_builder.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_simple_perception.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/cross_exit_element_evidence.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/cross_straight_path_planner.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/zebra_element_evidence.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/zebra_stop_scene.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/visual_element_evidence.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/visual_element_pipeline.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/circle_v2_scene.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/circle_v2_reference_adapter.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/detail/circle_v2_event_observer.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/detail/circle_v2_entry_cue_observer.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/detail/circle_v2_reducer.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/detail/circle_v2_geometry_observer.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/detail/circle_v2_composer.cpp" \
  "${REPO_ROOT}/new/code/reference/visual_reference_orchestration.cpp" \
  "${REPO_ROOT}/new/code/reference/reference_usability.cpp" \
  "${REPO_ROOT}/new/code/reference/reference_continuity.cpp" \
  "${REPO_ROOT}/new/code/reference/reference_lateral_error.cpp" \
  "${REPO_ROOT}/new/code/reference/reference_tracking_geometry.cpp" \
  "${REPO_ROOT}/new/code/reference/reference_control_readiness.cpp" \
  "${REPO_ROOT}/new/code/runtime/pipelines/steering_frame_pipeline.cpp" \
  -o "${OUT_BIN}" \
  $(pkg-config --libs opencv4)

"${OUT_BIN}" "$@"
