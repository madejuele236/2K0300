#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
CAPTURE_DIR="${CIRCLE_V2_CAPTURE_DIR:-${REPO_ROOT}/new/verification/host-capture-20260726T113215Z/steering-media}"
PARAMS="${REPO_ROOT}/new/config/default_params.json"
EVIDENCE_DIR="${REPO_ROOT}/new/verification/circle-v2-aligned-replay-20260726"
OUT_BIN="${SCRIPT_DIR}/circle_v2_aligned_replay"

# shellcheck disable=SC2046
c++ -std=c++17 -Wall -Wextra -Werror -pthread \
  -I"${REPO_ROOT}/new/code" \
  $(pkg-config --cflags opencv4) \
  "${SCRIPT_DIR}/circle_v2_aligned_replay.cpp" \
  "${REPO_ROOT}/new/code/platform/param_store.cpp" \
  "${REPO_ROOT}/new/code/port/perf_counter.cpp" \
  "${REPO_ROOT}/new/code/vision/image/luma_sampler.cpp" \
  "${REPO_ROOT}/new/code/vision/image/illumination_binary_model.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_projector.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_sample_projection_lut.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_boundary_row.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_sparse_row_scanner.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_image_segment_connectivity.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_reference_path_builder.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_simple_perception.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/single_boundary_offset.cpp" \
  "${REPO_ROOT}/new/code/reference/reference_usability.cpp" \
  "${REPO_ROOT}/new/code/reference/reference_tracking_geometry.cpp" \
  "${REPO_ROOT}/new/code/reference/reference_continuity.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/circle_v2_scene.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/detail/circle_v2_event_observer.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/detail/circle_v2_entry_cue_observer.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/detail/circle_v2_reducer.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/detail/circle_v2_geometry_observer.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/detail/circle_v2_composer.cpp" \
  -o "${OUT_BIN}" \
  $(pkg-config --libs opencv4)

if [[ "${BUILD_ONLY:-0}" == "1" ]]; then
  exit 0
fi

"${OUT_BIN}" "${CAPTURE_DIR}/frames" "${PARAMS}" "${EVIDENCE_DIR}"
