#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
OUT_BIN="${SCRIPT_DIR}/cross_straight_aligned_replay"

g++ -std=c++17 -O2 -Wall -Wextra -Werror \
  -I"${REPO_ROOT}/new/code" \
  -I"${REPO_ROOT}/new/code/port" \
  -I"${REPO_ROOT}/new/code/platform" \
  -I/usr/include/opencv4 \
  "${REPO_ROOT}/new/verification/tests/cross_straight_aligned_replay.cpp" \
  "${REPO_ROOT}/new/code/platform/param_store.cpp" \
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
  "${REPO_ROOT}/new/code/vision/image/luma_sampler.cpp" \
  "${REPO_ROOT}/new/code/vision/image/otsu_threshold.cpp" \
  -lopencv_core \
  -o "${OUT_BIN}"

"${OUT_BIN}" "$@"
