#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
OUT_BIN="/tmp/zebra_aligned_replay"
MEDIA_DIR="${ZEBRA_MEDIA_DIR:-/mnt/c/Users/27866/Documents/2K0300-long-captures/long-capture-20260728-1831/steering-media}"
PARAMS="${ZEBRA_PARAMS:-${REPO_ROOT}/new/config/default_params.json}"
REPORT="${ZEBRA_REPORT:-/tmp/zebra-aligned-replay.tsv}"

g++ -std=c++17 -O2 -Wall -Wextra -Werror \
  -I"${REPO_ROOT}/new/code" \
  -I"${REPO_ROOT}/new/code/port" \
  -I"${REPO_ROOT}/new/code/platform" \
  -I/usr/include/opencv4 \
  "${SCRIPT_DIR}/zebra_aligned_replay.cpp" \
  "${REPO_ROOT}/new/code/platform/param_store.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_projector.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_image_segment_connectivity.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/single_boundary_offset.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_boundary_row.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_sample_projection_lut.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_sparse_row_scanner.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_reference_path_builder.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_simple_perception.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/zebra_element_evidence.cpp" \
  "${REPO_ROOT}/new/code/vision/image/luma_sampler.cpp" \
  "${REPO_ROOT}/new/code/vision/image/illumination_binary_model.cpp" \
  -lopencv_core \
  -o "${OUT_BIN}"

"${OUT_BIN}" "${MEDIA_DIR}" "${PARAMS}" "${REPORT}"
