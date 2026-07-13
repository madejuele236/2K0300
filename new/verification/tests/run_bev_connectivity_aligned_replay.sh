#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SOURCE="${REPO_ROOT}/new/verification/live-check-20260712/latest.bin"
PARAMS="${REPO_ROOT}/new/config/default_params.json"
EVIDENCE_DIR="${REPO_ROOT}/new/verification/bev-connectivity-replay-20260713"
OUT_BIN="${SCRIPT_DIR}/bev_connectivity_aligned_replay"

mkdir -p "${EVIDENCE_DIR}"

# shellcheck disable=SC2046
c++ -std=c++17 -Wall -Wextra -Werror -pthread \
  -I"${REPO_ROOT}/new/code" \
  $(pkg-config --cflags opencv4) \
  "${SCRIPT_DIR}/bev_connectivity_aligned_replay.cpp" \
  "${REPO_ROOT}/new/code/platform/param_store.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_projector.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_image_segment_connectivity.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_reference_path_builder.cpp" \
  "${REPO_ROOT}/new/code/vision/image/luma_sampler.cpp" \
  -o "${OUT_BIN}" \
  $(pkg-config --libs opencv4)

SOURCE_SHA256="$(sha256sum "${SOURCE}" | awk '{print $1}')"
PARAMS_SHA256="$(sha256sum "${PARAMS}" | awk '{print $1}')"
"${OUT_BIN}" "${SOURCE}" "${PARAMS}" "${EVIDENCE_DIR}/report.json" \
  "${SOURCE_SHA256}" "${PARAMS_SHA256}"
