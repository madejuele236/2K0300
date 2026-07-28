#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
CAPTURE_DIR="${BEV_CONNECTIVITY_CAPTURE_DIR:-${REPO_ROOT}/new/verification/host-capture-20260726T090850Z/steering-media}"
FRAME_ID="${BEV_CONNECTIVITY_FRAME_ID:-607}"
METADATA="${CAPTURE_DIR}/frame_metadata.jsonl"
RAW_FRAME="${CAPTURE_DIR}/frames/frame-$(printf '%06d' "${FRAME_ID}").raw"
PARAMS="${REPO_ROOT}/new/config/default_params.json"
EVIDENCE_DIR="${REPO_ROOT}/new/verification/bev-connectivity-replay-20260726"
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
  "${REPO_ROOT}/new/code/vision/image/luma_sampler.cpp" \
  "${REPO_ROOT}/new/code/vision/image/illumination_binary_model.cpp" \
  -o "${OUT_BIN}" \
  $(pkg-config --libs opencv4)

if [[ "${BUILD_ONLY:-0}" == "1" ]]; then
  exit 0
fi

METADATA_SHA256="$(sha256sum "${METADATA}" | awk '{print $1}')"
RAW_SHA256="$(sha256sum "${RAW_FRAME}" | awk '{print $1}')"
PARAMS_SHA256="$(sha256sum "${PARAMS}" | awk '{print $1}')"
"${OUT_BIN}" "${METADATA}" "${RAW_FRAME}" "${PARAMS}" \
  "${EVIDENCE_DIR}/report-frame-${FRAME_ID}.json" "${METADATA_SHA256}" "${RAW_SHA256}" \
  "${PARAMS_SHA256}" 320 240 "${FRAME_ID}"
