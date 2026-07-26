#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
OUT_BIN="${SCRIPT_DIR}/assistant_telemetry_selftest"
STRICT_JSON_CHECK="${SCRIPT_DIR}/strict_json_protocol_output_check.mjs"
OPENCV_ROOT="${OPENCV_ROOT:-/opt/ls_2k0300_env/opencv_4_10_build}"

if ! command -v node >/dev/null 2>&1; then
  echo "assistant_telemetry_selftest requires Node.js for browser-compatible JSON.parse validation" >&2
  exit 1
fi

ARTIFACT_DIR="$(mktemp -d "${TMPDIR:-/tmp}/ls2k-assistant-json.XXXXXX")"
trap 'rm -rf "${ARTIFACT_DIR}"' EXIT

c++ -std=c++17 -Wall -Wextra -Werror -ffunction-sections -fdata-sections -pthread \
  -I"${REPO_ROOT}/new/code" \
  -I"${REPO_ROOT}/new/code/port" \
  -I"${REPO_ROOT}/new/code/platform" \
  -I"${REPO_ROOT}/new/code/runtime" \
  -I"${OPENCV_ROOT}/include/opencv4" \
  "${REPO_ROOT}/new/verification/tests/assistant_telemetry_selftest.cpp" \
  "${REPO_ROOT}/new/code/transport/assistant_protocol.cpp" \
  "${REPO_ROOT}/new/code/safety/control_gate.cpp" \
  "${REPO_ROOT}/new/code/safety/control_apply_observation.cpp" \
  "${REPO_ROOT}/new/code/control/motion_supervisor.cpp" \
  "${REPO_ROOT}/new/code/control/wheel_pid.cpp" \
  -Wl,--gc-sections \
  -o "${OUT_BIN}"

LS2K_STRICT_JSON_ARTIFACT_DIR="${ARTIFACT_DIR}" "${OUT_BIN}"
node "${STRICT_JSON_CHECK}" assistant \
  "${ARTIFACT_DIR}/assistant_telemetry.json"
