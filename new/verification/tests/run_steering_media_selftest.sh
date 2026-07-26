#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_BIN="${SCRIPT_DIR}/steering_media_selftest"
STRICT_JSON_CHECK="${SCRIPT_DIR}/strict_json_protocol_output_check.mjs"

source "${SCRIPT_DIR}/common_steering_test_build.sh"

if ! command -v node >/dev/null 2>&1; then
  echo "steering_media_selftest requires Node.js for browser-compatible JSON.parse validation" >&2
  exit 1
fi

ARTIFACT_DIR="$(mktemp -d "${TMPDIR:-/tmp}/ls2k-steering-media-json.XXXXXX")"
trap 'rm -rf "${ARTIFACT_DIR}"' EXIT

compile_test_binary \
  "${OUT_BIN}" \
  "${REPO_ROOT}/new/user/steering_media_selftest.cpp" \
  "${STEERING_MEDIA_SOURCES[@]}"

LS2K_STRICT_JSON_ARTIFACT_DIR="${ARTIFACT_DIR}" "${OUT_BIN}"
node "${STRICT_JSON_CHECK}" steering-media \
  "${ARTIFACT_DIR}/steering_media_config_snapshot.json" \
  "${ARTIFACT_DIR}/steering_media_image_header.json"
