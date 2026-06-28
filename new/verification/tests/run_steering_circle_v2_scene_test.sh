#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/common_steering_test_build.sh"

OUT_BIN="/tmp/steering_circle_v2_scene_test"
compile_test_binary \
  "${OUT_BIN}" \
  "${SCRIPT_DIR}/steering_circle_v2_scene_test.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/circle_v2_scene.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/circle_v2_reference_adapter.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/detail/circle_v2_event_observer.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/detail/circle_v2_expansion_observer.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/detail/circle_v2_reducer.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/detail/circle_v2_geometry_observer.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/circle_v2/detail/circle_v2_composer.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/single_boundary_offset.cpp"

"${OUT_BIN}"
