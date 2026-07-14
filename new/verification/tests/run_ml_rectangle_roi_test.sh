#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "$0")/../../.." && pwd)"
binary="${TMPDIR:-/tmp}/ml_rectangle_roi_test"
g++ -std=c++17 -Wall -Wextra -Werror -I"$repo_root/new/code" \
  "$repo_root/new/verification/tests/ml_rectangle_roi_test.cpp" \
  "$repo_root/new/code/vision/bev/bev_projector.cpp" \
  "$repo_root/new/code/vision/image/color_sampler.cpp" \
  "$repo_root/new/code/vision/ml/red_rectangle_detector.cpp" \
  "$repo_root/new/code/vision/ml/roi_sampler.cpp" \
  -o "$binary"
"$binary"
