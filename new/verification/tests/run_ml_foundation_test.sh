#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "$0")/../../.." && pwd)"
binary="${TMPDIR:-/tmp}/ml_foundation_test"
g++ -std=c++17 -Wall -Wextra -Werror -I"$repo_root/new/code" \
  "$repo_root/new/verification/tests/ml_foundation_test.cpp" \
  "$repo_root/new/code/vision/image/color_sampler.cpp" \
  "$repo_root/new/code/vision/ml/v9_descriptor.cpp" \
  "$repo_root/new/code/vision/ml/v9_replay.cpp" \
  "$repo_root/new/code/vision/ml/ml_class_mapping.cpp" \
  "$repo_root/new/code/vision/ml/selected_ml_classifier.cpp" \
  -o "$binary"
"$binary"
g++ -std=c++17 -Wall -Wextra -Werror -I"$repo_root/new/code" -c \
  "$repo_root/new/code/vision/ml/red_rectangle_detector.cpp" -o "${TMPDIR:-/tmp}/red_rectangle_detector.o"
g++ -std=c++17 -Wall -Wextra -Werror -I"$repo_root/new/code" -c \
  "$repo_root/new/code/vision/ml/roi_sampler.cpp" -o "${TMPDIR:-/tmp}/roi_sampler.o"
