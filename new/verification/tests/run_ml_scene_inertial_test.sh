#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/../../.." && pwd)"
out="${TMPDIR:-/tmp}/ml_scene_inertial_test"

g++ -std=c++17 -Wall -Wextra -Werror -I"$repo_root/new/code" \
  "$repo_root/new/verification/tests/ml_scene_inertial_test.cpp" \
  "$repo_root/new/code/vision/ml/ml_reference_adapter.cpp" \
  "$repo_root/new/code/vision/ml/ml_inertial_path_tracker.cpp" \
  "$repo_root/new/code/vision/ml/ml_scene.cpp" \
  "$repo_root/new/code/vision/ml/red_rectangle_detector.cpp" \
  "$repo_root/new/code/vision/ml/roi_sampler.cpp" \
  "$repo_root/new/code/vision/ml/selected_ml_classifier.cpp" \
  "$repo_root/new/code/vision/ml/ml_class_mapping.cpp" \
  "$repo_root/new/code/vision/ml/ml_path_generator.cpp" \
  "$repo_root/new/code/vision/bev/bev_projector.cpp" \
  "$repo_root/new/code/vision/bev/bev_reference_path_builder.cpp" \
  "$repo_root/new/code/vision/image/color_sampler.cpp" \
  -o "$out"
"$out"
