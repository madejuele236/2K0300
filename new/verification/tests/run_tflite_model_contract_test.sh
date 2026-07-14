#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/../../.." && pwd)"
tflm_root="$repo_root/true_LS2K0300_Library/Seekfree_LS2K0300_Opensource_Library/libraries/zf_components/tflm"
binary="${TMPDIR:-/tmp}/tflite_model_contract_probe"
model="$repo_root/model_training/experiments/v8_parent_c2612_d4_multiteacher_20260520_0001/s2d_c2_6_12_d4_shadow_init/parent_int8.tflite"

g++ -std=c++17 -O2 -Wall -Wextra -Werror \
  -I"$tflm_root" \
  -I"$tflm_root/third_party/flatbuffers/include" \
  "$repo_root/new/verification/tests/tflite_model_contract_probe.cpp" \
  -o "$binary"

output="$($binary "$model")"
grep -Fq "input name=serving_default_gray32:0 type=INT8 shape=1x32x32x1 scale=0.00392157 zero_point=-128" <<<"$output"
grep -Fq "output name=StatefulPartitionedCall_1:0 type=INT8 shape=1x4 scale=0.148821 zero_point=19" <<<"$output"
for op in SPACE_TO_DEPTH CONV_2D MAX_POOL_2D MEAN FULLY_CONNECTED; do
  grep -Fq "builtin=$op" <<<"$output"
done
printf '%s\n' "tflite_model_contract_test passed"
