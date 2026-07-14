#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "$0")/../../.." && pwd)"
out="${TMPDIR:-/tmp}/ml_speed_policy_test"
g++ -std=c++17 -Wall -Wextra -Werror -I"$repo_root/new/code" \
  "$repo_root/new/verification/tests/ml_speed_policy_test.cpp" \
  "$repo_root/new/code/control/tuning_state.cpp" \
  -o "$out"
"$out"
