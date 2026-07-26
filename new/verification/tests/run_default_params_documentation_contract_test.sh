#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATIC_CHECK="${SCRIPT_DIR}/default_params_doc_current_values_check.mjs"
SEMANTIC_RUNNER="${SCRIPT_DIR}/run_param_store_semantic_boundary_probe.sh"

for dependency in node c++ pkg-config; do
  if ! command -v "${dependency}" >/dev/null 2>&1; then
    echo "[ERROR] ${dependency} is required for the default_params documentation contract test" >&2
    exit 2
  fi
done

for required_file in "${STATIC_CHECK}" "${SEMANTIC_RUNNER}" \
  "${SCRIPT_DIR}/param_store_semantic_boundary_probe.cpp"; do
  if [[ ! -r "${required_file}" ]]; then
    echo "[ERROR] required documentation contract test input is missing: ${required_file}" >&2
    exit 2
  fi
done

node --check "${STATIC_CHECK}"
node "${STATIC_CHECK}"
if ! semantic_output="$(bash "${SEMANTIC_RUNNER}" 2>&1)"; then
  printf '%s\n' "${semantic_output}" >&2
  echo "[ERROR] production MakeParamStore semantic boundary runner failed" >&2
  exit 1
fi
printf '%s\n' "${semantic_output}"
if [[ "${semantic_output}" != *"param_store_semantic_boundary_probe passed"* ]]; then
  echo "[ERROR] production MakeParamStore semantic boundary runner omitted its success marker" >&2
  exit 1
fi

echo "default_params documentation contract test passed (static documentation + production MakeParamStore semantics)"
