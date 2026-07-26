#!/usr/bin/env python3
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
CIRCLE_DIR = ROOT / "new/code/vision/elements/circle_v2"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(f"circle_v2_metric_contract_test failed: {message}")


active_source = "\n".join(
    path.read_text(encoding="utf-8")
    for path in sorted(CIRCLE_DIR.rglob("*"))
    if path.suffix in {".cpp", ".hpp"}
)
for forbidden in (".spans", "entry_bottom", "kOpeningSustainRows"):
    require(forbidden not in active_source, f"active CircleV2 contains {forbidden}")

params = json.loads((ROOT / "new/config/default_params.json").read_text(encoding="utf-8"))
circle = params["BEV_ELEMENT"]
expected = {
    "CIRCLE_V2_MIN_SAMPLEABLE_WIDTH_M": 0.35,
    "CIRCLE_V2_OPENING_FORWARD_MIN_M": 0.05,
    "CIRCLE_V2_OPENING_FORWARD_MAX_M": 1.50,
    "CIRCLE_V2_OPENING_DISTANCE_MIN_M": 0.055,
    "CIRCLE_V2_OPENING_CONFIRM_FORWARD_SPAN_M": 0.10,
    "CIRCLE_V2_ENTRY_FORWARD_MIN_M": 0.10,
    "CIRCLE_V2_ENTRY_FORWARD_MAX_M": 0.50,
    "CIRCLE_V2_INNER_GEOMETRY_FORWARD_MIN_M": 0.05,
    "CIRCLE_V2_INNER_GEOMETRY_FORWARD_MAX_M": 0.50,
    "CIRCLE_V2_EXIT_GEOMETRY_FORWARD_MIN_M": 0.05,
    "CIRCLE_V2_EXIT_GEOMETRY_FORWARD_MAX_M": 0.50,
    "CIRCLE_V2_EXIT_STRAIGHT_MAX_LATERAL_SPAN_M": 0.13,
}
for key, value in expected.items():
    require(key in circle, f"missing runtime key {key}")
    require(abs(float(circle[key]) - value) < 1.0e-9, f"wrong default for {key}")

for removed in (
    "CIRCLE_V2_ENTRY_BOTTOM_MIN_ROW_COUNT",
    "CIRCLE_V2_ENTRY_BOTTOM_FORWARD_MIN_M",
    "CIRCLE_V2_ENTRY_BOTTOM_FORWARD_MAX_M",
):
    require(removed not in circle, f"removed runtime key remains: {removed}")

require(
    not (ROOT / "new/verification/tests/circle_v2_capture_diagnostic.cpp").exists(),
    "one-shot white-box diagnostic still exists",
)
require(
    (ROOT / "new/verification/tests/circle_v2_aligned_replay.cpp").exists(),
    "repeatable aligned replay is missing",
)
print("circle_v2_metric_contract_test passed")
