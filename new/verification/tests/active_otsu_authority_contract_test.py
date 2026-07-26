#!/usr/bin/env python3
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
CODE = ROOT / "new" / "code"
VIEWER = ROOT / "new" / "user" / "steering_media_live_server.py"


def read(relative: str) -> str:
    return (CODE / relative).read_text(encoding="utf-8")


pipeline = read("runtime/pipelines/steering_frame_pipeline.cpp")
boundary = read("vision/bev/bev_boundary_row.cpp")
connectivity = read("vision/bev/bev_image_segment_connectivity.cpp")
viewer = VIEWER.read_text(encoding="utf-8")
active_sources = "\n".join(
    path.read_text(encoding="utf-8", errors="replace")
    for path in CODE.rglob("*")
    if path.suffix in {".cpp", ".hpp"} and "archive" not in path.parts
)

assert pipeline.count("ComputeSparseOtsuThreshold(") == 1, (
    "active frame pipeline must compute Otsu exactly once per call site"
)
assert "RunBEVSimplePerception(capture.pixel_view," in pipeline
assert "otsu_state," in pipeline
assert "IsOtsuWhite(" in boundary, "boundary owner must use the shared predicate"
assert "IsOtsuWhite(" in connectivity, "connectivity owner must use the shared predicate"
assert "LOCAL_JUMP_MIN_Y" not in active_sources
assert "local_jump_min_y" not in active_sources
assert 'header.pixel_format === "gray8"' in viewer
assert "otsu?.valid === true" in viewer
assert "decoded.pixels[index] > threshold ? 255 : 0" in viewer
assert "binary unavailable: requires gray8" in viewer

print("active Otsu authority contract test passed")
