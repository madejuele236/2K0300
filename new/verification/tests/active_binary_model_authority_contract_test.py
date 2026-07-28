#!/usr/bin/env python3
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
CODE = ROOT / "new" / "code"
VIEWER = ROOT / "new" / "user" / "steering_media_live_server.py"


def read(relative: str) -> str:
    return (CODE / relative).read_text(encoding="utf-8")


pipeline = read("runtime/pipelines/steering_frame_pipeline.cpp")
scanner = read("vision/bev/bev_sparse_row_scanner.cpp")
boundary = read("vision/bev/bev_boundary_row.cpp")
connectivity = read("vision/bev/bev_image_segment_connectivity.cpp")
viewer = VIEWER.read_text(encoding="utf-8")
active_sources = "\n".join(
    path.read_text(encoding="utf-8", errors="replace")
    for path in CODE.rglob("*")
    if path.suffix in {".cpp", ".hpp"} and "archive" not in path.parts
)

assert pipeline.count("ComputeIlluminationBinaryModel(") == 1, (
    "the active frame pipeline must compute one binary model"
)
assert "RunBEVSimplePerception(capture.pixel_view," in pipeline
assert "binary_model," in pipeline
assert "ClassifyImagePoint(" in scanner, (
    "the sparse scanner must classify through the shared binary predicate"
)
assert "ClassifyImagePixel(" in connectivity, (
    "connectivity must classify through the shared binary predicate"
)
assert "ClassifyImage" not in boundary
assert ".white" in boundary, (
    "boundary extraction must consume scanner classification facts"
)
assert "LOCAL_JUMP_MIN_Y" not in active_sources
assert "local_jump_min_y" not in active_sources
assert "OtsuThreshold" not in active_sources
assert "ComputeSparseOtsu" not in active_sources
assert 'header.pixel_format === "gray8"' in viewer
assert "binary_model" in viewer
assert "lumaScale * decoded.pixels[index]" in viewer
assert "illuminationWeight * localIllumination" in viewer
assert "residual > threshold ? 255 : 0" in viewer
assert "requires aligned full-resolution gray8" in viewer

print("active binary model authority contract test passed")
