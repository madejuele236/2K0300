#!/usr/bin/env python3
"""Render the planned 32x32 ROI from the captured YUYV with production semantics."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parents[1] / "user"))
from ml_four_corner_calibration import homography, project, projector_points  # noqa: E402


config = json.loads((ROOT.parents[1] / "config" / "default_params.json").read_text())
source, target = projector_points({"bev_projector": config["BEV_PROJECTOR"]}, ROOT)
vehicle_as_source = [
    {"row_px": point["forward_m"], "col_px": point["lateral_m"]} for point in target
]
image_as_target = [
    {"forward_m": point["row_px"], "lateral_m": point["col_px"]} for point in source
]
vehicle_to_image = homography(vehicle_as_source, image_as_target)

calibration = json.loads((ROOT / "calibration-report.json").read_text())
corners = calibration["frames"][0]["bev_corners"]
points = [(point["forward_m"], point["lateral_m"]) for point in corners]
center_f = sum(point[0] for point in points) / 4.0
center_l = sum(point[1] for point in points) / 4.0
vectors = [
    (points[(index + 1) % 4][0] - points[index][0],
     points[(index + 1) % 4][1] - points[index][1])
    for index in range(4)
]
lengths = [math.hypot(*vector) for vector in vectors]
long_index = max(range(4), key=lengths.__getitem__)
lf, ll = vectors[long_index]
axis_norm = math.hypot(lf, ll)
lf, ll = lf / axis_norm, ll / axis_norm
if ll < 0.0 or (ll == 0.0 and lf < 0.0):
    lf, ll = -lf, -ll
nf, nl = -ll, lf
if nf < 0.0:
    nf, nl = -nf, -nl
long_edge = calibration["frames"][0]["long_edge_m"]
short_edge = calibration["frames"][0]["short_edge_m"]
edge_center_f = center_f + 0.5 * short_edge * nf
edge_center_l = center_l + 0.5 * short_edge * nl

raw = np.frombuffer((ROOT / "frame-raw.yuyv").read_bytes(), dtype=np.uint8).reshape(240, 160, 4)


def yuv_at(row: int, col: int) -> tuple[int, int, int]:
    pair = raw[row, col // 2]
    return int(pair[0 if col % 2 == 0 else 2]), int(pair[1]), int(pair[3])


def bilinear(row: float, col: float) -> tuple[int, int, int]:
    row0, col0 = int(row), int(col)
    row1, col1 = min(row0 + 1, 239), min(col0 + 1, 319)
    rf, cf = row - row0, col - col0
    samples = [yuv_at(row0, col0), yuv_at(row0, col1), yuv_at(row1, col0), yuv_at(row1, col1)]
    result = []
    for channel in range(3):
        top = samples[0][channel] * (1.0 - cf) + samples[1][channel] * cf
        bottom = samples[2][channel] * (1.0 - cf) + samples[3][channel] * cf
        result.append(max(0, min(255, int(top * (1.0 - rf) + bottom * rf + 0.5))))
    return tuple(result)


roi = np.empty((32, 32), dtype=np.uint8)
sample_points = []
for row in range(32):
    along_forward = ((row + 0.5) / 32.0) * long_edge
    for col in range(32):
        along_long = ((col + 0.5) / 32.0 - 0.5) * long_edge
        forward = edge_center_f + along_long * lf + along_forward * nf
        lateral = edge_center_l + along_long * ll + along_forward * nl
        image_row, image_col = project(vehicle_to_image, forward, lateral)
        y, u_byte, v_byte = bilinear(image_row, image_col)
        u, v = u_byte - 128.0, v_byte - 128.0
        red = min(255.0, max(0.0, y + 1.402 * v))
        green = min(255.0, max(0.0, y - 0.344136 * u - 0.714136 * v))
        blue = min(255.0, max(0.0, y + 1.772 * u))
        roi[row, col] = max(0, min(255, int((red + green + blue) / 3.0 + 0.5)))
        sample_points.append((image_row, image_col))

(ROOT / "planned-roi-gray8.raw").write_bytes(roi.tobytes())
Image.fromarray(roi, "L").resize((320, 320), Image.Resampling.NEAREST).save(ROOT / "planned-roi-gray8-10x.png")
(ROOT / "planned-roi-metadata.json").write_text(json.dumps({
    "long_edge_m": long_edge,
    "short_edge_m": short_edge,
    "center_forward_m": center_f,
    "center_lateral_m": center_l,
    "long_axis_forward": lf,
    "long_axis_lateral": ll,
    "forward_normal_forward": nf,
    "forward_normal_lateral": nl,
    "image_row_min": min(point[0] for point in sample_points),
    "image_row_max": max(point[0] for point in sample_points),
    "image_col_min": min(point[1] for point in sample_points),
    "image_col_max": max(point[1] for point in sample_points),
}, indent=2) + "\n")
print((ROOT / "planned-roi-metadata.json").read_text(), end="")
