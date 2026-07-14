#!/usr/bin/env python3
"""Derive repeatable red-marker pixel bounds for the stationary board capture series."""

from __future__ import annotations

import json
from collections import deque
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent
HEIGHT = 240
WIDTH = 320
STRIDE = 640


def marker_component(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    raw = np.frombuffer(path.read_bytes(), dtype=np.uint8).reshape(HEIGHT, WIDTH // 2, 4)
    y = np.empty((HEIGHT, WIDTH), dtype=np.uint8)
    y[:, 0::2] = raw[:, :, 0]
    y[:, 1::2] = raw[:, :, 2]
    u = np.repeat(raw[:, :, 1], 2, axis=1)
    v = np.repeat(raw[:, :, 3], 2, axis=1)
    mask = (y >= 45) & (y <= 115) & (u >= 100) & (u <= 145) & (v >= 140) & (v <= 200)
    # The physical calibration setup fixes the marker in this image window;
    # the crop excludes other red scene objects without changing marker pixels.
    window = np.zeros_like(mask)
    window[140:185, 130:215] = True
    mask &= window
    visited = np.zeros_like(mask)
    best: list[tuple[int, int]] = []
    for start_row, start_col in zip(*np.nonzero(mask)):
        if visited[start_row, start_col]:
            continue
        queue = deque([(int(start_row), int(start_col))])
        visited[start_row, start_col] = True
        points: list[tuple[int, int]] = []
        while queue:
            row, col = queue.popleft()
            points.append((row, col))
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = row + dr, col + dc
                if 0 <= nr < HEIGHT and 0 <= nc < WIDTH and mask[nr, nc] and not visited[nr, nc]:
                    visited[nr, nc] = True
                    queue.append((nr, nc))
        if len(points) > len(best):
            best = points
    if not best:
        raise RuntimeError(f"no marker component in {path}")
    component = np.zeros_like(mask)
    rows, cols = zip(*best)
    component[np.asarray(rows), np.asarray(cols)] = True
    return component, y, u, v


records = []
measurements = []
for path in sorted(ROOT.glob("ml_calibration_[0-7].yuyv")):
    component, y, u, v = marker_component(path)
    rows, cols = np.nonzero(component)
    row_min, row_max = int(rows.min()), int(rows.max())
    col_min, col_max = int(cols.min()), int(cols.max())
    # Pixel-edge coordinates make the annotated quadrilateral describe the
    # occupied marker area rather than only the sample-centre span.
    corners = [
        {"row_px": row_min - 0.5, "col_px": col_min - 0.5},
        {"row_px": row_min - 0.5, "col_px": col_max + 0.5},
        {"row_px": row_max + 0.5, "col_px": col_max + 0.5},
        {"row_px": row_max + 0.5, "col_px": col_min - 0.5},
    ]
    records.append({
        "frame_path": path.name,
        "format": "yuyv",
        "width": WIDTH,
        "height": HEIGHT,
        "stride": STRIDE,
        "bev_projector_config_path": "../../config/default_params.json",
        "corners": corners,
    })
    measurements.append({
        "frame": path.name,
        "component_pixels": int(component.sum()),
        "row_min": row_min,
        "row_max": row_max,
        "col_min": col_min,
        "col_max": col_max,
        "y_min": int(y[component].min()),
        "y_max": int(y[component].max()),
        "u_min": int(u[component].min()),
        "u_max": int(u[component].max()),
        "v_min": int(v[component].min()),
        "v_max": int(v[component].max()),
    })

if len(records) != 8:
    raise RuntimeError(f"expected 8 captures, found {len(records)}")
(ROOT / "annotations-series.jsonl").write_text(
    "".join(json.dumps(record, separators=(",", ":")) + "\n" for record in records),
    encoding="utf-8",
)
(ROOT / "capture-series-measurements.json").write_text(
    json.dumps(measurements, indent=2) + "\n", encoding="utf-8"
)
print(json.dumps(measurements, indent=2))
