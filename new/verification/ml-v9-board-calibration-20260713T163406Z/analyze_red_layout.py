#!/usr/bin/env python3
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parent
RAW = np.frombuffer((ROOT / "frame-raw.yuyv").read_bytes(), dtype=np.uint8).reshape(240, 160, 4)
LAYOUTS = {
    "yuyv": (0, 1, 2, 3),
    "yvyu": (0, 3, 2, 1),
}


def rgb_for(layout: tuple[int, int, int, int]) -> np.ndarray:
    y0, u_index, y1, v_index = layout
    # Use int32 for the BT.601 products below.  int16 overflows on terms such
    # as 409 * V and makes valid YUYV bytes look like the wrong colors.
    y = np.empty((240, 320), dtype=np.int32)
    y[:, 0::2] = RAW[:, :, y0]
    y[:, 1::2] = RAW[:, :, y1]
    u = np.repeat(RAW[:, :, u_index].astype(np.int32), 2, axis=1) - 128
    v = np.repeat(RAW[:, :, v_index].astype(np.int32), 2, axis=1) - 128
    c = np.maximum(y - 16, 0)
    return np.stack(
        [
            (298 * c + 409 * v + 128) // 256,
            (298 * c - 100 * u - 208 * v + 128) // 256,
            (298 * c + 516 * u + 128) // 256,
        ],
        axis=-1,
    ).clip(0, 255).astype(np.uint8)


def components(mask: np.ndarray) -> list[dict[str, int]]:
    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    found: list[dict[str, int]] = []
    for row, col in zip(*np.nonzero(mask & ~visited)):
        if visited[row, col]:
            continue
        queue = deque([(int(row), int(col))])
        visited[row, col] = True
        points: list[tuple[int, int]] = []
        while queue:
            current_row, current_col = queue.popleft()
            points.append((current_row, current_col))
            for row_delta in (-1, 0, 1):
                for col_delta in (-1, 0, 1):
                    if row_delta == 0 and col_delta == 0:
                        continue
                    next_row = current_row + row_delta
                    next_col = current_col + col_delta
                    if (0 <= next_row < height and 0 <= next_col < width and
                            mask[next_row, next_col] and not visited[next_row, next_col]):
                        visited[next_row, next_col] = True
                        queue.append((next_row, next_col))
        rows = [point[0] for point in points]
        cols = [point[1] for point in points]
        found.append(
            {
                "pixels": len(points),
                "row_min": min(rows),
                "row_max": max(rows),
                "col_min": min(cols),
                "col_max": max(cols),
            }
        )
    return sorted(found, key=lambda item: item["pixels"], reverse=True)


for name, layout in LAYOUTS.items():
    rgb = rgb_for(layout)
    Image.fromarray(rgb, "RGB").save(ROOT / f"frame-rgb-{name}.png")
    Image.fromarray(rgb[135:200, 115:220], "RGB").save(ROOT / f"frame-rgb-target-crop-{name}.png")
    r = rgb[:, :, 0].astype(np.int16)
    g = rgb[:, :, 1].astype(np.int16)
    b = rgb[:, :, 2].astype(np.int16)
    mask = (r > 70) & (r > g * 1.25 + 10) & (r > b * 1.25 + 10)
    found = components(mask)
    image = Image.fromarray(rgb, "RGB")
    draw = ImageDraw.Draw(image)
    for index, item in enumerate(found[:12]):
        if item["pixels"] < 8:
            continue
        color = "white" if index == 0 else "yellow"
        draw.rectangle(
            (item["col_min"], item["row_min"], item["col_max"], item["row_max"]),
            outline=color,
            width=1,
        )
        draw.text((item["col_min"], max(0, item["row_min"] - 10)), str(index), fill=color)
    image.save(ROOT / f"red-components-{name}.png")
    print(name, "red_pixels", int(mask.sum()), "top_components", found[:12])
