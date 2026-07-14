#!/usr/bin/env python3
"""Report ML red-marker BEV geometry and YUYV colour statistics from JSONL annotations.

Each non-empty JSONL record describes one raw frame::

  {"frame_path":"frame.yuyv","format":"yuyv","width":320,"height":240,
   "stride":640,
   "bev_projector":{"SOURCE_ROW_0":219.0, ...,
                    "TARGET_LATERAL_3":0.2316051101},
   "corners":[{"row_px":120,"col_px":100}, ... four points ...]}

``bev_projector`` may instead contain ``source_points``/``target_points`` arrays,
or ``bev_projector_config_path`` may point to a JSON runtime-parameter snapshot.
Paths are resolved relative to the manifest.  This tool is deliberately read-only:
it reports recommendations but never edits runtime configuration.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path
from typing import Any, Iterable


def solve(matrix: list[list[float]], rhs: list[float]) -> list[float]:
    n = len(rhs)
    augmented = [matrix[row][:] + [rhs[row]] for row in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(augmented[row][col]))
        if abs(augmented[pivot][col]) < 1e-12:
            raise ValueError("BEV projector four-point system is singular")
        augmented[col], augmented[pivot] = augmented[pivot], augmented[col]
        scale = augmented[col][col]
        augmented[col] = [value / scale for value in augmented[col]]
        for row in range(n):
            if row == col:
                continue
            factor = augmented[row][col]
            augmented[row] = [a - factor * b for a, b in zip(augmented[row], augmented[col])]
    return [augmented[row][-1] for row in range(n)]


def homography(source: list[dict[str, Any]], target: list[dict[str, Any]]) -> list[float]:
    if len(source) != 4 or len(target) != 4:
        raise ValueError("BEV projector requires exactly four source and target points")
    matrix: list[list[float]] = []
    rhs: list[float] = []
    for image, bev in zip(source, target):
        row = finite(image["row_px"], "source row_px")
        col = finite(image["col_px"], "source col_px")
        forward = finite(bev["forward_m"], "target forward_m")
        lateral = finite(bev["lateral_m"], "target lateral_m")
        matrix.append([row, col, 1.0, 0.0, 0.0, 0.0, -forward * row, -forward * col])
        rhs.append(forward)
        matrix.append([0.0, 0.0, 0.0, row, col, 1.0, -lateral * row, -lateral * col])
        rhs.append(lateral)
    return solve(matrix, rhs)


def project(h: list[float], row: float, col: float) -> tuple[float, float]:
    denominator = h[6] * row + h[7] * col + 1.0
    if not math.isfinite(denominator) or abs(denominator) < 1e-12:
        raise ValueError("annotated corner projects to infinity")
    result = ((h[0] * row + h[1] * col + h[2]) / denominator,
              (h[3] * row + h[4] * col + h[5]) / denominator)
    if not all(math.isfinite(value) for value in result):
        raise ValueError("annotated corner projection is non-finite")
    return result


def finite(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def ordered_corners(corners: list[dict[str, Any]]) -> list[tuple[float, float]]:
    if len(corners) != 4:
        raise ValueError("each frame requires exactly four annotated corners")
    points = [(finite(point["row_px"], "corner row_px"),
               finite(point["col_px"], "corner col_px")) for point in corners]
    if len(set(points)) != 4:
        raise ValueError("annotated corners must be distinct")
    center_row = sum(point[0] for point in points) / 4.0
    center_col = sum(point[1] for point in points) / 4.0
    return sorted(points, key=lambda point: math.atan2(point[0] - center_row,
                                                       point[1] - center_col))


def projector_points(payload: dict[str, Any], base: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    projector: Any
    if "bev_projector_config_path" in payload:
        config_path = (base / str(payload["bev_projector_config_path"])).resolve()
        projector = json.loads(config_path.read_text(encoding="utf-8"))
        projector = projector.get("BEV_PROJECTOR", projector)
    else:
        projector = payload.get("bev_projector", payload.get("BEV_PROJECTOR"))
    if not isinstance(projector, dict):
        raise ValueError("record must provide bev_projector or bev_projector_config_path")
    if "source_points" in projector and "target_points" in projector:
        return list(projector["source_points"]), list(projector["target_points"])
    source = [{"row_px": projector[f"SOURCE_ROW_{index}"],
               "col_px": projector[f"SOURCE_COL_{index}"]} for index in range(4)]
    target = [{"forward_m": projector[f"TARGET_FORWARD_{index}"],
               "lateral_m": projector[f"TARGET_LATERAL_{index}"]} for index in range(4)]
    return source, target


def percentile(values: Iterable[float], probability: float) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        raise ValueError("cannot summarize an empty sample")
    index = probability * (len(ordered) - 1)
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    fraction = index - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def distribution(values: Iterable[float]) -> dict[str, float | int]:
    data = list(values)
    return {
        "count": len(data),
        "median": statistics.median(data),
        "p05": percentile(data, 0.05),
        "p95": percentile(data, 0.95),
    }


def polygon_area(points: list[tuple[float, float]]) -> float:
    # Tuple order is (x=forward/row, y=lateral/col); the formula is axis agnostic.
    return abs(sum(points[index][0] * points[(index + 1) % 4][1] -
                   points[(index + 1) % 4][0] * points[index][1]
                   for index in range(4))) * 0.5


def point_in_polygon(row: float, col: float, polygon: list[tuple[float, float]]) -> bool:
    inside = False
    previous = polygon[-1]
    for current in polygon:
        r0, c0 = previous
        r1, c1 = current
        crosses = ((c0 > col) != (c1 > col))
        if crosses:
            boundary_row = (r1 - r0) * (col - c0) / (c1 - c0) + r0
            if row < boundary_row:
                inside = not inside
        previous = current
    return inside


def annotated_yuv(frame: bytes, width: int, height: int, stride: int,
                  corners: list[tuple[float, float]]) -> tuple[list[int], list[int], list[int]]:
    min_row = max(0, int(math.floor(min(point[0] for point in corners))))
    max_row = min(height - 1, int(math.ceil(max(point[0] for point in corners))))
    min_col = max(0, int(math.floor(min(point[1] for point in corners))))
    max_col = min(width - 1, int(math.ceil(max(point[1] for point in corners))))
    y_values: list[int] = []
    u_values: list[int] = []
    v_values: list[int] = []
    for row in range(min_row, max_row + 1):
        for col in range(min_col, max_col + 1):
            if not point_in_polygon(row + 0.5, col + 0.5, corners):
                continue
            pair_offset = row * stride + (col // 2) * 4
            y_offset = pair_offset + (0 if col % 2 == 0 else 2)
            y_values.append(frame[y_offset])
            u_values.append(frame[pair_offset + 1])
            v_values.append(frame[pair_offset + 3])
    if not y_values:
        raise ValueError("annotated polygon contains no sampleable pixel centres")
    return y_values, u_values, v_values


def geometry(bev: list[tuple[float, float]]) -> tuple[float, float, float, float]:
    vectors = [(bev[(index + 1) % 4][0] - bev[index][0],
                bev[(index + 1) % 4][1] - bev[index][1]) for index in range(4)]
    lengths = [math.hypot(vector[0], vector[1]) for vector in vectors]
    pair0 = (lengths[0] + lengths[2]) * 0.5
    pair1 = (lengths[1] + lengths[3]) * 0.5
    long_indices = (0, 2) if pair0 >= pair1 else (1, 3)
    long_edge = max(pair0, pair1)
    short_edge = min(pair0, pair1)
    if short_edge <= 0.0:
        raise ValueError("annotated rectangle has zero BEV edge length")
    vector = vectors[long_indices[0]]
    # Acute angle to the vehicle lateral axis; 0 means a horizontal marker.
    orientation = math.atan2(abs(vector[0]), abs(vector[1]))
    rectangularity = min(1.0, polygon_area(bev) / (long_edge * short_edge))
    return long_edge, short_edge, orientation, rectangularity


def analyze_record(payload: dict[str, Any], base: Path, line_number: int) -> tuple[dict[str, Any], dict[str, list[int]]]:
    frame_path = (base / str(payload["frame_path"])).resolve()
    image_format = str(payload.get("format", "")).lower()
    if image_format not in ("yuyv", "yuyv422", "yuyv_422"):
        raise ValueError(f"line {line_number}: format must be YUYV 4:2:2")
    width = int(payload["width"])
    height = int(payload["height"])
    stride = int(payload.get("stride", width * 2))
    if width <= 0 or height <= 0 or width % 2 != 0 or stride < width * 2:
        raise ValueError(f"line {line_number}: invalid YUYV dimensions or stride")
    raw = frame_path.read_bytes()
    required = stride * height
    if len(raw) != required:
        raise ValueError(f"line {line_number}: {frame_path} has {len(raw)} bytes, expected {required}")
    image_corners = ordered_corners(list(payload["corners"]))
    source, target = projector_points(payload, base)
    h = homography(source, target)
    bev_corners = [project(h, row, col) for row, col in image_corners]
    long_edge, short_edge, orientation, rectangularity = geometry(bev_corners)
    y_values, u_values, v_values = annotated_yuv(raw, width, height, stride, image_corners)
    result = {
        "line": line_number,
        "frame_path": str(frame_path),
        "long_edge_m": long_edge,
        "short_edge_m": short_edge,
        "long_edge_to_lateral_rad": orientation,
        "rectangularity": rectangularity,
        "bev_corners": [{"forward_m": point[0], "lateral_m": point[1]} for point in bev_corners],
        "y": distribution(y_values),
        "u": distribution(u_values),
        "v": distribution(v_values),
    }
    return result, {"y": y_values, "u": u_values, "v": v_values}


def recommendation(results: list[dict[str, Any]], colours: dict[str, list[int]]) -> dict[str, Any]:
    longs = [result["long_edge_m"] for result in results]
    shorts = [result["short_edge_m"] for result in results]
    orientations = [result["long_edge_to_lateral_rad"] for result in results]
    expected_long = statistics.median(longs)
    expected_short = statistics.median(shorts)
    long_p05, long_p95 = percentile(longs, 0.05), percentile(longs, 0.95)
    short_p05, short_p95 = percentile(shorts, 0.05), percentile(shorts, 0.95)
    return {
        "EXPECTED_LONG_EDGE_M": expected_long,
        "EXPECTED_SHORT_EDGE_M": expected_short,
        "LONG_EDGE_TOLERANCE_M": max(expected_long - long_p05, long_p95 - expected_long),
        "SHORT_EDGE_TOLERANCE_M": max(expected_short - short_p05, short_p95 - expected_short),
        "MAX_LONG_EDGE_TO_LATERAL_RAD": percentile(orientations, 0.95),
        "RED_Y_MIN": int(math.floor(percentile(colours["y"], 0.05) + 1e-9)),
        "RED_Y_MAX": int(math.ceil(percentile(colours["y"], 0.95) - 1e-9)),
        "RED_U_MIN": int(math.floor(percentile(colours["u"], 0.05) + 1e-9)),
        "RED_U_MAX": int(math.ceil(percentile(colours["u"], 0.95) - 1e-9)),
        "RED_V_MIN": int(math.floor(percentile(colours["v"], 0.05) + 1e-9)),
        "RED_V_MAX": int(math.ceil(percentile(colours["v"], 0.95) - 1e-9)),
        "basis": "median expected size; p05/p95 absolute coverage; p95 orientation; pooled annotated YUV p05/p95",
    }


def write_csv(path: Path, results: list[dict[str, Any]]) -> None:
    fields = ["line", "frame_path", "long_edge_m", "short_edge_m",
              "long_edge_to_lateral_rad", "rectangularity",
              "y_median", "y_p05", "y_p95", "u_median", "u_p05", "u_p95",
              "v_median", "v_p05", "v_p95"]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for result in results:
            flat = {key: result[key] for key in fields[:6]}
            for channel in ("y", "u", "v"):
                for statistic in ("median", "p05", "p95"):
                    flat[f"{channel}_{statistic}"] = result[channel][statistic]
            writer.writerow(flat)


def outlier_frames(results: list[dict[str, Any]], aggregate: dict[str, Any]) -> list[dict[str, Any]]:
    outliers: list[dict[str, Any]] = []
    for result in results:
        reasons: list[str] = []
        for metric in ("long_edge_m", "short_edge_m"):
            if result[metric] < aggregate[metric]["p05"] or result[metric] > aggregate[metric]["p95"]:
                reasons.append(f"{metric}_outside_p05_p95")
        if result["long_edge_to_lateral_rad"] > aggregate["long_edge_to_lateral_rad"]["p95"]:
            reasons.append("orientation_above_p95")
        if result["rectangularity"] < aggregate["rectangularity"]["p05"]:
            reasons.append("rectangularity_below_p05")
        for channel in ("y", "u", "v"):
            median = result[channel]["median"]
            if median < aggregate[channel]["p05"] or median > aggregate[channel]["p95"]:
                reasons.append(f"{channel}_median_outside_pooled_p05_p95")
        if reasons:
            outliers.append({"line": result["line"],
                             "frame_path": result["frame_path"],
                             "reasons": reasons})
    return outliers


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="JSONL annotated-frame manifest")
    parser.add_argument("--output", "--json-output", dest="json_output", type=Path,
                        help="optional JSON report path; stdout is used when omitted")
    parser.add_argument("--csv-output", type=Path, help="optional per-frame CSV report path")
    args = parser.parse_args()
    base = args.input.resolve().parent
    results: list[dict[str, Any]] = []
    colours: dict[str, list[int]] = {"y": [], "u": [], "v": []}
    for line_number, text in enumerate(args.input.read_text(encoding="utf-8").splitlines(), 1):
        if not text.strip():
            continue
        try:
            payload = json.loads(text)
            result, samples = analyze_record(payload, base, line_number)
        except (KeyError, TypeError, ValueError, OSError, json.JSONDecodeError) as error:
            raise ValueError(f"invalid annotation at line {line_number}: {error}") from error
        results.append(result)
        for channel in colours:
            colours[channel].extend(samples[channel])
    if not results:
        raise ValueError("annotation manifest contains no records")
    aggregate = {
        "long_edge_m": distribution(result["long_edge_m"] for result in results),
        "short_edge_m": distribution(result["short_edge_m"] for result in results),
        "long_edge_to_lateral_rad": distribution(result["long_edge_to_lateral_rad"] for result in results),
        "rectangularity": distribution(result["rectangularity"] for result in results),
        "y": distribution(colours["y"]),
        "u": distribution(colours["u"]),
        "v": distribution(colours["v"]),
    }
    report = {
        "artifact": "ml_four_corner_calibration_report",
        "frame_count": len(results),
        "frames": results,
        "aggregate": aggregate,
        "outlier_frames": outlier_frames(results, aggregate),
        "recommended_ml_roi_parameters": recommendation(results, colours),
        "writes_runtime_configuration": False,
    }
    output = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.json_output:
        args.json_output.write_text(output, encoding="utf-8")
    else:
        print(output, end="")
    if args.csv_output:
        write_csv(args.csv_output, results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
