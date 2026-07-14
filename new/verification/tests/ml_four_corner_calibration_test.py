#!/usr/bin/env python3
import json
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "user" / "ml_four_corner_calibration.py"


def main() -> int:
    with tempfile.TemporaryDirectory() as temporary:
        work = Path(temporary)
        frame = work / "frame.yuyv"
        frame.write_bytes(bytes([100, 50, 100, 200] * 2 * 4))
        record = {
            "frame_path": frame.name,
            "format": "yuyv",
            "width": 4,
            "height": 4,
            "stride": 8,
            "bev_projector": {
                "source_points": [
                    {"row_px": 0, "col_px": 0},
                    {"row_px": 0, "col_px": 4},
                    {"row_px": 4, "col_px": 4},
                    {"row_px": 4, "col_px": 0},
                ],
                "target_points": [
                    {"forward_m": 0, "lateral_m": 0},
                    {"forward_m": 0, "lateral_m": 4},
                    {"forward_m": 4, "lateral_m": 4},
                    {"forward_m": 4, "lateral_m": 0},
                ],
            },
            "corners": [
                {"row_px": 1, "col_px": 0},
                {"row_px": 1, "col_px": 4},
                {"row_px": 3, "col_px": 4},
                {"row_px": 3, "col_px": 0},
            ],
        }
        manifest = work / "annotations.jsonl"
        manifest.write_text(json.dumps(record) + "\n", encoding="utf-8")
        json_report = work / "report.json"
        csv_report = work / "report.csv"
        subprocess.run(
            [sys.executable, str(TOOL), str(manifest), "--output", str(json_report),
             "--csv-output", str(csv_report)],
            check=True,
        )
        report = json.loads(json_report.read_text(encoding="utf-8"))
        frame_report = report["frames"][0]
        assert report["frame_count"] == 1
        assert report["outlier_frames"] == []
        assert abs(frame_report["long_edge_m"] - 4.0) < 1e-9
        assert abs(frame_report["short_edge_m"] - 2.0) < 1e-9
        assert abs(frame_report["long_edge_to_lateral_rad"]) < 1e-9
        assert abs(frame_report["rectangularity"] - 1.0) < 1e-9
        assert frame_report["y"]["median"] == 100
        assert frame_report["u"]["median"] == 50
        assert frame_report["v"]["median"] == 200
        recommended = report["recommended_ml_roi_parameters"]
        assert recommended["EXPECTED_LONG_EDGE_M"] == 4.0
        assert recommended["EXPECTED_SHORT_EDGE_M"] == 2.0
        assert recommended["RED_Y_MIN"] == recommended["RED_Y_MAX"] == 100
        assert recommended["RED_U_MIN"] == recommended["RED_U_MAX"] == 50
        assert recommended["RED_V_MIN"] == recommended["RED_V_MAX"] == 200
        assert report["writes_runtime_configuration"] is False
        csv_text = csv_report.read_text(encoding="utf-8")
        assert "long_edge_m" in csv_text and str(frame.resolve()) in csv_text

        invalid = dict(record)
        invalid["format"] = "gray8"
        manifest.write_text(json.dumps(invalid) + "\n", encoding="utf-8")
        failed = subprocess.run(
            [sys.executable, str(TOOL), str(manifest)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        assert failed.returncode != 0
        assert "format must be YUYV" in failed.stderr
    print("ml_four_corner_calibration_test: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
