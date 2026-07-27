#!/usr/bin/env python3
"""Focused host tests for steering-media primary/auxiliary payload layouts."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "new" / "user"))

from steering_media_capture import SteeringMediaListener  # noqa: E402
from steering_media_live_server import LiveFrameHub, _split_image_payload, _viewer_html  # noqa: E402


def image_header(frame_id: int = 7) -> dict[str, object]:
    return {
        "type": "image_frame",
        "frame_id": frame_id,
        "width": 2,
        "height": 2,
        "pixel_format": "gray8",
        "payload_encoding": "raw",
    }


def auxiliary_layout(primary_size: int = 4, auxiliary_size: int = 1024) -> dict[str, object]:
    return {
        "version": 1,
        "primary": {"offset": 0, "size": primary_size},
        "auxiliary": {
            "name": "ml_roi",
            "offset": primary_size,
            "size": auxiliary_size,
            "width": 32,
            "height": 32,
            "pixel_format": "gray8",
        },
    }


class SteeringMediaAuxPayloadTest(unittest.TestCase):
    def make_listener(self, output_dir: Path) -> SteeringMediaListener:
        (output_dir / "frames").mkdir(parents=True)
        (output_dir / "ml_roi").mkdir(parents=True)
        return SteeringMediaListener("127.0.0.1", 0, output_dir)

    def read_metadata(self, output_dir: Path) -> dict[str, object]:
        return json.loads((output_dir / "frame_metadata.jsonl").read_text(encoding="utf-8"))

    def test_legacy_payload_is_saved_unchanged(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            output_dir = Path(temporary_dir)
            listener = self.make_listener(output_dir)
            primary = bytes((1, 2, 3, 4))

            listener._handle_frame(image_header(), primary, 100)

            self.assertEqual((output_dir / "frames" / "frame-000007.raw").read_bytes(), primary)
            self.assertFalse((output_dir / "ml_roi" / "frame-000007.raw").exists())
            metadata = self.read_metadata(output_dir)
            self.assertIsNone(metadata["auxiliary_payload"])
            self.assertIsNone(metadata["ml_roi_path"])
            self.assertIsNone(listener._summary["receiver_error"])

    def test_valid_32x32_auxiliary_payload_is_split_and_saved(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            output_dir = Path(temporary_dir)
            listener = self.make_listener(output_dir)
            header = image_header()
            header["payload_layout"] = auxiliary_layout()
            primary = bytes((1, 2, 3, 4))
            auxiliary = bytes(index % 256 for index in range(1024))

            listener._handle_frame(header, primary + auxiliary, 100)

            self.assertEqual((output_dir / "frames" / "frame-000007.raw").read_bytes(), primary)
            self.assertEqual((output_dir / "ml_roi" / "frame-000007.raw").read_bytes(), auxiliary)
            metadata = self.read_metadata(output_dir)
            self.assertEqual(metadata["auxiliary_payload"]["name"], "ml_roi")
            self.assertEqual(metadata["auxiliary_payload"]["width"], 32)
            self.assertEqual(metadata["payload_layout"]["version"], 1)
            self.assertEqual(listener._summary["ml_roi_count"], 1)
            self.assertIsNone(listener._summary["receiver_error"])

    def test_malformed_layout_or_length_is_rejected(self) -> None:
        cases = [
            ({**auxiliary_layout(), "version": 2}, 1028),
            (auxiliary_layout(auxiliary_size=1023), 1027),
            (auxiliary_layout(), 1027),
        ]
        for layout, payload_size in cases:
            with self.subTest(layout=layout, payload_size=payload_size):
                with tempfile.TemporaryDirectory() as temporary_dir:
                    output_dir = Path(temporary_dir)
                    listener = self.make_listener(output_dir)
                    header = image_header()
                    header["payload_layout"] = layout
                    listener._handle_frame(header, bytes(payload_size), 100)
                    self.assertIn("invalid steering image payload layout", listener._summary["receiver_error"])
                    self.assertFalse((output_dir / "frames" / "frame-000007.raw").exists())

    def test_live_hub_preserves_auxiliary_for_layout_aware_viewer(self) -> None:
        header = image_header()
        header["payload_layout"] = auxiliary_layout()
        primary = bytes((1, 2, 3, 4))
        auxiliary = bytes(1024)
        combined = primary + auxiliary
        split_primary, split_layout, split_auxiliary = _split_image_payload(header, combined)
        self.assertEqual(split_primary, primary)
        self.assertEqual(split_auxiliary, auxiliary)
        self.assertEqual(split_layout["name"], "ml_roi")

        hub = LiveFrameHub()
        hub.publish(header, combined, 123)
        _, message = hub.latest()
        self.assertIsNotNone(message)
        header_len = int.from_bytes(message[0:4], "big")
        payload_len = int.from_bytes(message[4:8], "big")
        forwarded_payload = message[8 + header_len :]
        self.assertEqual(payload_len, len(combined))
        self.assertEqual(forwarded_payload, combined)
        self.assertEqual(hub.summary()["auxiliary_image_messages"], 1)

    def test_live_viewer_exposes_board_owned_ml_config_and_telemetry(self) -> None:
        html = _viewer_html().decode("utf-8")
        for field_id in (
            "mlSummary",
            "mlConfig",
            "mlV9Gate",
            "mlMapping",
            "mlManeuver",
            "mlDetector",
            "mlClassification",
            "mlAction",
            "mlState",
            "mlTiming",
        ):
            self.assertIn(f'id="{field_id}"', html)
        self.assertIn("const mlConfig = config?.ML", html)
        self.assertIn("const ml = steering.ml || {}", html)
        self.assertIn('nested(ml, ["classification", "margin"])', html)
        self.assertIn("ml.takeover_selected", html)


if __name__ == "__main__":
    unittest.main()
