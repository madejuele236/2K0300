#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import struct
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
MODEL = REPO_ROOT / "model_training/experiments/v8_parent_c2612_d4_multiteacher_20260520_0001/s2d_c2_6_12_d4_shadow_init/parent_int8.tflite"
PARAMS = REPO_ROOT / "model_training/experiments/v8_input_canonicalization_20260522_0001/d4best_recompile_probe/identity_best_params.npz"
MODEL_SHA256 = "0362178a0f665a3bf9dde16d4f6d9883d06370f2559824ef2d723b32002600cf"
PARAMS_SHA256 = "5ac2827bfc8aa37f928802bc93f1274879e7ab73c97f8da534424094d8a701c5"


def load_generator_module():
    path = REPO_ROOT / "model_training/generate_tflite_classifier_artifact.py"
    spec = importlib.util.spec_from_file_location("tflite_artifact_generator", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def unpack_int8(array, expected_shape: tuple[int, ...], key: str) -> tuple[int, ...]:
    if array.descr != "|i1" or array.shape != expected_shape:
        raise ValueError(f"{key} contract mismatch: dtype={array.descr} shape={array.shape}")
    return struct.unpack(f"{len(array.payload)}b", array.payload)


def unpack_int64(array, expected_shape: tuple[int, ...], key: str) -> tuple[int, ...]:
    count = 1
    for dimension in expected_shape:
        count *= dimension
    if array.descr != "<i8" or array.shape != expected_shape or len(array.payload) != count * 8:
        raise ValueError(f"{key} contract mismatch: dtype={array.descr} shape={array.shape}")
    return struct.unpack(f"<{count}q", array.payload)


def score_python(feature: tuple[int, ...], prototypes: tuple[int, ...], parents: tuple[int, ...]):
    distances = [2**31 - 1, 2**31 - 1, 2**31 - 1]
    for prototype_index, parent in enumerate(parents):
        offset = prototype_index * 4
        distance = sum(
            (feature[coordinate] - prototypes[offset + coordinate]) ** 2
            for coordinate in range(4)
        )
        distances[parent] = min(distances[parent], distance)
    predicted = min(range(3), key=lambda parent: (distances[parent], parent))
    second = min(distance for parent, distance in enumerate(distances) if parent != predicted)
    return predicted, tuple(distances), second - distances[predicted]


def main() -> int:
    generator = load_generator_module()
    with zipfile.ZipFile(PARAMS) as archive:
        prototype_values = unpack_int8(generator.load_npy(archive, "prototypes_int8"), (844, 4), "prototypes_int8")
        parent_values = unpack_int64(generator.load_npy(archive, "prototype_parent"), (844,), "prototype_parent")
        embedding_values = unpack_int8(generator.load_npy(archive, "embedding_int8"), (6688, 4), "embedding_int8")
        stored_predictions = unpack_int64(generator.load_npy(archive, "int8_pred"), (6688,), "int8_pred")
        stored_margins = unpack_int64(generator.load_npy(archive, "int8_margin"), (6688,), "int8_margin")

    features = [tuple(embedding_values[offset : offset + 4]) for offset in range(0, len(embedding_values), 4)]
    expected = [score_python(feature, prototype_values, parent_values) for feature in features]
    for index, ((predicted, _distances, margin), stored_prediction, stored_margin) in enumerate(
        zip(expected, stored_predictions, stored_margins)
    ):
        if predicted != stored_prediction or margin != stored_margin:
            raise AssertionError(
                f"NPZ stored result mismatch at sample {index}: recomputed={(predicted, margin)} "
                f"stored={(stored_prediction, stored_margin)}"
            )

    with tempfile.TemporaryDirectory(prefix="ls2k_tflite_identity_") as temp_dir_text:
        temp_dir = Path(temp_dir_text)
        generated_header = temp_dir / "generated_tflite_classifier_artifact.hpp"
        generated_source = temp_dir / "generated_tflite_classifier_artifact.cpp"
        probe = temp_dir / "tflite_identity_parity_probe"
        subprocess.run(
            [
                sys.executable,
                str(REPO_ROOT / "model_training/generate_tflite_classifier_artifact.py"),
                str(MODEL),
                str(PARAMS),
                str(generated_header),
                str(generated_source),
                MODEL_SHA256,
                PARAMS_SHA256,
            ],
            check=True,
        )
        subprocess.run(
            [
                "g++",
                "-std=c++17",
                "-O2",
                "-Wall",
                "-Wextra",
                "-Werror",
                f"-I{temp_dir}",
                str(REPO_ROOT / "new/verification/tests/tflite_identity_parity_probe.cpp"),
                str(generated_source),
                "-o",
                str(probe),
            ],
            check=True,
        )
        input_text = "".join(" ".join(map(str, feature)) + "\n" for feature in features)
        result = subprocess.run([str(probe)], input=input_text, text=True, capture_output=True, check=True)

    actual_lines = result.stdout.splitlines()
    if len(actual_lines) != len(expected):
        raise AssertionError(f"C++ probe returned {len(actual_lines)} rows for {len(expected)} features")
    for index, (line, reference) in enumerate(zip(actual_lines, expected)):
        values = tuple(int(value) for value in line.split())
        wanted = (reference[0], *reference[1], reference[2])
        if values != wanted:
            raise AssertionError(f"C++ parity mismatch at sample {index}: actual={values} expected={wanted}")

    print(f"tflite_identity_parity_test passed samples={len(expected)} prototypes={len(parent_values)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
