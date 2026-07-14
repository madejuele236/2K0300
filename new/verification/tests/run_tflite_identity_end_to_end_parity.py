#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np
import tensorflow as tf


REPO_ROOT = Path(__file__).resolve().parents[3]
MODEL = REPO_ROOT / "model_training/experiments/v8_parent_c2612_d4_multiteacher_20260520_0001/s2d_c2_6_12_d4_shadow_init/parent_int8.tflite"
PARAMS = REPO_ROOT / "model_training/experiments/v8_input_canonicalization_20260522_0001/d4best_recompile_probe/identity_best_params.npz"
RESULT_PATTERN = re.compile(
    r"^result path=(?P<path>\S+) raw_class=(?P<class>\d+) class_name=\S+ "
    r"margin=(?P<margin>\d+) score0=(?P<score0>-?\d+) "
    r"score1=(?P<score1>-?\d+) score2=(?P<score2>-?\d+) feature_valid=true "
    r"feature0=(?P<feature0>-?\d+) feature1=(?P<feature1>-?\d+) "
    r"feature2=(?P<feature2>-?\d+) feature3=(?P<feature3>-?\d+)"
)


def score(feature: np.ndarray, prototypes: np.ndarray, parents: np.ndarray) -> tuple[int, tuple[int, int, int], int]:
    delta = prototypes.astype(np.int32) - feature.astype(np.int32)[None, :]
    prototype_distances = np.sum(delta * delta, axis=1)
    class_distances = tuple(
        int(np.min(prototype_distances[parents == parent])) for parent in range(3)
    )
    predicted = min(range(3), key=lambda parent: (class_distances[parent], parent))
    second = min(distance for parent, distance in enumerate(class_distances) if parent != predicted)
    return predicted, class_distances, second - class_distances[predicted]


def load_board_results(path: Path) -> dict[str, tuple[int, int, tuple[int, int, int], tuple[int, int, int, int]]]:
    results: dict[str, tuple[int, int, tuple[int, int, int], tuple[int, int, int, int]]] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        match = RESULT_PATTERN.match(line)
        if match is None:
            continue
        results[Path(match.group("path")).name] = (
            int(match.group("class")),
            int(match.group("margin")),
            tuple(-int(match.group(f"score{parent}")) for parent in range(3)),
            tuple(int(match.group(f"feature{coordinate}")) for coordinate in range(4)),
        )
    return results


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare independent TensorFlow d4+identity results with board TFLM results."
    )
    parser.add_argument("--board-log", type=Path, required=True)
    parser.add_argument(
        "--resolver",
        choices=("training", "reference"),
        default="training",
        help="training uses the experiment's default interpreter; reference matches board TFLM kernels",
    )
    parser.add_argument("--allow-mismatch", action="store_true")
    parser.add_argument("roi", type=Path, nargs="+")
    args = parser.parse_args()

    with np.load(PARAMS, allow_pickle=False) as payload:
        prototypes = np.asarray(payload["prototypes_int8"], dtype=np.int8)
        parents = np.asarray(payload["prototype_parent"], dtype=np.int64)
    board_results = load_board_results(args.board_log)

    resolver = (
        tf.lite.experimental.OpResolverType.AUTO
        if args.resolver == "training"
        else tf.lite.experimental.OpResolverType.BUILTIN_REF
    )
    interpreter = tf.lite.Interpreter(
        model_path=str(MODEL), experimental_op_resolver_type=resolver
    )
    interpreter.allocate_tensors()
    input_info = interpreter.get_input_details()[0]
    output_info = interpreter.get_output_details()[0]
    if tuple(input_info["shape"]) != (1, 32, 32, 1) or input_info["dtype"] != np.int8:
        raise AssertionError(f"unexpected input contract: {input_info}")
    if tuple(output_info["shape"]) != (1, 4) or output_info["dtype"] != np.int8:
        raise AssertionError(f"unexpected output contract: {output_info}")

    exact_matches = 0
    feature_matches = 0
    class_matches = 0
    for roi_path in args.roi:
        gray = np.fromfile(roi_path, dtype=np.uint8)
        if gray.size != 32 * 32:
            raise AssertionError(f"ROI must contain 1024 bytes: {roi_path}")
        quantized = (gray.astype(np.int16) - 128).astype(np.int8).reshape(1, 32, 32, 1)
        interpreter.set_tensor(input_info["index"], quantized)
        interpreter.invoke()
        feature = interpreter.get_tensor(output_info["index"])[0].astype(np.int8)
        predicted, distances, margin = score(feature, prototypes, parents)
        actual = board_results.get(roi_path.name)
        expected = (predicted, margin, distances, tuple(int(value) for value in feature))
        if actual is not None and actual[0] == predicted:
            class_matches += 1
        if actual is not None and actual[3] == expected[3]:
            feature_matches += 1
        if actual == expected:
            exact_matches += 1
        elif not args.allow_mismatch:
            raise AssertionError(
                f"board mismatch path={roi_path} feature={feature.tolist()} "
                f"expected={expected} actual={actual}"
            )
        print(
            f"parity path={roi_path.name} feature={feature.tolist()} "
            f"class={predicted} distances={distances} margin={margin}"
        )

    if len(board_results) != len(args.roi):
        raise AssertionError(
            f"board/result count mismatch: board={len(board_results)} roi={len(args.roi)}"
        )
    print(
        f"tflite_identity_end_to_end_parity resolver={args.resolver} roi={len(args.roi)} "
        f"exact={exact_matches} feature={feature_matches} class={class_matches}"
    )
    if not args.allow_mismatch and exact_matches != len(args.roi):
        raise AssertionError("exact board parity was required")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
