# ML classifier selection

The detector and ROI sampler are classifier-independent. `MlClassifier` is the
runtime contract: one valid `32x32 gray8` ROI enters and one raw parent-class
result (`0=supplies`, `1=vehicle`, `2=weapon`) leaves. Class-to-maneuver mapping
remains a separate scene policy.

`RunMlObserver()` owns the per-frame detector -> ROI -> classifier -> acceptance
-> class-mapping fact chain. It has no road-path or motion-history input and
cannot publish a reference candidate. `StepMlManeuver()` is the only consumer
that may turn an accepted observation into confirmation, a locked left/right
action, scalar encoder distance progress, and an ML reference candidate. While
active, it rebuilds the path from the current selected-side boundary and shifts
that boundary outward by `ML.MANEUVER.PATH_OUTWARD_OFFSET_M`; ML deliberately
does not apply image-connectivity filtering. Missing current boundary facts do
not end the maneuver: ML retains arbitration ownership and the global reference
continuity policy decides whether the last ML reference can be held. The action
is released when either `EXIT_FORWARD_M` or `MAX_DURATION_MS` is reached. With
`ML.ENABLED=1` and
`ML.MANEUVER.ENABLED=0`, only the observer runs; maneuver memory is reset and
ordinary visual-reference arbitration and speed selection remain authoritative.

Exactly one backend is selected at CMake configure time:

```bash
cmake -S new/user -B new/out -DLS2K_ML_CLASSIFIER=V9
cmake -S new/user -B new/out-tflite -DLS2K_ML_CLASSIFIER=TFLITE_INT8
```

CMake translates this selection into the global
`LS2K_ML_CLASSIFIER_BACKEND` macro. The V9 build generates and links only the
126-bit descriptor/Hamming table. The TFLite build generates and links only
the frozen d4 `parent_int8.tflite`, the `d4best_recompile:identity` prototype
table, and its five required TFLM operators. Unsupported selector values fail
configuration. The two branches do not embed each other's artifacts.

The TFLite contract is fixed and checked at initialization:

- artifact SHA-256:
  - d4 backbone:
    `0362178a0f665a3bf9dde16d4f6d9883d06370f2559824ef2d723b32002600cf`
  - identity parameters:
    `5ac2827bfc8aa37f928802bc93f1274879e7ab73c97f8da534424094d8a701c5`
  - runtime artifact identity: SHA-256 of the exact backbone bytes followed by
    the exact identity NPZ bytes; this is what `ArtifactSha256()` publishes
- input: `int8[1,32,32,1]`, scale `1/255`, zero point `-128`; therefore the
  existing gray8 ROI is converted exactly as `int8 = gray8 - 128`
- backbone output: `int8[1,4]`, scale approximately `0.148821`, zero point
  `19`
- identity classifier: 844 `int8[4]` prototypes with parent IDs `0..2`. For
  each prototype it computes squared Euclidean distance in signed int8
  coordinates using int32 arithmetic. Each parent score is its minimum
  prototype distance; the smallest parent ID wins an exact tie. `margin` is
  the second-smallest parent distance minus the winning distance.
- `class_scores[parent]` is the negative parent distance, preserving the
  existing higher-is-better score convention. `distance_valid` is true and
  `best_distance` is the winning parent distance.
- class ordering: `supplies`, `vehicle`, `weapon`
- TFLM tensor arena: 16 KiB. The arena is allocated lazily only when ML is
  enabled and classifier initialization runs.

`ml_classifier_board_probe` is built for the selected backend and accepts the
same saved ROI files, allowing an A/B comparison without changing detector,
crop geometry, frames, or class mapping.

Acceptance and confirmation are backend-specific. `ML.V9` remains the
126-bit Hamming policy. `ML.TFLITE_IDENTITY` owns the identity scorer's
squared-L2 `MIN_MARGIN` and `MAX_BEST_DISTANCE` plus its `CONFIRM_FRAMES`.
The artifact-calibrated defaults are `1`, `2076`, and `3`: among the supplied
6688 correct NPZ samples, the maximum winning squared-L2 distance is 2076 and
1226 distances exceed 126, so applying the V9 ceiling rejects valid identity
outputs. The scene selects the policy from the classifier result's explicit
`MlClassifierBackend`; TFLite identity results must carry a valid distance and
cannot silently bypass their distance gate.

Focused host checks are:

```bash
python3 new/verification/tests/run_tflite_identity_parity_test.py
bash new/verification/tests/run_tflite_model_contract_test.sh
python3 new/verification/tests/run_tflite_identity_end_to_end_parity.py \
  --resolver reference --board-log BOARD_LOG ROI.raw
```

The parity test regenerates the artifact, checks all 6688 stored NPZ int8
features against the training classifier's stored parent predictions and
margins, then checks the same inputs through the generated C++ scorer.

The end-to-end check distinguishes two numerical contracts instead of hiding
their difference. `--resolver reference` matches the TFLM reference kernels
used on the board and must match the recorded feature, distances, class, and
margin exactly. `--resolver training --allow-mismatch` uses the experiment's
default TensorFlow Lite interpreter, which enables XNNPACK on the training
host. Quantized kernels can differ by one output unit between XNNPACK and the
reference implementation; the report therefore counts exact feature and
final-class agreement separately. The supplied prototype table remains
unchanged and no output correction is applied in firmware.
