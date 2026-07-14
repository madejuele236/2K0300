# ML classifier selection

The detector and ROI sampler are classifier-independent. `MlClassifier` is the
runtime contract: one valid `32x32 gray8` ROI enters and one raw parent-class
result (`0=supplies`, `1=vehicle`, `2=weapon`) leaves. Class-to-maneuver mapping
remains a separate scene policy.

Exactly one backend is selected at CMake configure time:

```bash
cmake -S new/user -B new/out -DLS2K_ML_CLASSIFIER=V9
cmake -S new/user -B new/out-tflite -DLS2K_ML_CLASSIFIER=TFLITE_INT8
```

CMake translates this selection into the global
`LS2K_ML_CLASSIFIER_BACKEND` macro. The V9 build generates and links only the
126-bit descriptor/Hamming table. The TFLite build generates and links only
the frozen `parent_int8.tflite` bytes plus its five required TFLM operators.
Unsupported selector values fail configuration.

The TFLite contract is fixed and checked at initialization:

- artifact SHA-256:
  `7e128f963b576a23868a4519972caa5f23cff6d69e6f74097062e7a6efe913df`
- input: `int8[1,32,32,1]`, scale `1/255`, zero point `-128`; therefore the
  existing gray8 ROI is converted exactly as `int8 = gray8 - 128`
- output: `int8[1,6]`; the training exporter classifies with argmax over the
  first three parent logits only
- class ordering: `supplies`, `vehicle`, `weapon`
- TFLM tensor arena: 16 KiB; the frozen model reports 4656 used bytes on the
  board. The arena is allocated lazily only when ML is enabled and classifier
  initialization runs.

`ml_classifier_board_probe` is built for the selected backend and accepts the
same saved ROI files, allowing an A/B comparison without changing detector,
crop geometry, frames, or class mapping.

The existing `ML.V9.MIN_MARGIN` field is currently the shared integer margin
gate for both backends. `MAX_BEST_DISTANCE` applies only when the selected
result declares a Hamming distance (V9); TFLite has logits and therefore does
not invent a distance value.
