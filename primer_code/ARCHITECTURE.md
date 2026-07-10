# Primer Code Layering Contract

This refactor preserves the behavior of the original `primer_code` baseline at
commit `70f3c71ea`.  `new/code` is used only as a structural reference for
dependency direction, narrow fact types, orchestration ownership, and isolated
verification.  No algorithm, formula, threshold, state transition, or concrete
implementation is copied from it.

## Non-negotiable behavior invariants

- Startup remains `init -> pid_init -> image_init -> start 5 ms PIT -> Param_Init`.
- The PIT callback remains `IMU -> periodic range sample -> encoder update ->
  distance accumulation -> run-state control -> diagnostics`.
- The foreground loop keeps its buzzer, frame counter, stopped-state UI/key
  handling, and unconditional `ImageDeal` ordering.
- All original numeric conversions, truncation, unusual PID formulas, disabled
  limits, default values, and failure behavior remain unchanged.
- Vision keeps its exact stage order.  In particular, roundabout stages may
  modify the binary image and then re-run top/side-line extraction before the
  centerline and steering error are computed.
- Existing state codes, stale-state behavior, commented-out runtime paths, and
  currently unreachable branches are preserved.  Correcting them is a separate
  behavior change, not part of this refactor.
- The original thread-sharing behavior remains unsynchronized.  Introducing a
  snapshot or lock could change timing and is outside this strictly equivalent
  change.

## Dependency rule

Dependencies point toward contracts and facts; only `runtime` may compose
independent owners.

```text
user/main
    -> runtime
        -> platform
        -> estimation
        -> control
        -> parameters
        -> vision
        -> inference
        -> presentation
        -> transport

all owners -> port contracts / their own internal facts
```

- `port/` contains data-only cross-layer contracts.  It owns no device and runs
  no algorithm.
- `platform/` owns vendor device objects and physical I/O conventions.
- `estimation/` owns IMU conversion, calibration, AHRS, and filtering.
- `control/` owns PID state, formulas, target mixing, and actuator values, but
  not devices or image processing.
- `parameters/` owns defaults, parsing, and persistence.
- `vision/` owns frame processing, track facts, scene state machines, line
  repair, and steering observations.  Its pipeline is the sole owner of the
  original stage order.
- `inference/` owns classifier configuration and inference.
- `transport/` owns HTTP/MJPEG protocol and server state.
- `presentation/` observes published facts and owns keys/display behavior; it
  does not produce control decisions.
- `runtime/` is the only cross-owner orchestrator and owns lifecycle/cadence.
- Root-level legacy headers are compatibility facades only.  Active
  implementations must not use a common application umbrella header to learn
  unrelated owners.

## Reference-to-target mapping

| Original owner | Refactored owner | Action | Behavior boundary |
|---|---|---|---|
| `project/user/main.cpp` | `runtime/` + thin `user/main.cpp` | Adapt structure | Preserve exact startup, callback, and foreground-loop order |
| `project/code/init.*` | `platform/`, `runtime/` | Split owner | Preserve global device construction, cleanup order, encoder math, and PWM signs |
| `project/code/filt.*` | `estimation/` | Move owner | Preserve calibration count, signs, constants, in-place quaternion update, and yaw wrap behavior |
| `project/code/control.*` | `control/` | Move owner | Preserve every formula, clamp, conversion, and state update |
| `project/code/flash.*` | `parameters/` | Move owner | Preserve defaults, permissive parsing, partial-load semantics, and file format |
| `project/code/image.*` | `vision/` pipeline and stages | Split owner | Preserve function bodies, mutable facts, stage order, and all scene state codes |
| `project/code/lq_ncnn.*` | `inference/` | Move owner | Preserve model paths, blob names, preprocessing, exceptions, and softmax |
| `project/code/show.*` | `presentation/` | Move owner | Preserve key/page behavior and side effects |
| `project/code/ww_transmission.*` | `transport/` | Move owner | Preserve routes, status-line behavior, frame publication, and threading |
| `project/code/lq_camera_ex.hpp` | `platform/camera/` contract facade | Adapt structure | Preserve the complete public class interface and pImpl boundary |

`Replicate` is intentionally not used above: the target retains the original
primer algorithms.  Only the structural ownership model is adapted.

## Coverage gate

Each mapping is complete only when all of the following are true:

1. every original definition has exactly one active target owner;
2. old public symbols required by current callers remain available through a
   narrow compatibility facade;
3. the explicit source list builds the same executable inputs;
4. forbidden include/dependency checks pass;
5. static behavior comparison finds no unreviewed formula, constant, ordering,
   state-transition, or side-effect change;
6. the independent verifier returns `PASS` twice consecutively.

Current implementation coverage before the independent verifier gate:

| Contract | Coverage | Evidence |
|---|---|---|
| original definition ownership | complete | 102/102 function mapping and successful single link |
| compatibility symbols | complete | all 2,143 baseline global definitions remain available |
| explicit build ownership | complete | 23/23 layered application sources listed by CMake |
| dependency boundaries | complete | 43 active files pass the architecture scan |
| formula/order preservation | complete (static) | token-equivalent bodies plus checked façade-to-core delegation |
| hardware/runtime equivalence | not covered | no board, camera, model, or motion run was performed |
| independent reviewer | pending | requires two consecutive context-free `PASS` verdicts |

## Known limits of static equivalence

The baseline contains cross-thread data races and hardware-dependent behavior.
Static analysis and cross-compilation can prove ownership, source coverage, and
preservation of visible code paths; they cannot prove device availability,
actual 5 ms scheduling, camera/model inputs, or physical motor direction.  No
hardware-backed equivalence is claimed without board evidence.
