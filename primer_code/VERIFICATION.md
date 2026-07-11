# Refactor verification record

## Baseline

- Git baseline: `70f3c71ea` (`Add original primer code baseline`).
- Isolated source tree: detached worktree at that commit.
- Configure command:
  `cmake -S primer_code/project/user -B primer_code/project/out-local`
- Build command:
  `cmake --build primer_code/project/out-local --parallel`
- Result: configure, all 31 object builds, and final link passed with the
  LoongArch GNU 8.3.0 toolchain.
- Baseline executable SHA-256:
  `01a67a589b413b133963ae4cf9258738bd687db0e4149d9c9653f622967657ea`.

The original build emitted warnings that are treated as baseline behavior, not
refactor defects to repair:

- duplicate `MIN_RED_AREA` macro definitions;
- the mutually exclusive `small_rock` entry predicates;
- misleading indentation in `my_sobel_dajin` and `ImageDeal`;
- the unused `speed_add` local in the PIT callback and `temp2` in
  `my_sobel_dajin`.

## Required final gates

| Gate | What it proves | What it does not prove |
|---|---|---|
| token-level original function comparison | formulas, literals, calls, branches, and ordering remain mechanically accounted for | hardware timing or data-race outcomes |
| dependency-boundary scan | active owners do not regain umbrella/private cross-layer knowledge | semantic quality inside a function |
| same-toolchain configure/build/link | the integrated source list and ABI are buildable | board device availability |
| baseline global-symbol subset check | original externally visible definitions remain link-visible | binary identity or runtime values |
| independent static review, twice | a context-free reviewer accepts strict static equivalence and code structure | camera, model, or physical motion behavior |

No hardware-equivalence claim is made unless the refactored binary is also run
against equivalent board, camera, model, parameter, and timing conditions.

## Integrated refactor result

- Static function comparison: 102 baseline functions discovered; 102 are
  token-identical after relocation or mechanically delegated to one
  token-equivalent owner.
- Architecture scan: 24 layered application sources exactly match the 24
  explicit CMake entries; 58 active headers/sources and 21 public layer
  headers satisfy the façade, no-public-`extern`, umbrella, private-header,
  pure-vision-facts, singular-definition-owner, service-composition-order, and
  unique-pipeline-owner rules.
- Original `init.cpp` global construction block: all 263 tokens remain in one
  composition translation unit with identical order and initializer
  expressions; baseline and refactor block SHA-256 are both
  `dc6729f465c4ed9673df4ea13f67efe2bac3f8994d7ccba7d4154e105f158c8c`.
- Refactored clean configure/build/link: PASS from the previously nonexistent
  `/tmp/2K0300-primer-refactored-build-3` directory, using the same LoongArch
  GNU 8.3.0 toolchain and OpenCV 4.10 installation as the baseline.
- Warning profile: the refactored build reproduces the baseline warnings listed
  above at their new owner locations; it introduces no new compiler warning.
- ABI surface: all 2,143 globally defined baseline symbols remain present; the
  refactored binary has 2,169 definitions, with the additions belonging to the
  new orchestration/core boundaries.
- Refactored executable SHA-256:
  `069c63dbac6494c8d7c12f22223f614159f4f7d95063ac6581dcfbbbbc5783db`.
- `git diff --check`: PASS.

These checks establish source-level and link-level preservation.  Board,
camera, model-file, device-node, scheduler-jitter, and physical motor behavior
remain unverified because this refactor was not executed on hardware.
