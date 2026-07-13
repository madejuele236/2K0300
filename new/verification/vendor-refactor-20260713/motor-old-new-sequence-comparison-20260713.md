# Motor polarity and sequence comparison

Evidence date: 2026-07-13

The live board A/B disproved the earlier claim that the historical bridge polarity and the accepted `MotorDevice` have equivalent logical-forward semantics. The accepted implementation preserves the proven channel/transport/safety sequence, but it does not preserve the old sign inversion.

## Direct board observations

| Run | Source behavior and helper identity | Command and observed result |
| --- | --- | --- |
| Clean v2, rejected | `hardware_duty = -clamped`; helper SHA-256 `970c905df1a7b8d2d93dc0e9c4adafc734c9371ffb04246706d045def8dec06f` | Logical `+3000/+3000` for five seconds produced raw Encoder counts `-753/+870`. The then-current normalization, `left = -raw_left` and `right = raw_right`, reported `+753/+870`, but the user physically observed both wheels moving backward. Evidence: `motor-clean-3000-5s-20260713.log` and `motor-clean-3000-5s-pwm-20260713.log`. |
| Final v3, accepted forward | `hardware_duty = clamped`; encoder normalization `left = raw_left`, `right = -raw_right`; helper SHA-256 `d1b953f30cfa5caec9fd9fb3d60c5f57e1b60df544897379056eb4b8ca60209c` | Logical `+3000/+3000` for five seconds produced raw `+739/-877`, normalized `+739/+877`; the helper completed with final PWM zero, and the user physically confirmed normal forward motion. Both Motor channels were observed at duty `17646 ns` over period `58823 ns`, both ESC duties were `0`. Evidence: `motor-clean-forward-3000-5s-20260713.log` and `motor-clean-forward-3000-5s-pwm-20260713.log`. |
| Final v3, direction and stop | Same accepted source/helper | Forward normalized counts were `+710/+826`; reverse normalized counts were `-758/-870`; Motor Apply worst times were `52 us` and `74 us`; both 250-sample ADC windows had `adc_errors=0`; Stop reported `PASS`; final drive duty was `0`. Evidence: `motor-clean-direction-switch-20260713.log`, `motor-clean-direction-switch-pwm-20260713.log`, and `motor-clean-direction-switch-pre-20260713.log`. |

Observation and inference are distinct here: the v2 logs directly record raw and normalized counts, while physical direction comes from the user's live observation. The inference that positive v2 normalized counts meant forward motion was therefore false. The v3 evidence aligns the commanded sign, physical direction, raw Encoder signs, normalized logical signs, reverse behavior, and safe final state.

## I/O-mode timing A/B

This later v4 comparison tests persistent versus open-write-close I/O ownership; it does not retest or alter the v2/v3 polarity conclusion. Both runs used the accepted `hardware_duty = clamped` polarity and logical `+3000/+3000` for five seconds. The v4 helper SHA-256 was `c62e735226c77ee4bb4cf200239bb1674155d1c8d59ef2b15bdf8a45fed1fa19`.

| I/O mode | Direct board observations |
| --- | --- |
| `persistent` | `motor-clean-persistent-3000-5s-20260713.log` records logical counts `+723/+883`, 250 Motor Apply samples with min/max/mean `31/96/31 us`, Encoder read `46 us`, ADC errors `0`, and successful completion with `final_pwm=0`. `motor-clean-persistent-3000-5s-pwm-20260713.log` shows both Motor duties at `17646 ns` over `58823 ns` and both ESC duties `0`. |
| `open-write-close` | `motor-clean-fallback-3000-5s-20260713.log` records logical counts `+703/+871`, 250 Motor Apply samples with min/max/mean `49/117/50 us`, Encoder read `44 us`, ADC errors `0`, and successful completion with `final_pwm=0`. `motor-clean-fallback-3000-5s-pwm-20260713.log` shows the same Motor and ESC duties; the main run output also observed external final zero. |

These observations establish board reachability and successful stop behavior for both I/O modes under this helper workload. The timing samples show lower observed Apply min/max/mean for persistent mode in these runs; they do not by themselves establish a general latency guarantee.

## Injected-write failure safe stop

The v5 helper SHA-256 `5f90d4b285eadd0739cf2b6eb7559391ce3288794b4784fa174f633a622381bb` was executed with `--confirm-failure-safe-stop`. `motor-clean-failure-safe-stop-pre-20260713.log` directly records zero duty on both Motor and both ESC channels before the run.

The failure wrapper injected exactly one `EIO` on the second drive write and passed every subsequent write through. This makes the injected failure observable while leaving the production `FailSafeAfter` path able to issue its normal zeroing writes; it does not simulate a permanently failed output node.

`motor-clean-failure-safe-stop-20260713.log` records:

- initialization succeeded in `open-write-close` mode;
- initial safe zero succeeded;
- `injected_second_drive_write` returned `write_failed` with `errno=5`;
- `injected_failure_final_stop` returned `ok`;
- the helper completed with `FAILURE_SAFE_STOP_COMPLETE final_pwm=0`.

Main-thread post-run `debugfs` command output also observed both Motor and both ESC duties at zero. Because no separate durable post-run log is present, that observation is command-output evidence only; the durable completion claim is the helper's own successful Stop result and `final_pwm=0` line.

## Historical bridge versus accepted contract

The historical bridge is commit `7aa9ca284ae681651de9d9f3c3781b15f5a73ff5` at `/tmp/2k0300-vendor-baseline`; its source is `/tmp/2k0300-vendor-baseline/new/code/platform/true_ls2k0300/motor_bridge.cpp`. It used `vendor_signed_duty = -logical_duty`. The live A/B above supersedes that polarity: on this board, sign inversion is not a valid implementation of logical positive = physical forward.

The following historical behaviors remain intentionally preserved in the accepted `MotorDevice`:

- crossed channel mapping: logical left drives the physical right PWM/GPIO nodes and logical right drives the physical left nodes;
- raw binary `uint16_t` PWM magnitude payload and ASCII `'0'`/`'1'` GPIO payload;
- direction switching sequence PWM `0` -> GPIO -> new PWM;
- logical clamp `[-9000, 9000]` before conversion to magnitude/direction;
- safe stop, expanded in `MotorDevice` to zero both drive and ESC PWM channels on failure or stop.

The accepted implementation also differs in ownership: production defaults to `MotorIoPolicy::kOpenWriteClose`, probes all six nodes, writes drive/ESC zero during initialization, and writes both drive and ESC channels from `MotorDevice::Apply()`. Explicit `MotorIoPolicy::kPreferPersistent` remains opt-in; the v4 board A/B above now covers both it and the default open-write-close mode.

## Superseded baseline-helper attempt

`baseline_motor_board_test_head_7aa9ca2.cpp` built a helper with SHA-256 `b72f8a3b9483cdc76c7fca96881bff7181f9ce30dcafa6a9252836f011f353e3` and intended the sequence `InitializeMotor()` -> `ApplyMotorCommand(3000,3000)` -> five-second hold -> `DisableMotorOutput()`. Its automated board attempt produced no usable motion log and caused `/sys/kernel/debug/pwm` reads to block; it is not direction acceptance evidence. The later controlled v2/v3 board A/B is the applicable polarity evidence.

## Verification boundary

Host Motor, Encoder, and hot-path contract tests and the cross build passed in the main verification flow. This document does not reconstruct or claim additional command transcripts beyond the durable evidence indexed in `EVIDENCE.md`. The remaining full-runtime and independent-review gates are tracked there.
