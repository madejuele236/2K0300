# LS2K0300 vendor joint-refactor evidence

Evidence date: 2026-07-13

This file indexes observed build, host, and board results. The acceptance source of truth remains `/home/madejuele/projects/2K0300/plan-20260713.md`. A result recorded here proves only the command and conditions named beside it.

## Source and build identities

- Baseline source: commit `7aa9ca284ae681651de9d9f3c3781b15f5a73ff5` in detached worktree `/tmp/2k0300-vendor-baseline`.
- Baseline binary SHA-256: `9e6a0e15fdf85b66b2748263054b955193f57d96b07753e7bb9ab2ac6642e05f`.
- Baseline runtime log: `runtime-smoke-baseline-head-7aa9ca2.log`, SHA-256 `e738d572c54ddc151343c3ce081fa945973904483758dedb7b3c4584be23afc3`.
- Final evidence build directory: `/home/madejuele/projects/2K0300/new/out-final-evidence`.
- Final post-Motor/Encoder board binary local/remote SHA-256: `d97f0dbc81095ae1a68de2b1d2854b4fb316f0538eeb732e35eb0c3c4f75c13d`.
- Final whole-runtime integration log: `runtime-smoke-post-motor-20260713.log`, SHA-256 `c08bf3b993934529de94995bdae78516be4fa76b97596badc1edf4622f4c2ea7`.
- Final whole-runtime performance log: `runtime-smoke-post-motor-perf-20260713.log`, SHA-256 `3265c0fc3ff019e8adaa4c505b055e64b64e8f00760fc358663e74382abe2deb`.
- Durable final post-state: `runtime-smoke-post-motor-post-state-20260713.log`; it records the same remote binary hash, no detected runtime process before capture, and zero duty on both Motor and both ESC channels.
- Earlier cold-boot integration log: `/home/madejuele/projects/2K0300/new/verification/runtime-smoke.log`, run ID `coldboot-before-final-encoder-motor-fixes-20260713T065241Z`, SHA-256 `703ac1775ddecf52886e58e4a04218607152983eda0367cdd4e78201ae1dfc84`. It predates the later Encoder policy and Motor polarity work and is historical rather than final-runtime evidence.
- `runtime-smoke-final.log` in this directory also predates the final post-Motor/Encoder runs and is retained only as historical evidence.

## Final targets and owners

| Target | Owner surface |
| --- | --- |
| `ls2k_linux_io` | `new/code/platform/linux/UniqueFd`, `IoResult`, `SyscallApi` |
| `ls2k_platform_timer` | `new/code/platform/true_ls2k0300/TimerDevice` |
| `ls2k_platform_adc` | `new/code/platform/true_ls2k0300/AdcDevice` and power adapter |
| `ls2k_platform_encoder` | `new/code/platform/true_ls2k0300/EncoderPair` and encoder adapter |
| `ls2k_platform_imu` | `new/code/platform/true_ls2k0300/ImuDevice` and IMU adapter |
| `ls2k_platform_camera` | `new/code/platform/true_ls2k0300/CameraDevice` and camera owners |
| `ls2k_platform_motor` | `new/code/platform/true_ls2k0300/MotorDevice` and actuator adapter |
| `ls2k_assistant_protocol` | `new/code/transport/assistant_protocol.*` |
| `ls2k_assistant_transport` | existing assistant link/service state machine and platform transport |
| `vendor_tflm`, `vendor_ncnn` | imported third-party targets retained in upstream layout |

The target definitions and explicit source lists are in `new/user/CMakeLists.txt`. The ownership boundary is documented in `new/docs/ls2k0300-platform-ownership.md`.

## Host contract tests

The following commands were rerun from repository root on 2026-07-13. Each exited `0`.

| Command | Observed result |
| --- | --- |
| `rtk new/verification/tests/run_linux_io_test.sh` | `linux_io_test passed` |
| `rtk new/verification/tests/run_timer_device_test.sh` | `timer_device_test passed` |
| `rtk new/verification/tests/run_adc_device_test.sh` | `adc_device_test passed` |
| `rtk new/verification/tests/run_assistant_protocol_contract_test.sh` | `assistant_protocol_contract_test passed` |
| `rtk new/verification/tests/run_assistant_bridge_contract_test.sh` | `assistant_bridge_contract_test passed` |
| `rtk new/verification/tests/run_encoder_pair_test.sh` | `encoder_pair_test passed` |
| `rtk new/verification/tests/run_imu_device_test.sh` | `imu_device_test passed` |
| `rtk new/verification/tests/run_camera_device_test.sh` | exit `0` |
| `rtk new/verification/tests/run_motor_device_test.sh` | exit `0` |
| `rtk new/verification/tests/run_power_adapter_threshold_test.sh` | `power_adapter_threshold_test passed` |
| `rtk new/verification/tests/run_camera_frame_source_test.sh` | `camera_frame_source_test passed` |
| `rtk new/verification/tests/run_camera_frame_store_test.sh` | `camera_frame_store_test passed` |
| `rtk bash new/verification/tests/run_assistant_telemetry_selftest.sh` | `assistant_telemetry_selftest passed` |
| `rtk new/verification/tests/run_startup_low_voltage_order_test.sh` | `startup_low_voltage_order_test passed` |
| `rtk new/verification/tests/run_hot_path_allocation_contract_test.sh` | `hot_path_allocation_contract_test passed` |

The allocation contract measures initialized successful calls to `EncoderPair::ReadCounts`, `ImuDevice::ReadRawSample`, and `MotorDevice::Apply`; it records zero global allocations and has compile-time `noexcept` assertions. Timer uses a fixed function-pointer/context callback ABI with `noexcept` callbacks and a preallocated board sample buffer.

The board encoder driver has no `llseek` operation. `EncoderPair` therefore defaults to its observed open/read/close contract; explicit `EncoderIoPolicy::kPreferPersistent` keeps the seekable persistent path and fallback independently testable without imposing a failed probe on this board.

## Cross build

Commands:

```text
rtk cmake -S new/user -B new/out-final-evidence -DLS2K_BUILD_VERIFICATION_HELPERS=ON -DLS2K_TFLM_OP_BENCH=OFF
rtk cmake --build new/out-final-evidence --target new camera_device_board_test timer_device_board_test motor_device_board_test assistant_bridge_board_test -j2
```

Both commands exited `0`. `rtk file` identified the five outputs as 64-bit LoongArch Linux ELF executables.

## Static, object, and symbol audit

The static legacy-call/include audit returned no matches:

```text
rtk rg -n '#include[[:space:]]+[<"](zf_|.*zf_)|seekfree_assistant_interface_init|uvc_camera_init|pit_ms_init|encoder_init|pwm_init|gpio_init' new/code new/user --glob '!out/**' --glob '!out-*/**'
```

Object audit:

```text
rtk /usr/bin/find /tmp/2k0300-vendor-baseline/new/out -type f -name '*.o' | rtk /usr/bin/grep -E '/(zf_driver|zf_device|zf_common|seekfree_assistant)/' | rtk /usr/bin/wc -l
# 19
rtk /usr/bin/find new/out-final-evidence -type f -name '*.o' | rtk /usr/bin/grep -E '/(zf_driver|zf_device|zf_common|seekfree_assistant)/' | rtk /usr/bin/wc -l
# 0
```

The baseline `nm -C` audit found seven legacy lines: the `zf_driver_pit.cpp` global initializer, `uvc_camera_init`, three seekfree assistant ABI functions, and two assistant callback globals. The same expression found no line in `new/out-final-evidence/new`:

```text
rtk /opt/ls_2k0300_env/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6/bin/loongarch64-linux-gnu-nm -C new/out-final-evidence/new | rtk rg -i '(^|[^a-z])pit|uvc_camera|seekfree_assistant'
```

## Board evidence

Board endpoint: `root@192.168.137.100`. Uploads used Windows OpenSSH `scp.exe -O`; local and remote SHA-256 values were compared before execution.

| Module/gate | Run ID and evidence | Observed result |
| --- | --- | --- |
| Baseline integration | `baseline-head-7aa9ca2`, `runtime-smoke-baseline-head-7aa9ca2.log` | ADC path bound; IMU660RA persistent stream reached 32 valid reads; encoders initialized; camera produced frames; Assistant connected; timer started/stopped; exit `0`. |
| Earlier integration after cold boot | `coldboot-before-final-encoder-motor-fixes-20260713T065241Z`, `/home/madejuele/projects/2K0300/new/verification/runtime-smoke.log` | At that source state, IMU660RA reached 32 valid reads; Camera and Assistant started; resources released; exit `0`. Later Motor/Encoder changes make this historical rather than final evidence. |
| Final whole-runtime integration | `runtime-smoke-post-motor-20260713.log` | Final binary hash `d97f0dbc81095ae1a68de2b1d2854b4fb316f0538eeb732e35eb0c3c4f75c13d`; actuator-disabled smoke profile; 449 processed-frame lines for the 400-frame target plus controlled-stop drain; IMU continuity `32`; accepted Encoder normalization string; Camera and timer start/stop; controlled return to `DISARMED`; shutdown complete; `remote_runtime_exit=0` and `remote_log_copy_exit=0`. With no host listeners, Assistant recorded eight `Connection refused`/backoff attempts; this run does not claim an Assistant connection. |
| Final whole-runtime performance | `runtime-smoke-post-motor-perf-20260713.log` | 293 processed-frame lines and five one-second performance windows; maxima across the windows were `control.tick=1416 us`, `control.imu_read=95 us`, and `control.encoder_read=147 us`; controlled `remote_runtime_exit=0` and `remote_log_copy_exit=0`. |
| Final runtime post-state | `runtime-smoke-post-motor-post-state-20260713.log` | Same remote binary hash; no runtime process detected before capture; both Motor and both ESC duties `0`. |
| Timer A/B | `timer-ab-20260713.log` | Old worst absolute jitter `111 us`; final cold-boot run `105 us`, 1000 samples, no missed expiration or overrun. |
| Camera restart/resource | `camera-restart-coldboot-20260713.log` | Three start/capture/stop cycles, five frames per cycle, final fd count `6`, virtual pages after stop `322`, exit `0`. |
| Assistant transport | `assistant-board-coldboot-20260713.log` plus host bridge/protocol tests | Separate Assistant board contract exit `0`; host covers invalid numeric DNS, connecting, partial I/O, would-block, peer close, backoff/recovery, reliable and drop modes. This is the Assistant success evidence; the final whole-runtime run had no host listeners and observed refused/backoff state instead. |
| Encoder and IMU timing | `encoder-board-coldboot-20260713.log`, historical cold-boot runtime evidence, and `runtime-smoke-post-motor-perf-20260713.log` | Instrumented Encoder init/read/close and 20 repeated processes passed. In the final five-window performance run, observed maxima were `control.encoder_read=147 us` and `control.imu_read=95 us`; complete sample validity and shutdown are present. |
| Motor polarity/direction/safety | `motor-clean-forward-3000-5s-20260713.log`, `motor-clean-forward-3000-5s-pwm-20260713.log`, `motor-clean-direction-switch-20260713.log`, `motor-clean-direction-switch-pwm-20260713.log`, and `motor-clean-direction-switch-pre-20260713.log` | v3 helper SHA-256 `d1b953f30cfa5caec9fd9fb3d60c5f57e1b60df544897379056eb4b8ca60209c`; user confirmed normal forward motion for logical `+3000/+3000`; raw `+739/-877` normalized to logical `+739/+877`; full direction run observed forward `+710/+826`, reverse `-758/-870`, Apply worst `52/74 us`, ADC errors `0`, Stop `PASS`, and final drive duty `0`. |
| Motor I/O-mode timing A/B | `motor-clean-persistent-3000-5s-20260713.log`, `motor-clean-persistent-3000-5s-pwm-20260713.log`, `motor-clean-fallback-3000-5s-20260713.log`, and `motor-clean-fallback-3000-5s-pwm-20260713.log` | v4 helper SHA-256 `c62e735226c77ee4bb4cf200239bb1674155d1c8d59ef2b15bdf8a45fed1fa19`; persistent and open-write-close both completed 250 Apply samples with positive logical forward counts, ADC errors `0`, and final PWM `0`. Apply min/max/mean was `31/96/31 us` persistent versus `49/117/50 us` open-write-close. |
| Motor injected-write failure safe stop | `motor-clean-failure-safe-stop-pre-20260713.log` and `motor-clean-failure-safe-stop-20260713.log` | v5 helper SHA-256 `5f90d4b285eadd0739cf2b6eb7559391ce3288794b4784fa174f633a622381bb`, executed with `--confirm-failure-safe-stop`; initialization and safe zero succeeded, the injected second drive write failed with `errno=5`, the production failure-stop path succeeded, and the helper reported `FAILURE_SAFE_STOP_COMPLETE final_pwm=0`. |

The failed cold-boot Motor/Encoder investigation is retained in `motor-coldboot-diagnostic-20260713.log`. It is diagnostic evidence, not an acceptance result.
The direct kernel PWM-state observation is retained in `motor-pwm-debugfs-20260713.log`.
The Motor-output/Encoder-direction GPIO propagation is retained in `motor-direction-gpio-trace-20260713.log`.

The first clean A/B run is retained in `motor-clean-3000-5s-20260713.log` and `motor-clean-3000-5s-pwm-20260713.log`. Its helper SHA-256 was `970c905df1a7b8d2d93dc0e9c4adafc734c9371ffb04246706d045def8dec06f`. That source used `hardware_duty = -clamped`; logical `+3000/+3000` for five seconds produced raw counts `-753/+870`, which the then-current encoder normalization (`left = -raw_left`, `right = raw_right`) reported as `+753/+870`. The user observed both wheels moving backward, so interpreting those positive normalized counts as forward was wrong. This run is diagnostic evidence for the rejected polarity/normalization pairing, not forward acceptance.

The accepted source restores encoder normalization to `left = raw_left`, `right = -raw_right` and uses `hardware_duty = clamped`. In `motor-clean-forward-3000-5s-20260713.log`, raw `+739/-877` therefore normalizes to `+739/+877`, and the user physically confirmed normal forward motion. The paired PWM log observed both Motor channels at duty `17646 ns` over period `58823 ns` with both ESC duties `0`; the final pre-run state in `motor-clean-direction-switch-pre-20260713.log` shows both Motor duties at `0`.

The later v4 board A/B exercised both Motor I/O modes under the same logical `+3000/+3000` five-second window. `motor-clean-persistent-3000-5s-20260713.log` records `io_mode=persistent`, logical counts `+723/+883`, 250 Apply samples at min/max/mean `31/96/31 us`, Encoder read `46 us`, ADC errors `0`, and `final_pwm=0`. `motor-clean-fallback-3000-5s-20260713.log` records `io_mode=open-write-close`, logical counts `+703/+871`, 250 Apply samples at `49/117/50 us`, Encoder read `44 us`, ADC errors `0`, and `final_pwm=0`. Both paired PWM logs show both Motor duties at `17646/58823 ns` and ESC duty `0`; external final zero was observed in the main run output. This is an I/O-mode timing/behavior A/B, separate from the earlier v2/v3 polarity A/B.

The v5 failure-injection gate used the open-write-close mode. `motor-clean-failure-safe-stop-pre-20260713.log` records zero duty on both Motor and both ESC channels before execution. The helper's wrapper injected exactly one `EIO` on the second drive write, then passed all subsequent writes through so the production `FailSafeAfter` path could perform its zeroing writes. The durable helper log records `initialize status=ok`, `safe_zero status=ok`, `injected_second_drive_write status=write_failed errno=5`, `injected_failure_final_stop status=ok`, and `FAILURE_SAFE_STOP_COMPLETE final_pwm=0`. Main-thread post-run `debugfs` command output also showed both Motor and both ESC duties at zero, but no separate durable post-run log is claimed.

## Archive integrity

- Report: `/home/madejuele/projects/2K0300/archive/true_ls2k0300_vendor_drivers_20260713/ARCHIVE_REPORT.md`.
- Manifest: `/home/madejuele/projects/2K0300/archive/true_ls2k0300_vendor_drivers_20260713/metadata/MANIFEST.tsv`.
- The manifest has one header plus 43 file rows.
- The manifest-to-archive `sha256sum -c` check returned `OK` for all 43 files.
- Live vendor source retains only `zf_components/tflm` and `zf_components/ncnn`; the archive is absent from all final target source/include/link inputs.

## Uncovered checks

- Final commit/worktree identity and the independent no-context reviewer verdict are intentionally absent until all board gates pass and the reviewed scope is stable.
