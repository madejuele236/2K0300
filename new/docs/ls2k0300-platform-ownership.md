# LS2K0300 platform ownership

`new/code` is the only maintained implementation of LS2K0300 platform drivers used by the `new` application.

## Build targets

| Target | Owner |
| --- | --- |
| `ls2k_linux_io` | `new/code/platform/linux` fd and syscall mechanisms |
| `ls2k_platform_timer` | `TimerDevice` |
| `ls2k_platform_adc` | `AdcDevice` and the power adapter |
| `ls2k_platform_encoder` | `EncoderPair` and the encoder adapter |
| `ls2k_platform_imu` | `ImuDevice` and the IMU adapter |
| `ls2k_platform_camera` | `CameraDevice`, camera adapter, and frame source |
| `ls2k_platform_motor` | `MotorDevice` and actuator adapter |
| `ls2k_assistant_protocol` | Byte-level Assistant framing and decoding |
| `ls2k_assistant_transport` | Assistant link and non-blocking TCP state machine |
| `vendor_tflm` | Unmodified imported TFLM library |
| `vendor_ncnn` | Unmodified imported NCNN library |

The main executable links these targets explicitly. It does not expose or compile `zf_common`, `zf_driver`, `zf_device`, or vendor Assistant headers and translation units.

## Hardware contracts

- Encoder reads preserve the board driver's non-POSIX contract: a zero return may still carry a valid 32-bit driver write whose low signed 16 bits are the count. The board's non-seekable character device defaults to open/read/close; `EncoderIoPolicy::kPreferPersistent` remains explicit for independently validated seekable implementations. Initialization fixes the selected mode, and runtime failures do not silently switch modes.
- IMU discovery, model/channel ownership, axis order, and whole-sample validity are owned by `ImuDevice`.
- Camera frames remain valid only until the next capture, stop, or destruction. `Stop()` performs STREAMOFF, unmap, buffer release, and fd close, and the same object can restart.
- Motor logical left uses physical PWM/GPIO channel 2 and logical right uses channel 1. Logical positive duty maps directly to the hardware-positive direction. The earlier `-logical_duty` conversion is rejected historical behavior: an equal-condition board A/B moved both wheels backward with the negative conversion and normally forward without it. Encoder sign remains an independent owner contract (`left=raw_left`, `right=-raw_right`).
- Motor production defaults to open/write/close. The current character driver exposes no safe zero-output signal that proves simultaneous persistent PWM fd isolation, and board tests showed persistent multi-channel output was not reliable. `MotorIoPolicy::kPreferPersistent` remains explicit for environments that independently prove that contract.
- Motor initialization first establishes PWM=0, then writes both direction GPIOs to the software initial state. This prevents a new process from inheriting stale reverse GPIO levels from the preceding process.

## Archive boundary

Replaced project vendor sources are preserved under:

`archive/true_ls2k0300_vendor_drivers_20260713`

The archive includes a SHA-256 manifest and original-path mapping. It is not an include, compile, or link input. TFLM and NCNN remain under the live vendor component tree because they are imported third-party dependencies.

## Verification entry points

Host contract runners are under `new/verification/tests/run_*device_test.sh` plus the Assistant protocol/bridge runners. Cross-build and board helper targets are defined by `new/user/CMakeLists.txt` when `LS2K_BUILD_VERIFICATION_HELPERS=ON`.

The authoritative completion and board-evidence index is the final section of `/home/madejuele/projects/2K0300/plan-20260713.md`.
