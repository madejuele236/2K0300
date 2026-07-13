# LS2K0300 vendor driver archive

Archive date: 2026-07-13

Source tree:

`true_LS2K0300_Library/Seekfree_LS2K0300_Opensource_Library/libraries`

Repository commit at inventory time:

`7aa9ca284ae681651de9d9f3c3781b15f5a73ff5`

The upstream package does not expose a separate version file in this checkout. The per-file SHA-256 inventory in `metadata/MANIFEST.tsv` is therefore the authoritative snapshot identity.

Archived project-owned vendor surfaces:

- `zf_common`
- `zf_driver`
- `zf_device`
- `zf_components/seekfree_assistant`

Intentionally retained in the live vendor tree:

- `zf_components/tflm`
- `zf_components/ncnn`

The retained components are imported as `vendor_tflm` and `vendor_ncnn`. This archive directory is not present in `new/user/CMakeLists.txt`, include paths, target sources, or link inputs.

Replacement owners:

| Vendor surface | Canonical owner |
| --- | --- |
| PIT/timer | `new/code/platform/true_ls2k0300/TimerDevice` |
| ADC | `new/code/platform/true_ls2k0300/AdcDevice` |
| Encoder | `new/code/platform/true_ls2k0300/EncoderPair` |
| IMU core/models | `new/code/platform/true_ls2k0300/ImuDevice` |
| UVC | `new/code/platform/true_ls2k0300/CameraDevice` |
| PWM/GPIO/file motor support | `new/code/platform/true_ls2k0300/MotorDevice` plus `new/code/platform/linux` |
| Assistant protocol/glue/TCP/UDP | `ls2k_assistant_protocol`, `ls2k_assistant_transport`, and their sources under `new/code/transport` plus `new/code/platform/true_ls2k0300/assistant_bridge.*` |
| Unused display/common helpers | No main-program runtime owner; removed from the main build after call/object/symbol audit |

Verification evidence and commands are indexed in `/home/madejuele/projects/2K0300/plan-20260713.md`.
