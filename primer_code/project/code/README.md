# Active code boundaries

The active source tree is layered by ownership.  These rules are mechanically
checked by `primer_code/verification`.

- A layer includes its own headers and `port/` contracts.
- A layer never includes another layer's private/internal header.
- `runtime/` may include public headers from all layers to compose the original
  startup, foreground, and 5 ms control flows.
- `presentation/` and `transport/` are observers/IO owners; they do not derive
  control decisions.
- Root-level `init.h`, `filt.h`, `flash.h`, `control.h`, `image.h`, `show.h`,
  `lq_ncnn.hpp`, `lq_camera_ex.hpp`, and `ww_transmission.h` are compatibility
  facades.  New implementation files must include the narrow owner header.
- `zf_common_headfile.hpp` is a vendor/library umbrella, not an application
  dependency shortcut.  It must not include application headers.
- Algorithms, constants, state codes, update order, and failure behavior are
  inherited from baseline commit `70f3c71ea`; architectural cleanup must not
  silently repair them.
