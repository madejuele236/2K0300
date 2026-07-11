# Active code boundaries

The active source tree is layered by ownership.  These rules are mechanically
checked by `primer_code/verification`.

- A layer includes its own headers and `port/` contracts.
- A layer never includes another layer's private/internal header.  The sole
  exception is a runtime composition translation unit, which may wire private
  legacy symbols without publishing them.
- `runtime/` composes the original startup, foreground, 5 ms control, and
  cross-owner object-construction flows.
- Public owner headers expose query/command/service APIs and contain no legacy
  mutable `extern` declarations.
- Layer-private legacy bindings may retain old identifier spellings only by
  delegating to public owner APIs; they are not reusable application umbrellas.
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
