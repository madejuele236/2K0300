# Active code boundaries

The active source tree is layered by ownership.  These rules are mechanically
checked by `primer_code/verification`.

- A layer includes its own headers and `port/` contracts.
- A non-runtime owner never includes another owner, including that owner's
  public API.  Only runtime composition/adapters know multiple owners, and
  `port/` never depends back on an owner.
- A layer never includes another layer's private/internal header.  Only
  `runtime/hardware_composition.cpp` and `runtime/service_composition.cpp` may
  wire another owner's private construction state without publishing it.
- `runtime/` composes the original startup, foreground, 5 ms control, and
  cross-owner object-construction flows.
- Public owner headers expose query/command/service APIs and contain no legacy
  mutable `extern` declarations or generic legacy macros.
- Layer-private legacy bindings may retain old identifier spellings only by
  delegating to public owner or `port/` APIs; they are not reusable application
  umbrellas.  Each active vision translation unit has exactly one private,
  one-to-one dependency header.
- A legacy binding is either a constant-initialized empty proxy or an
  expression-time private macro included after all library headers.  It may not
  cache another owner's view/reference before `main`.
- Vision control uses one small const `ControlLiveView`; presentation uses
  fact-granular const-reference queries.  Both preserve the baseline's live,
  unsynchronised read timing while mutation stays behind explicit commands.
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
