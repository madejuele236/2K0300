# BEV metric consumer audit

Calibration authority:

- `BEV_PROJECTOR.TARGET_FORWARD_*` and `TARGET_LATERAL_*` are the only image-to-BEV metric scale authority.
- Vehicle origin `(forward=0,lateral=0)` is the fixed scale origin.
- Corrected transform: `forward_new=1.304402843698539*forward_old`,
  `lateral_new=1.1028814766321489*lateral_old`.
- The original `SOURCE_ROW/COL_*` points remain unchanged.

## Converted values

| Surface | Ownership | Action |
| --- | --- | --- |
| `new/config/default_params.json:BEV_PROJECTOR.TARGET_*` | image-to-BEV metric producer | converted about `(0,0)` |
| `new/code/port/bev_geometry_types.hpp:BEVProjectorCalibration` | built-in fallback for the same producer | synchronized with JSON, including current source points and identity |
| `ML.ROI.SEARCH_FORWARD_*` | search window derived from the old marker location | converted by forward scale |
| `ML.ROI.SEARCH_LATERAL_LIMIT_M` | search window derived from the old marker location | converted by lateral scale |
| `ML.ROI.EXPECTED_LONG_EDGE_M/EXPECTED_SHORT_EDGE_M` | physical marker truth | replaced by `0.12/0.05` |
| `ML.ROI.LONG_EDGE_TOLERANCE_M/SHORT_EDGE_TOLERANCE_M` | tolerance calibrated in old BEV axes | converted by matching axis scale |
| `BEV_GEOMETRY.FORWARD_SAMPLE_*` | old fictitious forward coordinates | multiplied by forward scale |
| `BEV_GEOMETRY.SEARCH_LATERAL_LIMIT_M/LATERAL_STEP_M` | old fictitious lateral values | multiplied by lateral scale |
| `BEV_GEOMETRY.NOMINAL_ROAD_HALF_WIDTH_M` | real track width | set directly to measured `0.45/2=0.225m` |
| `BOUNDARY_TRACE_MAX_ADJACENT_DISTANCE_M` | trace progresses between forward rows | multiplied by forward scale; transformed equivalent, not a new measurement |
| CircleV2 forward gates/gaps | old fictitious forward values | multiplied by forward scale |
| CircleV2 lateral offsets/expansion/drift/span limits | old fictitious lateral values | multiplied by lateral scale |
| BEV control gains | inputs changed scale/dimension | lateral divided by lateral scale; heading and curvature use small-slope axis transforms |
| `REFERENCE_TIME_ALIGNMENT.MAX_DELTA_FORWARD/LATERAL_M` | old fictitious axis limits | multiplied by their matching axis scale |
| `ML.ROI.GRID_STEP_M` | old fictitious isotropic step | removed; split into independently converted forward/lateral grid steps |

## Consumers that inherit corrected BEV and must not rescale

| Surface | Why no second conversion is allowed |
| --- | --- |
| `BEVSampleProjectionLut` / sparse row scanner | calls `ProjectVehicleToImage` with already-real policy coordinates and stores BEV points directly |
| boundary jumps, spans and traces | produced from corrected LUT `forward_m/lateral_m` |
| ordinary center/actual-left/actual-right paths | copy corrected boundary facts |
| reference usability/lateral error/tracking geometry | consume corrected path coordinates |
| Cross and CircleV2 observations/composers | consume corrected sparse rows and paths |
| ML rectangle detector and ROI sampler | search and sample directly in corrected BEV |
| assistant/media protocols and live viewer | serialize/project the authoritative config and facts; they do not own scale |

## Real measurements and unresolved physical scales

| Surface | Unit owner |
| --- | --- |
| red marker dimensions | measured truth: long `0.12m`, short `0.05m` |
| wheel center track | measured outer width `0.18m` minus one wheel thickness `0.026m` = `0.154m` |
| track half width | measured track width `0.45m` divided by two = `0.225m` |
| `MOTION_ODOMETRY.ENCODER_TICKS_TO_METER` | still unknown; remains zero and disabled |
| ML maneuver exit/progress tolerances | zero-valued and disabled; no physical value invented |

Before this calibration, every nonzero BEV metric parameter was expressed in
the old fictitious axes. The converted numbers preserve the old operating
regions/thresholds under the new axis scales; the marker dimensions, wheel
center track, and track width/half-width are independently measured real
quantities. Converted
Circle/control/trace values still require later track evidence before being
treated as physically validated policy choices.

Historical capture bundles and logs remain immutable evidence of their original projector identity.
Synthetic unit-test coordinates remain synthetic SI fixtures and are not current-camera calibration data.
