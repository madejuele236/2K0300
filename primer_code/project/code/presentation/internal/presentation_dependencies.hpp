#pragma once

// Private compatibility boundary for the token-identical operator UI.
#include "zf_driver_delay.hpp"
#include "presentation/internal/legacy_vision_constants.hpp"
#include "presentation/show.hpp"

#include <cstdio>
#include <cstring>
#include <opencv2/opencv.hpp>

// Legacy expression macros must be introduced only after every library header
// has been parsed; names such as `distance` are intentionally TU-local.
#include "presentation/internal/legacy_presentation_bindings.hpp"
