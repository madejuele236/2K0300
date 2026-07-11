#pragma once

#include "port/vision_geometry.hpp"

// Private spellings retained only for token-identical primer algorithms.
#ifndef MIN
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#endif
#ifndef MAX
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#endif

#define LCDH_0 ::primer::port::vision::kSourceHeight
#define LCDW_0 ::primer::port::vision::kSourceWidth
#define LCDH_1 ::primer::port::vision::kProcessedHeight
#define LCDW_1 ::primer::port::vision::kProcessedWidth
#define FPS ::primer::port::vision::kExpectedFramesPerSecond
#define white ::primer::port::vision::kWhite
#define black ::primer::port::vision::kBlack
