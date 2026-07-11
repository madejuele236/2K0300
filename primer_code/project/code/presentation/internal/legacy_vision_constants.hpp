#pragma once

#include "port/vision_geometry.hpp"

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
