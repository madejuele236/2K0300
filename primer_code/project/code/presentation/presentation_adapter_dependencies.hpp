#pragma once

// Presentation is an external adapter: it may observe public runtime facts and
// invoke operator-facing I/O, but no vision stage includes this header.

#include "zf_common_typedef.hpp"
#include "zf_driver_delay.hpp"
#include "zf_driver_gpio.hpp"
#include "zf_driver_adc.hpp"
#include "zf_driver_pwm.hpp"
#include "zf_device_ips200_fb.hpp"

#include "../control.h"
#include "../filt.h"
#include "../flash.h"
#include "../image.h"
#include "../init.h"
#include "show.hpp"

#include <cstdio>
#include <cstring>
#include <opencv2/opencv.hpp>
