#pragma once

// Presentation is an external adapter: it may observe public runtime facts and
// invoke operator-facing I/O, but no vision stage includes this header.

#include "zf_driver_delay.hpp"
#include "control/pid_controller.h"
#include "estimation/imu_estimator.h"
#include "parameters/parameter_store.h"
#include "platform/device_platform.h"
#include "runtime/runtime_state.hpp"
#include "vision/vision_facts.hpp"
#include "vision/vision_queries.hpp"
#include "show.hpp"

#include <cstdio>
#include <cstring>
#include <opencv2/opencv.hpp>
