#ifndef INIT_H
#define INIT_H

// Legacy compile-time compatibility only; active platform contracts do not
// publish generic network macros.
#define SERVER_IP "192.168.3.68"
#define PORT 1347

#include "platform/device_platform.h"
#include "runtime/lifecycle.h"
#include "runtime/runtime_state.hpp"

// Historically reachable through init.h even though estimation owns it.
#include "estimation/imu_estimator.h"

#endif
