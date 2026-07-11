#ifndef CONTROL_H
#define CONTROL_H

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#define LIMIT(input, low, upper) MIN(MAX(input, low), upper)
#define ABS(a) ((a >= 0) ? (a) : (-a))

#include "control/pid_controller.h"
#include "runtime/control_adapter.hpp"
#include "runtime/runtime_state.hpp"

#endif
