#ifndef CONTROL_H
#define CONTROL_H

#include "control/pid_controller.h"

// Legacy compatibility wrapper. The pure formula is owned by control/;
// runtime supplies the encoder facts used by the original signature.
float Dis_PID_Calculate(Direction_PID *pid, float expect, float feedback);

#endif
