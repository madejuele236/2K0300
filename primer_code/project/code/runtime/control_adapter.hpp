#pragma once

#include "control/pid_controller.h"

float Dis_PID_Calculate(Direction_PID *pid, float expect, float feedback);
