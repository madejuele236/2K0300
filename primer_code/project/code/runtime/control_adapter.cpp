#include "control.h"
#include "control/pid_controller_internal.hpp"
#include "platform/device_platform.h"

float Dis_PID_Calculate(Direction_PID *pid, float expect, float feedback)
{
    return Dis_PID_CalculateCore(pid, expect, feedback,
                                 encoder_L.D_speed, encoder_R.D_speed);
}
