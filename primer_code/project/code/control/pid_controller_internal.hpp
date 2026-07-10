#ifndef PRIMER_CODE_CONTROL_PID_CONTROLLER_INTERNAL_HPP_
#define PRIMER_CODE_CONTROL_PID_CONTROLLER_INTERNAL_HPP_

#include "control/pid_controller.h"

float Dis_PID_CalculateCore(Direction_PID *pid, float expect, float feedback,
                            float encoder_left_d_speed,
                            float encoder_right_d_speed);

#endif
