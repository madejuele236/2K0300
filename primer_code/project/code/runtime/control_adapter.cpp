#include "runtime/control_adapter.hpp"
#include "runtime/internal/control_adapter_bindings.hpp"

float Dis_PID_Calculate(Direction_PID *pid, float expect, float feedback)
{
    return Dis_PID_CalculateCore(pid, expect, feedback,
                                 encoder_L.D_speed, encoder_R.D_speed);
}
