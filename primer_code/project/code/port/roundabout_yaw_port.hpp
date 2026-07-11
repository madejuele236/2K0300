#pragma once

namespace primer::port {

float CurrentYaw();

// Cross-owner command implemented by the runtime composition boundary.
// Vision requests the operation without learning where IMU and yaw state live.
void CorrectRoundaboutYaw();

}  // namespace primer::port
