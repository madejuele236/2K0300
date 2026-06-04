#ifndef LS2K_PORT_VEHICLE_POSE_DELTA_TYPES_HPP
#define LS2K_PORT_VEHICLE_POSE_DELTA_TYPES_HPP

#include <cstddef>
#include <cstdint>
#include <string>

namespace ls2k::port {

/// 从 start_time_ms 到 end_time_ms 的车体位姿增量。
struct VehiclePoseDelta {
    bool valid = false;
    std::string reason = "not_computed";

    uint64_t start_time_ms = 0;
    uint64_t now_time_ms = 0;
    uint64_t end_time_ms = 0;
    uint64_t measured_until_ms = 0;
    uint64_t predicted_ms = 0;

    double delta_forward_m = 0.0;
    double delta_lateral_m = 0.0;
    double delta_yaw_rad = 0.0;

    double measured_forward_mps = 0.0;
    double measured_yaw_rate_radps = 0.0;
    double predicted_forward_mps = 0.0;
    double predicted_yaw_rate_radps = 0.0;

    std::size_t integrated_motion_segments = 0;
    bool used_encoder_forward = false;
    bool used_imu_yaw = false;
    bool used_wheel_yaw = false;
    bool used_command_prediction = false;
};

}  // namespace ls2k::port

#endif  // LS2K_PORT_VEHICLE_POSE_DELTA_TYPES_HPP
