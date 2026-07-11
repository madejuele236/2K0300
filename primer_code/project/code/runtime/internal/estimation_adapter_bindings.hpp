#pragma once

#include "estimation/imu_estimator.h"
#include "platform/device_platform.h"
#include "vision/vision_queries.hpp"

#define PRIMER_RAW_IMU() (primer::estimation::AccessRawImuState())
#define PRIMER_ROUNDABOUT_YAW() (primer::vision::AccessRoundaboutYawState())
#define imu660ra_acc_x (PRIMER_RAW_IMU().acc_x)
#define imu660ra_acc_y (PRIMER_RAW_IMU().acc_y)
#define imu660ra_acc_z (PRIMER_RAW_IMU().acc_z)
#define imu660ra_gyro_x (PRIMER_RAW_IMU().gyro_x)
#define imu660ra_gyro_y (PRIMER_RAW_IMU().gyro_y)
#define imu660ra_gyro_z (PRIMER_RAW_IMU().gyro_z)
#define imu_dev (primer::platform::Imu())
#define icm_data (primer::estimation::CurrentImuEstimate())
#define Yaw_Huandao (PRIMER_ROUNDABOUT_YAW().initial_yaw)
#define yaw_correct (PRIMER_ROUNDABOUT_YAW().corrected_yaw)
#define Yaw_Huandao_err (PRIMER_ROUNDABOUT_YAW().yaw_error)
