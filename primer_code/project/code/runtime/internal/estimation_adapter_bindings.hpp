#pragma once

#include "estimation/imu_estimator.h"
#include "platform/device_platform.h"
#include "vision/vision_queries.hpp"

namespace {

const primer::estimation::RawImuState raw_imu =
    primer::estimation::AccessRawImuState();
const primer::vision::RoundaboutYawState roundabout_yaw =
    primer::vision::AccessRoundaboutYawState();

static auto &imu660ra_acc_x = raw_imu.acc_x;
static auto &imu660ra_acc_y = raw_imu.acc_y;
static auto &imu660ra_acc_z = raw_imu.acc_z;
static auto &imu660ra_gyro_x = raw_imu.gyro_x;
static auto &imu660ra_gyro_y = raw_imu.gyro_y;
static auto &imu660ra_gyro_z = raw_imu.gyro_z;
static auto &imu_dev = primer::platform::Imu();
static const auto &icm_data = primer::estimation::CurrentImuEstimate();
static const auto &Yaw_Huandao = roundabout_yaw.initial_yaw;
static auto &yaw_correct = roundabout_yaw.corrected_yaw;
static auto &Yaw_Huandao_err = roundabout_yaw.yaw_error;

}  // namespace
