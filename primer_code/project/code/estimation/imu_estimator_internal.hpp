#ifndef PRIMER_CODE_ESTIMATION_IMU_ESTIMATOR_INTERNAL_HPP_
#define PRIMER_CODE_ESTIMATION_IMU_ESTIMATOR_INTERNAL_HPP_

#include "estimation/imu_estimator.h"

extern quater_param_t Q_info;
extern uint8 GyroOffset_init;
extern int16 imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z;
extern int16 imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z;
extern icm_param_t icm_data;
extern gyro_param_t GyroOffset;
extern float param_Kp;
extern float param_Ki;

#endif
