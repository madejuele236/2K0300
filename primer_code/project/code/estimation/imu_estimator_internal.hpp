#ifndef PRIMER_CODE_ESTIMATION_IMU_ESTIMATOR_INTERNAL_HPP_
#define PRIMER_CODE_ESTIMATION_IMU_ESTIMATOR_INTERNAL_HPP_

#include "estimation/imu_estimator.h"

extern quater_param_t Q_info;
extern uint8 GyroOffset_init;
extern int16 imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z;

using estimation_sample_callback_t = void (*)(void);
using estimation_delay_callback_t = void (*)(uint32_t milliseconds);

void GyroOffset_InitCore(estimation_sample_callback_t read_acc,
                         estimation_sample_callback_t read_gyro,
                         estimation_delay_callback_t delay_ms);
void ICM_GetEulerianAnglesCore(estimation_sample_callback_t read_gyro,
                               estimation_sample_callback_t read_acc);
void HuandaoYawCorrectCore(float yaw_huandao, float current_yaw,
                           float *yaw_correct_value,
                           float *yaw_huandao_error);

#endif
