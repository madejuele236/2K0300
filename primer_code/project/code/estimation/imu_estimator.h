
#ifndef PRIMER_CODE_ESTIMATION_IMU_ESTIMATOR_H_
#define PRIMER_CODE_ESTIMATION_IMU_ESTIMATOR_H_

#include "zf_common_typedef.hpp"


// extern float angle, angle_dot;
// void Kalman_Filter(float Accel,float Gyro);
// void Yijielvbo(float angle_m, float gyro_m);
// void Erjielvbo(float angle_m, float gyro_m);


typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float pitch;
    float roll;
    float yaw;
} icm_param_t;


typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quater_param_t;



typedef struct {
    float Xdata;
    float Ydata;
    float Zdata;
    float AXdata;
    float AYdata;
    float AZdata;
} gyro_param_t;

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


//extern float icm_ay;
//extern float icm_sy;
namespace primer::estimation {
struct RawImuState {
    int16 &acc_x;
    int16 &acc_y;
    int16 &acc_z;
    int16 &gyro_x;
    int16 &gyro_y;
    int16 &gyro_z;
};

RawImuState AccessRawImuState();
const icm_param_t &CurrentImuEstimate();
void SetRawGyroscope(int16 x, int16 y, int16 z);
}  // namespace primer::estimation
float fast_sqrt(float x);
void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);
void ICM_getValues();
void LPF_1(float hz,float time,float in,float *out);
//void getEulerianAngles(void);
// void imu660_init(void);
#endif
