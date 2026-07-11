#include "runtime/estimation_adapter.hpp"

#include "estimation/imu_estimator_internal.hpp"
#include "platform/device_platform_internal.hpp"
#include "vision/vision_facts.hpp"
#include "vision/internal/legacy_vision_state.hpp"
#include "zf_driver_delay.hpp"

namespace {
void DelayMilliseconds(uint32_t milliseconds)
{
    system_delay_ms(milliseconds);
}
}

void imu660ra_get_acc(void)
{
    imu660ra_acc_x = imu_dev.get_acc_x();//
    imu660ra_acc_y = imu_dev.get_acc_y();//
    imu660ra_acc_z = -imu_dev.get_acc_z();//

    // imu660ra_acc_x = imu_dev.get_acc_x()*2;//
    // imu660ra_acc_y = imu_dev.get_acc_y()*2;//
    // imu660ra_acc_z = imu_dev.get_acc_z()*2;//
}

void imu660ra_get_gyro(void)
{
    imu660ra_gyro_x = imu_dev.get_gyro_x();//
    imu660ra_gyro_y = imu_dev.get_gyro_y();//
    imu660ra_gyro_z = -imu_dev.get_gyro_z();//

    // imu660ra_gyro_x = imu_dev.get_gyro_x()*2;//
    // imu660ra_gyro_y = imu_dev.get_gyro_y()*2;//
    // imu660ra_gyro_z = imu_dev.get_gyro_z()*2;//
}

void gyroOffset_init(void)
{
    GyroOffset_InitCore(imu660ra_get_acc, imu660ra_get_gyro,
                        DelayMilliseconds);
}

void ICM_getEulerianAngles(void)
{
    ICM_GetEulerianAnglesCore(imu660ra_get_gyro, imu660ra_get_acc);
}

void huandao_yaw_correct(void)
{
    HuandaoYawCorrectCore(Yaw_Huandao, icm_data.yaw,
                          &yaw_correct, &Yaw_Huandao_err);
}
