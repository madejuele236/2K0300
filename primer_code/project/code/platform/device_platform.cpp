#include "platform/device_platform.h"
#include "platform/device_platform_internal.hpp"

namespace primer::platform {
encoder &LeftEncoder() { return encoder_L; }
encoder &RightEncoder() { return encoder_R; }
zf_device_imu &Imu() { return imu_dev; }
zf_device_ips200 &Display() { return ips200; }
zf_driver_gpio &DigitalKey(unsigned index)
{
    zf_driver_gpio *keys[] = {&key_1, &key_2, &key_3, &key_4, &key_5, &key_6};
    return *keys[index];
}
zf_driver_adc &AnalogKey(unsigned index)
{
    zf_driver_adc *keys[] = {&key_7, &key_8};
    return *keys[index];
}
void SetBeeper(bool enabled) { beep.set_level(enabled ? 1 : 0); }
void SetEscDuty(int duty) { esc_pwm.set_duty(duty); }
int16 DistanceRaw() { return dl1x_distance_raw; }
const volatile int16 &DistanceRawSignal() { return dl1x_distance_raw; }
}  // namespace primer::platform

//到时候看一下左右电机哪个对应 pwm_1
void set_pwm(int16_t pwm_l,int16_t pwm_r)
{
    if(pwm_l>10000) pwm_l = 10000;
    if(pwm_l<-10000) pwm_l = -10000;
    if(pwm_r>10000) pwm_r = 10000;
    if(pwm_r<-10000) pwm_r = -10000;

    if(pwm_l>0)//正转
    {
        drv8701e_dir_1.set_level(1);
        drv8701e_pwm_1.set_duty(pwm_l);
    }
    else if(pwm_l<=0)
    {
        drv8701e_dir_1.set_level(0);
        drv8701e_pwm_1.set_duty(-pwm_l);
    }
    if(pwm_r>0)
    {
        drv8701e_dir_2.set_level(0);                                      // DIR输出高电平
        drv8701e_pwm_2.set_duty(pwm_r);       // 计算占空比
    }
    else if(pwm_r<=0)
    {
        drv8701e_dir_2.set_level(1);
        drv8701e_pwm_2.set_duty(-pwm_r);
    }
}
