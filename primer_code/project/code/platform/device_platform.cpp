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
void SetMotorDriverDuty(unsigned index, int duty)
{
    (index == 0 ? drv8701e_pwm_1 : drv8701e_pwm_2).set_duty(duty);
}
void CaptureMotorDriverInfo(unsigned index)
{
    if (index == 0) drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);
    else drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);
}
void StopPeriodicTimer() { pit_timer.stop(); }
void InitializeImu() { imu_dev.init(); }
void CaptureEscInfo() { esc_pwm.get_dev_info(&esc_info); }
void InitializeDistanceSensor() { dl1x_dev.init(); }
void InitializeDisplay(const char *framebuffer_path) { ips200.init(framebuffer_path); }
int32_t ReadEncoderCount(unsigned index)
{
    return (index == 0 ? encoder_dir_1 : encoder_dir_2).get_count();
}
void ClearEncoderCount(unsigned index)
{
    (index == 0 ? encoder_dir_1 : encoder_dir_2).clear_count();
}
int16 ReadDistanceSensor() { return dl1x_dev.get_distance(); }
void StartPeriodicTimer(uint32_t milliseconds, void (*callback)(void))
{
    pit_timer.init_ms(milliseconds, callback);
}
int16 DistanceRaw() { return dl1x_distance_raw; }
const volatile int16 &DistanceRawSignal() { return dl1x_distance_raw; }
volatile int16 &MutableDistanceRawSignal() { return dl1x_distance_raw; }
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
