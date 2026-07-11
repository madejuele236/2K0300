#ifndef PRIMER_CODE_PLATFORM_DEVICE_PLATFORM_H_
#define PRIMER_CODE_PLATFORM_DEVICE_PLATFORM_H_

#include <cstdint>
#include "zf_common_typedef.hpp"
#include "zf_driver_adc.hpp"
#include "zf_driver_encoder.hpp"
#include "zf_driver_gpio.hpp"
#include "zf_driver_pit.hpp"
#include "zf_driver_pwm.hpp"
#include "zf_device_dl1x.hpp"
#include "zf_device_imu.hpp"
#include "zf_device_ips200_fb.hpp"

#define SERVER_IP "192.168.3.68"
// 端口号
#define PORT 1347
typedef struct
{
int16_t count_now;
int32_t total_count;       //总脉冲
float speed;         // 速度（脉冲/秒）
float rpm;            // 转速（转/秒）
float last_speed;
float D_speed;
}encoder;
namespace primer::platform {
encoder &LeftEncoder();
encoder &RightEncoder();
zf_device_imu &Imu();
zf_device_ips200 &Display();
zf_driver_gpio &DigitalKey(unsigned index);
zf_driver_adc &AnalogKey(unsigned index);
void SetBeeper(bool enabled);
void SetEscDuty(int duty);
int16 DistanceRaw();
const volatile int16 &DistanceRawSignal();
}  // namespace primer::platform

void set_pwm(int16_t pwm_l,int16_t pwm_r);

#endif
