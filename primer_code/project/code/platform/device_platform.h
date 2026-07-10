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
extern encoder encoder_L;
extern encoder encoder_R;
extern zf_device_imu imu_dev;
extern zf_device_ips200 ips200;
extern zf_driver_gpio  key_1;
extern zf_driver_gpio  key_2;
extern zf_driver_gpio  key_3;
extern zf_driver_gpio  key_4;
extern zf_driver_gpio  key_5;// 右按键中键
extern zf_driver_gpio  key_6;//左按键中键
extern zf_driver_adc   key_7;//右按键 左键
extern zf_driver_adc   key_8;//右按键 右键
extern zf_driver_gpio  beep;
extern zf_driver_pwm   esc_pwm;
extern pwm_info esc_info;
extern zf_device_dl1x dl1x_dev;
extern zf_driver_encoder encoder_dir_1;
extern zf_driver_encoder encoder_dir_2;
extern pwm_info drv8701e_pwm_1_info;
extern pwm_info drv8701e_pwm_2_info;
extern zf_driver_gpio drv8701e_dir_1;
extern zf_driver_gpio drv8701e_dir_2;
extern zf_driver_pwm drv8701e_pwm_1;
extern zf_driver_pwm drv8701e_pwm_2;
extern int32_t encoder_left, encoder_right;
extern zf_driver_pit pit_timer;
extern dl1x_device_type_enum dl1x_dev_type;
extern zf_driver_pit dl1x_pit_timer;
extern volatile int16 dl1x_distance_raw ;      // 定时采集的距离原始数据（volatile防止编译器优化）

void set_pwm(int16_t pwm_l,int16_t pwm_r);

#endif
