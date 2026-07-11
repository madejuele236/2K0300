#ifndef PRIMER_CODE_PLATFORM_DEVICE_PLATFORM_INTERNAL_HPP_
#define PRIMER_CODE_PLATFORM_DEVICE_PLATFORM_INTERNAL_HPP_

#include "platform/device_platform.h"

extern encoder encoder_L;
extern encoder encoder_R;
extern zf_device_imu imu_dev;
extern zf_device_ips200 ips200;
extern zf_driver_gpio key_1, key_2, key_3, key_4, key_5, key_6;
extern zf_driver_adc key_7, key_8;
extern zf_driver_gpio beep;
extern zf_driver_pwm esc_pwm;
extern pwm_info esc_info;
extern zf_device_dl1x dl1x_dev;
extern zf_driver_encoder encoder_dir_1, encoder_dir_2;
extern pwm_info drv8701e_pwm_1_info, drv8701e_pwm_2_info;
extern zf_driver_gpio drv8701e_dir_1, drv8701e_dir_2;
extern zf_driver_pwm drv8701e_pwm_1, drv8701e_pwm_2;
extern int32_t encoder_left, encoder_right;
extern zf_driver_pit pit_timer;
extern dl1x_device_type_enum dl1x_dev_type;
extern zf_driver_pit dl1x_pit_timer;
extern volatile int16 dl1x_distance_raw;

#endif
