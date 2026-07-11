#include "platform/device_platform_internal.hpp"
#include "port/run_state_port.hpp"
#include "runtime/runtime_state_internal.hpp"

/* Preserve the baseline init.cpp static construction sequence in one TU. */
#define KEY_1_PATH        ZF_GPIO_KEY_1
#define KEY_2_PATH        ZF_GPIO_KEY_2
#define KEY_3_PATH        ZF_GPIO_KEY_3
#define KEY_4_PATH        ZF_GPIO_KEY_4
#define KEY_5_PATH        ZF_GPIO_KEY_5
#define KEY_6_PATH        ZF_GPIO_KEY_6

#define BEEP_PATH        ZF_GPIO_BEEP
#define ENCODER_DIR_1_PATH           ZF_ENCODER_DIR_1
#define ENCODER_DIR_2_PATH           ZF_ENCODER_DIR_2
#define PWM_1_PATH        ZF_PWM_MOTOR_1
#define DIR_1_PATH        ZF_GPIO_MOTOR_1
#define PWM_2_PATH        ZF_PWM_MOTOR_2
#define DIR_2_PATH        ZF_GPIO_MOTOR_2
#define ESC_PATH        ZF_PWM_ESC_1
struct pwm_info esc_info;
zf_driver_pwm   esc_pwm(ESC_PATH);
// 在设备树中，设置的10000。如果要修改，需要与设备树对应。
#define MOTOR1_PWM_DUTY_MAX    (drv8701e_pwm_1_info.duty_max)
// 在设备树中，设置的10000。如果要修改，需要与设备树对应。
#define MOTOR2_PWM_DUTY_MAX    (drv8701e_pwm_2_info.duty_max)
zf_driver_gpio  key_1(KEY_1_PATH, O_RDWR);//下
zf_driver_gpio  key_2(KEY_2_PATH, O_RDWR);//右
zf_driver_gpio  key_3(KEY_3_PATH, O_RDWR);//上
zf_driver_gpio  key_4(KEY_4_PATH, O_RDWR);//左
zf_driver_gpio  key_5(KEY_5_PATH, O_RDWR);// 右按键中键
zf_driver_gpio  key_6(KEY_6_PATH, O_RDWR);//左按键中键
zf_driver_adc   key_7(ADC_CH3_PATH);//右按键 左键
zf_driver_adc   key_8(ADC_CH1_PATH);//右按键 右键
zf_driver_gpio  beep(BEEP_PATH, O_RDWR);
zf_driver_encoder encoder_dir_1(ENCODER_DIR_1_PATH);
zf_driver_encoder encoder_dir_2(ENCODER_DIR_2_PATH);
struct pwm_info drv8701e_pwm_1_info;
struct pwm_info drv8701e_pwm_2_info;
zf_driver_gpio  drv8701e_dir_1(DIR_1_PATH, O_RDWR);
zf_driver_gpio  drv8701e_dir_2(DIR_2_PATH, O_RDWR);
zf_driver_pwm   drv8701e_pwm_1(PWM_1_PATH);
zf_driver_pwm   drv8701e_pwm_2(PWM_2_PATH);
zf_device_imu imu_dev;
zf_device_ips200 ips200;
int32_t encoder_left,encoder_right;
encoder encoder_L = {0};
encoder encoder_R = {0};
float LPF_Speed = 0.0;
int8_t run_flag;
uint32_t fPS;
int16_t encoder_abs;
int32_t it_time,encode_l_total,encode_r_total;
zf_driver_pit pit_timer;

zf_device_dl1x dl1x_dev;                  // DL1X设备对象
enum dl1x_device_type_enum dl1x_dev_type;  // DL1X设备类型
zf_driver_pit dl1x_pit_timer;             // PIT定时器对象（用于100ms定时采集）
volatile int16 dl1x_distance_raw = 0;      // 定时采集的距离原始数据（volatile防止编译器优化）

namespace primer::runtime {
int8_t RunFlag() { return run_flag; }
void SetRunFlag(int8_t value) { run_flag = value; }
int32_t &CycleCounter() { return it_time; }
float &SpeedFusionWeight() { return LPF_Speed; }
int32_t LeftEncoderTotal() { return encode_l_total; }
int32_t RightEncoderTotal() { return encode_r_total; }
int16_t EncoderDistance() { return encoder_abs; }
RuntimeTelemetry ObserveRuntimeTelemetry()
{
    return {encode_l_total, encode_r_total, encoder_abs};
}
RuntimeCounters AccessRuntimeCounters()
{
    return {encode_l_total, encode_r_total, encoder_abs};
}
}  // namespace primer::runtime

namespace primer::port {
void SetRunMode(int8_t value) { run_flag = value; }
}  // namespace primer::port
