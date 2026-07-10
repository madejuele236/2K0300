#ifndef INIT_H
#define INIT_H

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
extern zf_device_dl1x dl1x_dev;       
extern int8_t run_flag;
extern uint32_t fPS;
extern int32_t it_time, it_time,encode_l_total,encode_r_total;
extern zf_driver_pit pit_timer;
extern int16_t encoder_abs;
extern volatile int16 dl1x_distance_raw ;      // 定时采集的距离原始数据（volatile防止编译器优化）

void init(void);
void Encoder_update(void);
void LPF_1(float hz,float time,float in,float *out);
void set_pwm(int16_t pwm_l,int16_t pwm_r);
void pid_init(void);

#endif