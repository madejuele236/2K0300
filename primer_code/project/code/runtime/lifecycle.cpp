#include "runtime/lifecycle.h"

#include "control/pid_controller_internal.hpp"
#include "estimation/imu_estimator_internal.hpp"
#include "platform/camera/camera_service_internal.hpp"
#include "platform/device_platform_internal.hpp"
#include "runtime/estimation_adapter.hpp"
#include "runtime/runtime_state_internal.hpp"

#include <csignal>
#include <cstdio>
#include <cstdlib>
/*存放 定时器 电机 Flash 按键 编码器 蜂鸣器等外设的初始化*/
void pid_init(void)
{
    run_flag=0;
    //720 duty 小车刚好能在地面上静止
    Direction_PID_Init();//转向环 正为左转 负为右转

    //Speed_PID_Init(&Velocity,3,20,0.1,6,5000,10000,30);//位置式速度环

}
void sigint_handler(int signum)
{
    drv8701e_pwm_1.set_duty(0);
    drv8701e_pwm_2.set_duty(0);
    printf("收到Ctrl+C,程序即将退出\n");
    exit(0);
}

void cleanup()
{
    pit_timer.stop();
    esc_pwm.set_duty(500);
    cam.stop_collect();
    // 关闭电机
    drv8701e_pwm_1.set_duty(0);
    drv8701e_pwm_2.set_duty(0);
    printf("程序异常退出，执行清理操作\n");
}


void init()
{   atexit(cleanup);//清理函数注册
    signal(SIGINT, sigint_handler);// 注册SIGINT信号的处理函数
    drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);//电机初始化 获取设备信息
    drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);//电机初始化 获取设备信息
    imu_dev.init();// IMU传感器初始化，自动检测挂载的IMU型号并完成底层配置
    gyroOffset_init();
    esc_pwm.get_dev_info(&esc_info);
    dl1x_dev.init();
    ips200.init(FB_PATH);//屏幕初始化
    encode_l_total = 0;
    encode_r_total = 0;
    encoder_abs = 0;
}

void Encoder_update()
{

    //转速 = (脉冲数 / 编码器线数) / 采样时间
    encoder_L.count_now = encoder_dir_1.get_count()*2.0;
    encoder_R.count_now = -encoder_dir_2.get_count()*2.0;
    encoder_dir_1.clear_count();
    encoder_dir_2.clear_count();
    // if(Flag.picture == 2 || Flag.picture == 3)
    // {
    // encode_l_total +=  encoder_L.count_now;
    // encode_r_total += encoder_R.count_now;
    // encoder_abs = (encode_l_total + encode_r_total)*0.5;
    // printf("encode_l_total:%d encode_r_total:%d,encoder_abs:%d\n",encode_l_total,encode_r_total,encoder_abs);
    // }
    //printf("encode_l_total:%d encode_r_total:%d,encoder_abs:%d\n",encode_l_total,encode_r_total,encoder_abs);


    LPF_1(20, 5.0e-3,encoder_L.count_now, &encoder_L.speed);
    LPF_1(20, 5.0e-3,encoder_R.count_now, &encoder_R.speed);

        LPF_1(20, 5.0e-3, 0.75*MAX(encoder_L.speed,encoder_R.speed)+0.25*MIN(encoder_L.speed,encoder_R.speed), &Master_Speed);
    // encoder_L.speed=encoder_L.count_now;
    // encoder_R.speed=encoder_R.count_now;
    LPF_1(20, 5.0e-3, 0.5*(encoder_L.count_now + encoder_R.count_now), &Now_Speed);
    // Now_Speed=0.5*(encoder_L.count_now + encoder_R.count_now);
    LPF_1(20, 5.0e-3, (encoder_L.speed - encoder_R.speed), &Tpm_Dis);

        encoder_L.D_speed=encoder_L.speed-encoder_L.last_speed;
        encoder_R.D_speed=encoder_R.speed-encoder_R.last_speed;
        encoder_L.last_speed=encoder_L.speed;
        encoder_R.last_speed=encoder_R.speed;

      G_dis = -icm_data.gyro_z*1.8596*7.63*1.9836/1;//转换为编码器的值./1.5
   //G_dis = -icm_data.gyro_z * 4.88*1.85*1.1;


    Dis_Speed = LPF_Speed* (Tpm_Dis)+(1-LPF_Speed)*G_dis;//差速

    last_G_dis = G_dis;

}
