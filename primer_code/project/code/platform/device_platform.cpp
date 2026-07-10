#include "platform/device_platform.h"

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
