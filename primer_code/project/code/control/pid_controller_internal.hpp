#ifndef PRIMER_CODE_CONTROL_PID_CONTROLLER_INTERNAL_HPP_
#define PRIMER_CODE_CONTROL_PID_CONTROLLER_INTERNAL_HPP_

#include "control/pid_controller.h"

extern Direction_PID Image, Rate,Dis_1,speed_difference;
extern PID Dis,picture_distance,Velocity,Velocity_L,Velocity_R,Groy_turn,Image_turn;
extern int16_t speed_now_left,speed_now_right,speed_now,speed_last_left,speed_last_right,pos_goal,pos_now_left,pos_now_right,PWM_L,PWM_R,expect_angle;
extern float Image_E1, Image_E2, v_left_target,v_right_target,speed_goal,speed_goal1;
extern float Now_Speed, Kal_Now_Speed, Dis_Speed, Tpm_Dis, G_dis,last_G_dis,Master_Speed;
extern float V_out,V_out1,Image_out,Dis_Out,distance2,distance1;

float Dis_PID_CalculateCore(Direction_PID *pid, float expect, float feedback,
                            float encoder_left_d_speed,
                            float encoder_right_d_speed);

#endif
