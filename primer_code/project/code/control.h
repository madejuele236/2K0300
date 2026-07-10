 #ifndef CONTROL_H
 #define CONTROL_H

#define LIMIT(input, low, upper)    MIN(MAX(input, low), upper)//限幅
#define ABS(a) ((a >= 0) ? (a) : (-a))
// /************************************PID***********************************/

typedef struct
{
    float kp, ki, kd;
    float kq;//平方项系数
    float kf;//前馈项系数
    float error,last_error,last_last_error;//误差 上次误差 上上次误差
    float Max_error;
    float feedback,Last_feedback;//反馈值 上次反馈值
    float Differential;//微分项
    float Integral, Max_integral;
    float output, Max_output,final_output;
    float K;
}PID;

typedef struct{
        float Kp;//一次项系数
        float Kp2;//二次项系数
        float Ki;
        float Kd;
        float Kd_feedback;      //反馈值微分系数

        float Error;            //误差
        float Integral;         //误差积分
        float Max_Error;            //误差
        float Last_Error;       //上次误差
        float Last_Last_Error;

        float OutPut;
        float MAX_Integral;
        float MAX_OutPut;
        float expect_last;

}Direction_PID;
extern Direction_PID Image, Rate,Dis_1,speed_difference;
extern PID Dis,picture_distance;//差速转向环
extern PID Velocity;//速度环
extern PID Velocity_L;//速度环
extern PID Velocity_R;//速度环
extern PID Groy_turn;
extern PID Image_turn;
extern int16_t speed_now_left,speed_now_right,speed_now,speed_last_left,speed_last_right,pos_goal,pos_now_left,pos_now_right,PWM_L,PWM_R,expect_angle;
extern int8_t run_flag;
extern float Image_E1, Image_E2, v_left_target,v_right_target,speed_goal,speed_goal1;
extern float Now_Speed, Kal_Now_Speed, Dis_Speed, Tpm_Dis, G_dis,last_G_dis,Master_Speed;//当前速度
extern float V_out,V_out1,Image_out,Dis_Out,distance2,distance1;
float Pos_Cal(PID*pid_t,float expect,float feedback);
float Inc_Cal(PID *pid_t, float expect, float feedback);
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut,float K,float q,float f);
float Speed_PID_Cal(PID *PID,int16_t Target, int16_t feedback);
void Speed_PID_Init(PID *pid, float kf,float kp, float ki, float kd,float maxI, float maxOut, float max_error);
void Direction_PID_Init(void);
float Image_PID_Calculate(Direction_PID *pid, float expect, float feedback);
float Dis_PID_Calculate(Direction_PID *pid, float expect, float feedback);
float speed_difference_Calculate(Direction_PID *pid, float expect, float feedback);




 #endif