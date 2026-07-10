#include "zf_common_headfile.hpp"
#include <math.h>
int16_t speed_now_left,speed_now_right,speed_now,speed_last_left,speed_last_right,pos_goal,pos_now_left,pos_now_right,PWM_L,PWM_R,expect_angle;
PID Velocity = {0};//速度环
PID Velocity_L = {0};//速度环
PID Velocity_R = {0};//速度环
PID picture_distance = {0};//差速转向环
Direction_PID Image = {0};//图像转向环
Direction_PID Dis_1 = {0};//图像转向环
Direction_PID speed_difference = {0};//图像转向环
float V_out,Image_out,Dis_Out,V_out1;//distance1,distance
float Now_Speed,Master_Speed, Kal_Now_Speed, Dis_Speed, Tpm_Dis, G_dis ,last_G_dis,v_left_target,v_right_target,speed_goal,speed_goal1;//当前速度

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut,float K,float q,float f)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->Max_integral = maxI;//输入上限
    pid->Max_output = maxOut;//输出上限
    pid->K =K;
    pid->kq = q;
    pid->kf = f;
    
    
}
/*位置式速度环PID初始化*/ 
void Speed_PID_Init(PID *pid, float kf,float kp, float ki, float kd, float maxI, float maxOut, float max_error)
{   
    pid->kf = kf;//前馈控制系数
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->Max_integral = maxI;//输入上限
    pid->Max_output = maxOut;//输出上限
    pid->Max_error = max_error;
}

/*******************************************位置式PID*********************************************/
float Pos_Cal(PID*pid_t,float expect,float feedback){
    pid_t->error = expect - feedback;
    
    pid_t->Integral += pid_t->error;

    if(pid_t->Integral > pid_t->Max_integral)
    pid_t->Integral = pid_t->Max_integral;
    if(pid_t->Integral < -pid_t->Max_integral)
    pid_t->Integral = -pid_t->Max_integral;

    pid_t->output = pid_t->kp * pid_t->error + 
    pid_t->ki *pid_t->Integral + 
    pid_t->kd * (pid_t->error - pid_t->last_error);


    if(pid_t->output > pid_t->Max_output)
    pid_t->output = pid_t->Max_output;
    if(pid_t->output < -pid_t->Max_output)
    pid_t->output = -pid_t->Max_output;

    pid_t ->last_error = pid_t->error;

    return pid_t->output;
    
}
/****************************************************增量式PID**********************************************/
float Inc_Cal(PID *pid_t, float expect, float feedback) {
    pid_t->error = expect - feedback;
     //Δu(k) = Kp * [e(k) - e(k-1)] + Ki * e(k) + Kd * [e(k) - 2*e(k-1) + e(k-2)]

     float quad_term = 0.0f;

    if (fabsf(feedback) > 50.0f)
    {
        quad_term = pid_t->kq * pid_t->error * fabsf(pid_t->error);
    }

    float delta_output = pid_t->kp * (pid_t->error - pid_t->last_error) +
                         pid_t->ki * pid_t->error +
                         pid_t->kd * (pid_t->error - 2 * pid_t->last_error + pid_t->last_last_error) + quad_term;

    pid_t->output += delta_output;

    float feedforward = pid_t->kf * expect;

    pid_t->final_output = pid_t->output + feedforward;


    if (pid_t->final_output > pid_t->Max_output)
        pid_t->final_output = pid_t->Max_output;
    else if (pid_t->final_output < -pid_t->Max_output)
        pid_t->final_output = -pid_t->Max_output;
    

    pid_t->last_last_error = pid_t->last_error;
    pid_t->last_error = pid_t->error;    

    return pid_t->final_output;
}
/*******************************************位置式速度环PID*********************************************/
float Speed_PID_Cal(PID *PID,int16_t Target, int16_t feedback){
    PID->error = Target - feedback;

     LPF_1(10, 5.0e-3, (feedback - PID->Last_feedback), &PID->Differential);

    PID->Integral += LIMIT(PID->error, -PID->Max_error, PID->Max_error);
    PID->Integral = LIMIT(PID->Integral, -PID->Max_integral, PID->Max_integral);

    PID->output = PID->kf*Target + PID->kp*PID->error + PID->ki*PID->Differential + PID->kd*(PID->error - PID->last_error);
    PID->output = LIMIT(PID->output, -PID->Max_output, PID->Max_output);//限幅
    
    PID->Last_feedback = feedback;
    PID->last_error=PID->error;
    return PID->output;

}

/*******************************************图像专用PID*************************************************/
float Image_PID_Calculate(Direction_PID *pid, float expect, float feedback)
{
    pid->Error = expect;

    pid->OutPut = pid->Kp*pid->Error + pid->Kp2*pid->Error*ABS(pid->Error) + pid->Kd*(pid->Error-pid->Last_Error)+ pid->Kd_feedback*feedback;
    pid->OutPut = LIMIT(pid->OutPut,-pid->MAX_OutPut,pid->MAX_OutPut);

    pid->Last_Error = pid->Error;

    return pid->OutPut;
}
/*******************************************差速PID*************************************************/
float Dis_PID_Calculate(Direction_PID *pid, float expect, float feedback)
{ 
    //Δu(k) = Kp * [e(k) - e(k-1)] + Ki * e(k) + Kd * [e(k) - 2*e(k-1) + e(k-2)]

    pid->Error = expect - feedback;
    //  pid->Kp =6-0.6*imgInfo.top;//第一个数是最大的阻尼系数，第二个是截至行的比例系数越大阻尼越弱
    //  if(pid->Kp<=1.0)
    // pid->Kp=6.0;//是最小阻尼系数
    float delta_output ;
    pid->Integral=pid->Kd * (encoder_L.D_speed - encoder_R.D_speed);
    delta_output = pid->Kp * (pid->Error - pid->Last_Error) 
                         +pid->Ki * pid->Error +
                         pid->Integral;
                         //

    pid->OutPut += delta_output;
    // LIMIT(pid->OutPut,-pid->MAX_OutPut,pid->MAX_OutPut);

    pid->Last_Last_Error = pid->Last_Error;
    pid->Last_Error = pid->Error;
    pid->expect_last = expect;
    return pid->OutPut;

}


float speed_difference_Calculate(Direction_PID *pid, float expect, float feedback)
{
    pid->Error = expect-feedback;

    pid->OutPut = pid->Kp*pid->Error + pid->Kp2*pid->Error*ABS(pid->Error) + pid->Kd*(pid->Error-pid->Last_Error)+ pid->Kd_feedback*feedback;

    pid->Last_Error = pid->Error;

    return pid->OutPut;
}
/*******************************************方向控制*************************************************/
float Image_E1, Image_E2 = 0;
// void Dir_Control(){

//     Image_PID_Calculate(&Image, Image_E2 , icm_data.gyro_z);//方向环（外环）
//     Inc_Cal(&Dis, Image.OutPut , Dis_Speed);//方向环（内环）

//     // PWM_L = Velocity_L.output + Dis.output;
//     // PWM_R = Velocity_R.output + Dis.output;

//     set_pwm(PWM_L,PWM_R);
// }

    void Direction_PID_Init()   
    {
    // //forward = 28;//打角行



    float K = 2.5;//2.3//12
    Image.Kp = 3.5*K;//  0.8; //250*0.015
    Image.Ki = 0;   
    Image.Kd =2;// Image.Kp *0.3
    Image.Kd_feedback =Image.Kp * 0;
    Image.Kp2 = 0;
    Image.MAX_OutPut = 10000;

    Dis_1.Ki = 5*0.06;
    Dis_1.Kp = 0.70;// ;5*Dis_1.Ki 50


    // Dis_1.Ki = 5;
    // Dis_1.Kp = 60;// ;5*Dis_1.Ki 50

    Dis_1.Kd =0;//Dis_1.Kp*0.001;Dis_1.Kp*0.0005Dis_1.Kp*0.001

    Dis_1.MAX_OutPut = 9999;


    float K_v=1.0;
    Speed_PID_Init(&Velocity,8*K_v,30*K_v,0,9*K_v,1500,10000,2000); //位置式速度环.
    Speed_PID_Init(&Velocity_L,8*K_v,30*K_v,0,9*K_v,1500,10000,2000); //位置式速度环.
    Speed_PID_Init(&Velocity_R,8*K_v,30*K_v,0,9*K_v,1500,10000,2000); //位置式速度环.

    // float K_v=2.0;
    // Speed_PID_Init(&Velocity,5*K_v,1*K_v,0,0*K_v,1500,10000,2000); //位置式速度环.
    // Speed_PID_Init(&Velocity_L,5*K_v,1*K_v,0,0*K_v,1500,10000,2000); //位置式速度环.
    // Speed_PID_Init(&Velocity_R,5*K_v,1*K_v,0,0*K_v,1500,10000,2000); //位置式速度环.

    PID_Init(&picture_distance, -10, 0, -5, 0, 1000,0,0,0);



    // speed_difference.Kp=1.0;
    // speed_difference.Kd=speed_difference.Kp*0.5;
    // float V_i=2;
    // Speed_PID_Init(&Velocity,0,V_i*10,V_i,V_i*0.75 ,10000,10000,2000);
    // Speed_PID_Init(&Velocity_L,0,V_i*10,V_i,V_i*1.5,10000,10000,2000);
    // Speed_PID_Init(&Velocity_R,0,V_i*10,V_i,V_i*1.5,10000,10000,2000);
    // Speed_PID_Init(&Velocity_L,2,35,0.5,18,3000,8000,100);
    // Speed_PID_Init(&Velocity_R,2,35,0.5,18,3000,8000,100);
    
}
