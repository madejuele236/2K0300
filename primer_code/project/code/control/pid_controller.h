#ifndef PRIMER_CODE_CONTROL_PID_CONTROLLER_H_
#define PRIMER_CODE_CONTROL_PID_CONTROLLER_H_

#include <cstdint>

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

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
namespace primer::control {
Direction_PID &ImageController();
Direction_PID &DistanceController();
PID &LeftVelocityController();
PID &RightVelocityController();
float MasterSpeed();
float ImageOutput();
float &MutableImageOutput();
float CurrentSpeed();
float DistanceOutput();
}  // namespace primer::control
float Pos_Cal(PID*pid_t,float expect,float feedback);
float Inc_Cal(PID *pid_t, float expect, float feedback);
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut,float K,float q,float f);
float Speed_PID_Cal(PID *PID,int16_t Target, int16_t feedback);
void Speed_PID_Init(PID *pid, float kf,float kp, float ki, float kd,float maxI, float maxOut, float max_error);
void Direction_PID_Init(void);
float Image_PID_Calculate(Direction_PID *pid, float expect, float feedback);
float speed_difference_Calculate(Direction_PID *pid, float expect, float feedback);




#endif
