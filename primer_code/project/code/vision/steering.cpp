#include "internal/vision_stage_contracts.hpp"
#include "../control.h"
#include <cmath>

/***************************************************动态前瞻********************************************************/

int forward,forward1;
float B_near,A_near,B_far,A_far,BA_ratio;
void dynamic_forward(){

regression(imgInfo.top+21,imgInfo.top+1);
B_far = B;
A_far = A;
regression(imgInfo.bottom-21,imgInfo.bottom-1);
B_near = B;
A_near = A;

regression(imgInfo.top + 1,imgInfo.bottom - 1);
//左弯
if(B<0){calculateCurvature((float)Left_Sideline[imgInfo.bottom - 1], (float)imgInfo.bottom - 1, (float)Left_Sideline[imgInfo.bottom - 11], (float)imgInfo.bottom - 11, (float)Left_Sideline[imgInfo.bottom - 21], (float)imgInfo.bottom - 21);}
//右弯
if(B>0){calculateCurvature((float)Right_Sideline[imgInfo.bottom - 1], (float)imgInfo.bottom - 1, (float)Right_Sideline[imgInfo.bottom - 11], (float)imgInfo.bottom - 11, (float)Right_Sideline[imgInfo.bottom - 21], (float)imgInfo.bottom - 21);}

float base_forward = 15;
// 根据曲率和近处斜率增大 forward（弯越急，看得越近）
float k_curv = 500.0f;  // 曲率增益
float k_slope = 80.0f;  // 斜率增益
float forward_candidate = base_forward  + k_curv * curvature  + k_slope * fabsf(B_near);
// 限幅到有效范围
if (forward_candidate < imgInfo.top+1) {
  forward = imgInfo.top+1;
} else if (forward_candidate > imgInfo.bottom-1) {
  forward = imgInfo.bottom-1;
  }
else {
  forward = (int)forward_candidate;
  }
  // 极端急弯强制看最近
if (curvature > 0.12f || fabsf(B_near) > 0.8f) {
  forward = imgInfo.bottom-1;
  }
}

/***************************************************误差计算********************************************************/
float Dir_err = 0, Last_Dir_err = 0,Dir_Err[60],D_ERR;  //图像误差
void Err_Sum(void)
{




forward=forward1;
for(int i=imgInfo.top + 1;i<imgInfo.bottom - 1;i++)
{
    Dir_Err[i]=(float)(LCDW_1/2-(((float)Left_Sideline[i]+(float)Right_Sideline[i])/2.0f));
    //  if(!( Dir_Err[i]<100&& Dir_Err[i]>-100)) Dir_Err[i]=0;
}
    BA_ratio=0;
    for(int i=imgInfo.top + 1;i<imgInfo.bottom - 1;i++)
    {
        Dir_Err[i] = (1-BA_ratio)*Dir_Err[i] + BA_ratio*(B*i + A);
    }

        //  if(Flag.Huandao_R==2||Flag.Huandao_L==2)forward=-1;
        //  if(Flag.Huandao_R==3||Flag.Huandao_L==3)forward=-1;
         if(Flag.Huandao_R==5||Flag.Huandao_L==5)   forward+=1;

     if(forward<imgInfo.top+1)forward=imgInfo.top+1;
     if(forward>50)forward=50;

        Dir_err=Dir_Err[forward];

    // if(Flag.small_rock==1)Dir_err-=10;
    // if(Flag.small_rock==2)Dir_err+=10;

    // if(Flag.Zebra_cross==1)Dir_err=Dir_Err[maxkuan_line]*90/(120-maxkuan_line);
    // if(Flag.Zebra_cross==2)Dir_err=Dir_Err[maxkuan_line]*90/(120-maxkuan_line);
    // if(Flag.Zebra_cross==4)Dir_err=Dir_Err[maxkuan_line]*90/(120-maxkuan_line);

    // if(Flag.picture==3)Dir_err =Dir_Err[maxkuan_line]*90/(120-maxkuan_line)+30;//
        // if(Flag.picture==2)Dir_err =(float)(LCDW_1/2-((float)Left_Sideline[forward]));//
        if(Flag.picture==3)Dir_err =(float)(LCDW_1/2-((float)Left_Sideline[forward]-10));//
        if(Flag.picture==4)Dir_err =(float)(LCDW_1/2-((float)Right_Sideline[forward]+10));//
        // if(Flag.picture==3){Dir_err = Dir_err+30;}//[maxkuan_line]*90/(120-maxkuan_line)+20

// Dir_err =(float)(LCDW_1/2-((float)Left_Sideline[forward]));

    if(!(Dir_err<100&&Dir_err>-100))Dir_err=0;
    if(Dir_err>47)Dir_err=47;
    if(Dir_err<-47)Dir_err=-47;



            if(Flag.small_rock ==1)
    {
        Dir_err-=10;
    }

    if(Flag.small_rock ==2)
    {
        Dir_err+=10;
    }
    // if(Dir_err<-47)Dir_err=-47;
    // if(Dir_err>47)Dir_err=47;


                    D_ERR=Dir_err-Last_Dir_err;

                // if(Flag.picture==2||Flag.picture==3)
                // {
                //                     if((D_ERR)>1)Dir_err=Last_Dir_err+4;
                // if((D_ERR)<-1)Dir_err=Last_Dir_err-4;
                // }
                // else{
                // if(Flag.picture==0)
                // {

                if((D_ERR)>=4)Dir_err=Last_Dir_err+4.0f;
                else if((D_ERR)<-4)Dir_err=Last_Dir_err-4.0f;
                //                 if(Flag.picture==3)
                // {
                // if((D_ERR)>=1)Dir_err=Last_Dir_err+1.0f;
                // else if((D_ERR)<-1)Dir_err=Last_Dir_err-1.0f;
                // }

            //     // }
            //     // }
            Last_Dir_err = Dir_err;

        // Image.Kp = 3.5*3.0;//  0.8; //250*0.015
        // if(Image.Kp>4)Image.Kp=4;

        // if(Now_Speed<200)Image.Kp=3.5*2;
        // if(Now_Speed<150)Image.Kp=3.5*Now_Speed*0.007;
        // if(fabs(Dir_err)<10)Image.Kp=3.5*fabs(Dir_err)*0.3;
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/1)
        // {
        Image.Kp=3.5*Master_Speed/400*3.0;
        // if(Image.Kp<3.5*0.5)Image.Kp=3.5*0.5;
        // Image.Kd=Image.Kp*0.3;
        // Dis_1.Kp=Image.Kp/3.5/3;
        //  Dis_1.Kp = Master_Speed/400*60;//
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/1.5)Image.Kp=3.5*MAX(encoder_L.speed,encoder_R.speed)/400*2.5;
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/2)Image.Kp=3.5*MAX(encoder_L.speed,encoder_R.speed)/400*1.5;
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/3)Image.Kp=3.5*MAX(encoder_L.speed,encoder_R.speed)/400*1;
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/4)Image.Kp=3.5*MAX(encoder_L.speed,encoder_R.speed)/400*0.5;
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/5)Image.Kp=0;
        // if(fabs(Dir_err)<15)Image.Kp*=0.75;
        // else if(fabs(Dir_err)<5)Image.Kp*=0.5;
        // else if(fabs(Dir_err)<5)Image.Kp*=0.25;
        // }
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/2)Image.Kp=3.5*MAX(encoder_L.speed,encoder_R.speed)/speed_goal*3;
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/3)Image.Kp=3.5*MAX(encoder_L.speed,encoder_R.speed)/speed_goal*2;
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/4)Image.Kp=3.5*MAX(encoder_L.speed,encoder_R.speed)/speed_goal*1;


        if(real_distance[imgInfo.top]>150)Image.Kp=3.5*Master_Speed/400*2.75;
        // // if(Now_Speed<0)Image.Kp=0;
        if(real_distance[imgInfo.top]>200)Image.Kp=3.5*Master_Speed/400*2.5;
        if(real_distance[imgInfo.top]>250)Image.Kp=3.5*Master_Speed/400*2.0;
        if(real_distance[imgInfo.top]>300)Image.Kp=3.5*Master_Speed/400*1.5;
        // if(Flag.picture==2)Image.Kp=3.5*1.5;
        // if(Flag.picture==3)Image.Kp=3.5*1.5;
        // if(Flag.picture==4)Image.Kp=3.5*1.5;

        Image_out =Image_PID_Calculate(&Image,Dir_err,0);//-icm_data.gyro_z//Image_E2
}
