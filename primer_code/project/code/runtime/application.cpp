#include "runtime/application.hpp"

#include "parameters/parameter_store.h"
#include "control/pid_controller.h"
#include "estimation/imu_estimator.h"
#include "platform/device_platform.h"
#include "presentation/show.hpp"
#include "runtime/control_adapter.hpp"
#include "runtime/estimation_adapter.hpp"
#include "runtime/lifecycle.h"
#include "runtime/runtime_state.hpp"
#include "vision/element_runtime.hpp"
#include "vision/pipeline.hpp"
#include "vision/vision_queries.hpp"
#include "zf_common_typedef.hpp"
#include "zf_driver_tcp_client.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iostream>

#include "runtime/internal/legacy_math_macros.hpp"

//中断回调函数
uint8 Tcp_buffer[128];
zf_driver_tcp_client tcp_client_dev;
int frame_count = 0;                                      // 累计帧数
auto start_time = std::chrono::steady_clock::now();       // 统计起始时间
auto last_print_time = start_time;                        // 上一次打印帧率的时间
#define PRINT_INTERVAL 1000           // 帧率打印间隔（毫秒），即每秒打印一次

int32_t frame_cnt = 0;

namespace {

static inline void SamplePeriodicInputs()
{
    ICM_getEulerianAngles();
    if(primer::runtime::CycleCounter()%10==0)
    primer::platform::MutableDistanceRawSignal() = primer::platform::ReadDistanceSensor();
    Encoder_update();
    distance_judge();
}

static inline void ApplyActiveDriveCycle()
{
    // run_flag=1;
    if(primer::runtime::RunFlag()==1)
    {   //33100/2 16550         15*35
        // forward1 = 39-Now_Speed/20;//23
        // forward1 = 34-Master_Speed/40;//23
        primer::vision::SetVisionDynamicForward(41-primer::control::AccessRuntimeControlState().master_speed/20);//23
        primer::vision::SetVisionDynamicForward(30-primer::control::AccessRuntimeControlState().master_speed/60);//23
        // forward1 = 35;//23
    //    forward1 = 32;
        primer::control::AccessRuntimeControlState().speed_goal=10*65;
    //      Image.Kp=3.5*(0.2143*imgInfo.top+0.4286);
    //  if(Image.Kp<=1.3)Image.Kp=1.3;
    //  if(Image.Kp>=3.5)Image.Kp=3.5;
    //   //  speed_goal=10*30;
        // if(Flag.picture==2)speed_goal=50;
    //   if(Flag.picture==2&&real_distance[MAX(R_h_guai.row,L_h_guai.row)]>50)speed_goal=(real_distance[MAX(R_h_guai.row,L_h_guai.row)]-25)*9;
      if(primer::vision::ObserveVisionControlLiveView().elements.picture==2)
      {
             primer::control::AccessRuntimeControlState().speed_goal=100;
    //     if(real_distance[MAX(R_h_guai.row,L_h_guai.row)]<recognize_distance2)
    //     {
    //   Pos_Cal(&picture_distance,recognize_distance2,real_distance[MAX(R_h_guai.row,L_h_guai.row)]);
    //   speed_goal=picture_distance.output;
    //     }

      }


        if(primer::vision::ObserveVisionControlLiveView().elements.ramp==2)
        primer::control::AccessRuntimeControlState().speed_goal=150;




       int speed_add;

    //     //printf("pit!\n");
        // if(imgInfo.top<=12&&Flag.Huandao_L==0&&Flag.Huandao_R==0)
        // {
        //     speed_add=(real_distance[imgInfo.top]-100)*0.6;

        //     if(speed_add>=0)
        //     speed_goal+=speed_add;

        // }



       primer::control::AccessRuntimeControlState().distance_output = Dis_PID_Calculate(&primer::control::AccessRuntimeControlState().distance_controller,-primer::control::AccessRuntimeControlState().image_output,primer::control::AccessRuntimeControlState().distance_speed); //- 100*(Image.Error - Image.Last_Error);//角速度环icm_data.gyro_z * K+ 400*(Image->Error - Image->Last_Error



    //    float K_turn=2;
    //     V_out=Speed_PID_Cal(&Velocity,speed_goal,MAX(encoder_L.speed,encoder_R.speed));

    //    if(Dis_Out>0)
    //  {
    //        PWM_R=V_out-Dis_Out*K_turn;//
    //        PWM_L=V_out+Dis_Out*(2-K_turn);//
    //  }
    //    if(Dis_Out<=0)
    //  {
    //        PWM_R=V_out-Dis_Out*(2-K_turn);//
    //        PWM_L=V_out+Dis_Out*K_turn;//
    //  }

       float K_turn=2.0;//

       if(primer::control::AccessRuntimeControlState().distance_output>0)
     {
           primer::control::AccessRuntimeControlState().right_velocity_target=primer::control::AccessRuntimeControlState().speed_goal-primer::control::AccessRuntimeControlState().distance_output*K_turn;//
           primer::control::AccessRuntimeControlState().left_velocity_target=primer::control::AccessRuntimeControlState().speed_goal+primer::control::AccessRuntimeControlState().distance_output*(2-K_turn);//
     }
       if(primer::control::AccessRuntimeControlState().distance_output<=0)
     {
           primer::control::AccessRuntimeControlState().right_velocity_target=primer::control::AccessRuntimeControlState().speed_goal-primer::control::AccessRuntimeControlState().distance_output*(2-K_turn);//
           primer::control::AccessRuntimeControlState().left_velocity_target=primer::control::AccessRuntimeControlState().speed_goal+primer::control::AccessRuntimeControlState().distance_output*K_turn;//S
     }
        primer::control::AccessRuntimeControlState().right_pwm = Speed_PID_Cal(&primer::control::AccessRuntimeControlState().right_velocity_controller,primer::control::AccessRuntimeControlState().right_velocity_target,primer::platform::RightEncoder().speed)+primer::vision::ObserveVisionControlLiveView().steering_difference_error*0+0* (primer::platform::LeftEncoder().D_speed - primer::platform::RightEncoder().D_speed)+0*primer::control::AccessRuntimeControlState().distance_controller.Integral;//
        primer::control::AccessRuntimeControlState().left_pwm = Speed_PID_Cal(&primer::control::AccessRuntimeControlState().left_velocity_controller,primer::control::AccessRuntimeControlState().left_velocity_target,primer::platform::LeftEncoder().speed)-primer::vision::ObserveVisionControlLiveView().steering_difference_error*0-0* (primer::platform::LeftEncoder().D_speed - primer::platform::RightEncoder().D_speed)-0*primer::control::AccessRuntimeControlState().distance_controller.Integral;//


        set_pwm(primer::control::AccessRuntimeControlState().left_pwm,primer::control::AccessRuntimeControlState().right_pwm);
        //printf("Yaw:%f\n",icm_data.yaw);
    //set_pwm(1000,1000);
    //    sprintf((char*)Tcp_buffer,"%d,%f,%f,%f\n",(encoder_L.count_now - encoder_R.count_now),Tpm_Dis,G_dis,Dis_Speed);
    //   sprintf((char*)Tcp_buffer,"%f,%.1f,%f,%f,%f,%f,%f,%f\n",speed_goal,Now_Speed,Dir_err,-Image_out,Dis_Out,Dis_Speed,Tpm_Dis,G_dis);
    //    sprintf((char*)Tcp_buffer,"%f,%f,%f,%f,%f,%f\n",Image_out,Dis_Out,v_right_target,encoder_R.speed,v_left_target,encoder_L.speed);
    //  tcp_client_dev.send_data(Tcp_buffer,strlen((char*)Tcp_buffer));
    }
}

static inline void ApplyStoppedDriveCycle()
{
    if(primer::runtime::RunFlag()==2)
    {
        primer::control::AccessRuntimeControlState().speed_goal=0;
        set_pwm(0,0);
        primer::platform::SetEscDuty(500);
    }
}

static inline void CompletePeriodicCycle()
{
   primer::runtime::CycleCounter()++;

    if(primer::runtime::CycleCounter()%20==0)
    {
        printf("Yaw:%.3f   dl1x_distance_raw:%d  Flag.picture:%d   Flag.Zebra_cross:%d  jump_point:%d  imgInfo.top:%d    Flag.ramp:%d  real_guai:%.2f Dir_Err:%.1f   Huandao_R:%d    Huandao_L:%d   V_max:%.1f\n"
            ,primer::estimation::CurrentImuEstimate().yaw,primer::platform::MutableDistanceRawSignal(),primer::vision::ObserveVisionControlLiveView().elements.picture,primer::vision::ObserveVisionControlLiveView().elements.Zebra_cross,primer::vision::ObserveVisionControlLiveView().jump_point,primer::vision::ObserveVisionControlLiveView().image.top,primer::vision::ObserveVisionControlLiveView().elements.ramp,primer::vision::ObserveVisionControlLiveView().row_distance[MAX(primer::vision::ObserveVisionControlLiveView().right_high_corner.row,primer::vision::ObserveVisionControlLiveView().left_high_corner.row)],primer::vision::ObserveVisionControlLiveView().direction_error,primer::vision::ObserveVisionControlLiveView().elements.Huandao_R,primer::vision::ObserveVisionControlLiveView().elements.Huandao_L,primer::control::AccessRuntimeControlState().master_speed);
        // printf("PWM_L:%d.PWM_R:%d\n",PWM_L,-PWM_R);
    }
}

static inline void InitializeApplication()
{
    init();
    //zf_model_init();
    pid_init();
    image_init();
//    tcp_client_dev.init(SERVER_IP, PORT);
    //Flag.picture = 2;
    primer::platform::StartPeriodicTimer(5, pit_callback);//5ms定时器初始化

    Param_Init();
}

static inline void UpdateForegroundBeeper()
{
    if(primer::vision::ObserveVisionControlLiveView().elements.picture>0||primer::vision::ObserveVisionControlLiveView().elements.small_rock!=0||primer::vision::ObserveVisionControlLiveView().elements.ramp!=0||primer::vision::ObserveVisionControlLiveView().elements.Zebra_cross==1||primer::vision::ObserveVisionControlLiveView().elements.Zebra_cross==4
        )//||Flag.Huandao_L==1||Flag.Huandao_R==1 ||Flag.Huandao_L==3||Flag.Huandao_R==3 ||Flag.Huandao_L==5||Flag.Huandao_R==5
    {primer::platform::SetBeeper(true);}
    else{primer::platform::SetBeeper(false);}
}

static inline void UpdateForegroundFrameTiming()
{
    frame_count++;
    // 5. 每秒统计一次帧率s
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed = current_time - last_print_time;

    if (elapsed.count() >= PRINT_INTERVAL)
    {
        // 计算实际帧率：帧数 / 耗时（秒）
        double fps = frame_count / (elapsed.count() / 1000.0);
        // printf("top:%d,Dir_err:%.1f\n",imgInfo.top,Dir_err);
        //         printf("Yaw:%f,distance=%f\n",icm_data.yaw,distance);
               printf("encoder_L.count_now:%d,encoder_R.count_now:%d\n",primer::platform::LeftEncoder().count_now,primer::platform::RightEncoder().count_now);


        // 输出帧率信息
        std::cout << "实时帧率：" << fps << " FPS | 累计帧数：" << frame_count << std::endl;

        // 重置统计变量
        frame_count = 0;
        last_print_time = current_time;
    }
}

static inline void RunForegroundPresentation()
{
    if(primer::runtime::RunFlag()==2||primer::runtime::RunFlag()==0)
    {
        oled_show();
        key_scan();
    }
}

static inline void RunForegroundCycle()
{
    UpdateForegroundBeeper();
    UpdateForegroundFrameTiming();
    RunForegroundPresentation();
    ImageDeal();
}

}  // namespace

void pit_callback()
{
    SamplePeriodicInputs();
    ApplyActiveDriveCycle();
    ApplyStoppedDriveCycle();
    CompletePeriodicCycle();

}

namespace primer::runtime {

int RunApplication()
{
    InitializeApplication();
    while(1)
    {
        RunForegroundCycle();
    }
}

}  // namespace primer::runtime
