#include <cmath>
#include "../internal/dependencies/element_scenes_dependencies.hpp"

uint16_t jump_point,finish_flag;
float distance_cross,distance,distance_picture;


void distance_judge(void)//1m=66
{
           if(Flag.Huandao_L==1||Flag.Huandao_R==1||Flag.Huandao_L==2||Flag.Huandao_R==2||Flag.Huandao_R==6||Flag.Huandao_L==6
                  ||Flag.small_rock||Flag.Zebra_cross==1||Flag.Zebra_cross==2||Flag.Zebra_cross==4||Flag.ramp!=0)//||Flag.Huandao_R==3||Flag.Huandao_L==3
    distance+=(float)(primer::port::VisionLeftEncoder().count_now+primer::port::VisionRightEncoder().count_now)/2/350;
    if(Flag.picture==3||Flag.picture==4||Flag.picture==5||Flag.picture==6)
    {
    distance_picture+=(float)(primer::port::VisionLeftEncoder().count_now+primer::port::VisionRightEncoder().count_now)/2/350;
    }
}


 void zebra_corssing(void)
 {


    if(Flag.Zebra_cross==0)
    {
    jump_point=0;
    for(int i = 25;i < 44; i++)
    {
        for(int j = 30; j < 64; j++)
        {
            if(Image_Use[i][j] == 255 && Image_Use[i][j+1] == 0)
            {
                jump_point ++;
            }
        }
    }

    if(jump_point>25)//&&top_white_num>0&&fabs(real_distance[imgInfo.top]-real_distance[MAX(R_h_guai.row,L_h_guai.row)])<20&&imgInfo.top>30
    {
                Flag.Zebra_cross=1;
            // }

    }
    }

    if(Flag.Zebra_cross==1)
    {

        if(distance>15)
        {
            Flag.Zebra_cross=2;
            distance=0;
        }
    }

    if(Flag.Zebra_cross==2)
    {
        if(distance>30)
        {
            Flag.Zebra_cross=3;
            distance=0;
        }
    }

    if(Flag.Zebra_cross==3)
    {
        jump_point=0;
        for(int i = 25;i < 44; i++)
        {
            for(int j = 30; j < 64; j++)
            {
                if(Image_Use[i][j] == 255 && Image_Use[i][j+1] == 0)
                {
                    jump_point ++;
                }
            }
        }


        if(jump_point>25&&Flag.ramp==0&&Flag.picture==0)//&&top_white_num>0&&fabs(real_distance[imgInfo.top]-real_distance[MAX(R_h_guai.row,L_h_guai.row)])<20&&imgInfo.top>30
        {
            Flag.Zebra_cross=4;
        }
    }

    if(Flag.Zebra_cross==4)
    {
        if(distance>10)
        {
            Flag.Zebra_cross=5;
            primer::port::SetVisionRunMode(2);
            primer::port::SetVisionEscDuty(500);
        }

    }

 }






uint16_t small_rock_r_num,small_rock_l_num;
namespace {

void ComputeSmallRockSearchCorridor()
{
    float K1 = xielv_sideline(55, Left_Sideline[55], 45, Left_Sideline[45], 'k');
    float B1 = xielv_sideline(55, Left_Sideline[55], 45, Left_Sideline[45], 'b');
    float K2 = xielv_sideline(55, Right_Sideline[55], 45, Right_Sideline[45], 'k');
    float B2 = xielv_sideline(55, Right_Sideline[55], 45, Right_Sideline[45], 'b');
    red_left = K1 * MAX(R_h_guai.row, L_h_guai.row) + B1;
    red_right = K2 * MAX(R_h_guai.row, L_h_guai.row) + B2;
}

void DetectSmallRockCandidates()
{
    if(R_h_guai.flag==1&&L_h_guai.flag==1)
    {
        if(MAX(R_h_guai.row,L_h_guai.row)>real_distance_to_row(40)&&MAX(R_h_guai.row,L_h_guai.row)<real_distance_to_row(10)&&
           (real_distance[imgInfo.top]-real_distance[MAX(R_h_guai.row,L_h_guai.row)])>60&&real_distance[imgInfo.top]>100)
        {
            Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
            red_find_x=red_left;
            red_find_y=real_distance_to_row(real_distance[MAX(R_h_guai.row,L_h_guai.row)]+12);
            red_find_y1=MAX(R_h_guai.row,L_h_guai.row);
            DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right-red_find_x,red_find_y1-red_find_y);
            if(!red_objects.empty())
            {
                red_x_mid=resize_cx;
                red_y_mid=resize_cy;
                err_picture=fabs(real_distance[red_y_mid]-real_distance[MAX(R_h_guai.row,L_h_guai.row)]);
                x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
            }
        }
    }

    if(R_h_guai.flag==1&&R_h_guai.row>real_distance_to_row(40)&&R_h_guai.row<real_distance_to_row(10)&&
       (real_distance[imgInfo.top]-real_distance[R_h_guai.row])>60&&real_distance[imgInfo.top]>100)
    {
        Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
        red_find_x=red_left;
        red_find_y=real_distance_to_row(real_distance[R_h_guai.row]+12);
        red_find_y1=R_h_guai.row;
        DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right-red_find_x,red_find_y1-red_find_y);
        if(!red_objects.empty())
        {
            red_x_mid=resize_cx;
            red_y_mid=resize_cy;
            err_picture=fabs(real_distance[red_y_mid]-real_distance[R_h_guai.row]);
            x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
        }
    }
    else if(L_h_guai.flag==1&&L_h_guai.row>real_distance_to_row(40)&&L_h_guai.row<real_distance_to_row(10)&&
            (real_distance[imgInfo.top]-real_distance[L_h_guai.row])>60&&real_distance[imgInfo.top]>100)
    {
        Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
        red_find_x=red_left;
        red_find_y=real_distance_to_row(real_distance[L_h_guai.row]+12);
        red_find_y1=L_h_guai.row;
        DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right-red_find_x,red_find_y1-red_find_y);
        if(!red_objects.empty())
        {
            red_x_mid=resize_cx;
            red_y_mid=resize_cy;
            err_picture=fabs(real_distance[red_y_mid]-real_distance[L_h_guai.row]);
            x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
        }
    }
}

void UpdateSmallRockStateTransition()
{
    if(x_err_red>5&&imgInfo.R_straight_flag==0&&imgInfo.R_straight_flag==1)
    {
        Flag.small_rock=1;
    }
    if(x_err_red>5&&imgInfo.R_straight_flag==1&&imgInfo.R_straight_flag==0)
    {
        Flag.small_rock=2;
    }
}

void RunSmallRockState0()
{
    if(Flag.small_rock==0)
    {
        ComputeSmallRockSearchCorridor();
        DetectSmallRockCandidates();
        UpdateSmallRockStateTransition();
    }
}

void RunSmallRockTerminalState()
{
    if(Flag.small_rock ==1||Flag.small_rock ==2)
    {
        if(distance>10)
        {
            distance=0;
            Flag.small_rock =0;
        }
    }
}

}  // namespace

void small_rock(void)
{
    RunSmallRockState0();
    RunSmallRockTerminalState();
}



float ramp_line,ramp_err;
void ramp(void)
{

if(Flag.ramp==0)
{

    if(imgInfo.top<20)
  {
        ramp_err=0;
       for(int i = imgInfo.top+5;i<55 ;i++)
    {
          ramp_err+=Dir_Err[i];
    }
       if(primer::port::VisionDistanceRaw()>50&&primer::port::VisionDistanceRaw()<1500&&fabs(ramp_err)<30)//ramp_line==0&&&&B<0.2
    {
           Flag.ramp=2;

    }

  }
}
if(Flag.ramp==2)
{
    if(distance>100)
 {
        Flag.ramp=3;
        distance=0;

 }
}
if(Flag.ramp==3)
{
    if(distance>150)
 {
        Flag.ramp=0;
        distance=0;

 }
}
}
