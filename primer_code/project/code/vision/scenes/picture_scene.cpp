#include "../internal/dependencies/picture_scene_dependencies.hpp"
#include <cmath>
#include <cstdio>


int red_find_x,red_find_y,red_find_y1;
int picture_first_num,picture_second_num,picture_third_num,maxlong_colume,colume_long[94], long_max,picture_white=0,picture_black=0,jump_point1;
float black_ratio;
int red_x_mid,red_y_mid,x_err_red;
float err_picture;
int red_left,red_right;
float real_picture_distance,recognize_distance,recognize_distance2;

namespace {

static inline void ResetPictureObservations()
{
        picture_white=0;
        picture_black=0;
        jump_point1=0;
        black_ratio=0;
        err_picture=100;
        x_err_red=200;
}

static inline void UpdatePictureGeometry()
{
                   float K1 = xielv_sideline(55, Left_Sideline[55], 45, Left_Sideline[45], 'k');
                   float B1 = xielv_sideline(55, Left_Sideline[55], 45, Left_Sideline[45], 'b');
                    float K2 = xielv_sideline(55, Right_Sideline[55], 45, Right_Sideline[45], 'k');
                   float B2 = xielv_sideline(55, Right_Sideline[55], 45, Right_Sideline[45], 'b');
                   red_left=K1*MAX(R_h_guai.row,L_h_guai.row)+B1;
                   red_right=K2*MAX(R_h_guai.row,L_h_guai.row)+B2;

                recognize_distance=primer::port::VisionCurrentSpeed()*0.1+40;
                recognize_distance2=30;
}

static inline bool TryPictureState0BothCorners()
{
      if(R_h_guai.flag==1&&L_h_guai.flag==1)
{

 //           &&(real_distance[imgInfo.top]-real_distance[MAX(R_h_guai.row,L_h_guai.row)])>60&&real_distance[imgInfo.top]>100&&MAX(R_h_guai.row,L_h_guai.row)<real_distance_to_row(40)
            if(MAX(R_h_guai.row,L_h_guai.row)>real_distance_to_row(recognize_distance))
            {
                    Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//94//320
               red_find_y=real_distance_to_row(real_distance[MAX(R_h_guai.row,L_h_guai.row)]+12);//red_find_y到R_h_guai.row（*4)
                              if(red_find_y<imgInfo.top)red_find_y=imgInfo.top;
               red_find_y1=MAX(R_h_guai.row,L_h_guai.row);
               DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right- red_find_x,red_find_y1 - red_find_y);
                // cv::Rect rect(red_find_x,red_find_y,Right_Sideline[MAX(R_h_guai.row,L_h_guai.row)],red_find_y1 - red_find_y);
                // cv::rectangle(resizedFrame, rect, cv::Scalar(0, 255, 0), 1);

                //cv::circle(resizedFrame, bestCenter, 1, cv::Scalar(0, 255, 0), -1);
            if(!red_objects.empty())
               {
                // Flag.Redblock=1;
                red_x_mid=resize_cx;
                red_y_mid=resize_cy;
                err_picture=fabs(real_distance[red_y_mid]-real_distance[MAX(R_h_guai.row,L_h_guai.row)]);
                x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
                // if(err_picture<20)//&&abs(center.x-R_h_guai.column)<20)
                Flag.Redblock=1;
               }
            }


      return true;
}
      return false;
}

static inline bool TryPictureState0RightCorner()
{
     if(R_h_guai.flag==1){
        //&&(real_distance[imgInfo.top]-real_distance[R_h_guai.row])>60&&real_distance[imgInfo.top]>100&&R_h_guai.row<real_distance_to_row(40)
      if(R_h_guai.row>real_distance_to_row(recognize_distance)
      )
{


                    Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//94//320
               red_find_y=real_distance_to_row(real_distance[R_h_guai.row]+12);//red_find_y到R_h_guai.row（*4)
               if(red_find_y<imgInfo.top)red_find_y=imgInfo.top;
               red_find_y1=R_h_guai.row;
               DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right - red_find_x,red_find_y1 - red_find_y);
                // cv::Rect rect(red_find_x,red_find_y,Right_Sideline[R_h_guai.row],red_find_y1 - red_find_y);
                // cv::rectangle(resizedFrame, rect, cv::Scalar(0, 255, 0), 1);

                //cv::circle(resizedFrame, bestCenter, 1, cv::Scalar(0, 255, 0), -1);

            if(!red_objects.empty())
               {
                // Flag.Redblock=1;
                red_x_mid=resize_cx;
                red_y_mid=resize_cy;
                err_picture=fabs(real_distance[red_y_mid]-real_distance[R_h_guai.row]);
                x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
                // if(err_picture<15)//&&abs(center.x-R_h_guai.column)<20
                Flag.Redblock=1;
               }


}
      return true;
}
      return false;
}

static inline bool TryPictureState0LeftCorner()
{
      if(L_h_guai.flag==1){
        //&&(real_distance[imgInfo.top]-real_distance[L_h_guai.row])>60&&real_distance[imgInfo.top]>100&&L_h_guai.row<real_distance_to_row(40)
       if(L_h_guai.row>real_distance_to_row(recognize_distance)
      )
{



                    Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//Right_Sideline[L_h_guai.row]
               red_find_y=real_distance_to_row(real_distance[L_h_guai.row]+12);//
                              if(red_find_y<imgInfo.top)red_find_y=imgInfo.top;
                            red_find_y1=L_h_guai.row;
               DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right - red_find_x,red_find_y1 - red_find_y);
            //    cv::Rect rect(red_find_x,red_find_y,Right_Sideline[L_h_guai.row],red_find_y1 - red_find_y);
            //    cv::rectangle(resizedFrame, rect, cv::Scalar(0, 255, 0), 1);


            if(!red_objects.empty())
               {
                // Flag.Redblock=1;
                red_x_mid=resize_cx;
                red_y_mid=resize_cy;
                err_picture=fabs(real_distance[red_y_mid]-real_distance[L_h_guai.row]);
                x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
                // if(err_picture<15)//&&abs(center.x-L_h_guai.column)<20
                Flag.Redblock=1;
               }

}
      return true;
}
      return false;
}

static inline void RunPictureState0()
{
    if(Flag.picture==0)
    {
               if(!TryPictureState0BothCorners())
               {
                   if(!TryPictureState0RightCorner())
                   {
                       TryPictureState0LeftCorner();
                   }
               }
               if(Flag.Redblock==1){Flag.picture=2;}
    }
}





static inline void RunPictureState1()
{
    if(Flag.picture==1){
    }
}

static inline bool TryPictureState2BothCorners()
{
           if(R_h_guai.flag==1&&L_h_guai.flag==1)
            {

            // if(MAX(R_h_guai.row,L_h_guai.row)>real_distance_to_row(80)&&(real_distance[imgInfo.top]-real_distance[MAX(R_h_guai.row,L_h_guai.row)])>60&&real_distance[imgInfo.top]>100)
            // {
                Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//94//320
               red_find_y=real_distance_to_row(real_distance[MAX(R_h_guai.row,L_h_guai.row)]+12);//red_find_y到R_h_guai.row（*4)
                              if(red_find_y<imgInfo.top)red_find_y=imgInfo.top;
               red_find_y1=MAX(R_h_guai.row,L_h_guai.row);
               DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right - red_find_x,red_find_y1 - red_find_y);
                // cv::Rect rect(red_find_x,red_find_y,Right_Sideline[MAX(R_h_guai.row,L_h_guai.row)],red_find_y1 - red_find_y);
                // cv::rectangle(resizedFrame, rect, cv::Scalar(0, 255, 0), 1);

                //cv::circle(resizedFrame, bestCenter, 1, cv::Scalar(0, 255, 0), -1);
            if(!red_objects.empty())
               {
                // Flag.Redblock=1;
                red_x_mid=resize_cx;
                red_y_mid=resize_cy;
                err_picture=fabs(real_distance[red_y_mid]-real_distance[MAX(R_h_guai.row,L_h_guai.row)]);
                x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
                // if(err_picture<20)//&&abs(center.x-R_h_guai.column)<20)
                // Flag.Redblock=1;
               }
            // }


      return true;
}
      return false;
}

static inline bool TryPictureState2RightCorner()
{
      if(R_h_guai.flag==1&&abs(recognize_distance2-real_distance[R_h_guai.row])<2)
{


                Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//94//320
               red_find_y=real_distance_to_row(real_distance[R_h_guai.row]+12);//red_find_y到R_h_guai.row（*4)
                              if(red_find_y<imgInfo.top)red_find_y=imgInfo.top;
               red_find_y1=R_h_guai.row;
               DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right - red_find_x,red_find_y1 - red_find_y);
                // cv::Rect rect(red_find_x,red_find_y,Right_Sideline[R_h_guai.row],red_find_y1 - red_find_y);
                // cv::rectangle(resizedFrame, rect, cv::Scalar(0, 255, 0), 1);

                //cv::circle(resizedFrame, bestCenter, 1, cv::Scalar(0, 255, 0), -1);

            if(!red_objects.empty())
               {
                // Flag.Redblock=1;
                red_x_mid=resize_cx;
                red_y_mid=resize_cy;
                err_picture=fabs(real_distance[red_y_mid]-real_distance[R_h_guai.row]);
                x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
                // if(err_picture<15)//&&abs(center.x-R_h_guai.column)<20
                // Flag.Redblock=1;
               }

      return true;
}
      return false;
}

static inline bool TryPictureState2LeftCorner()
{
      if(L_h_guai.flag==1&&abs(recognize_distance2-real_distance[L_h_guai.row])<2)
{



                Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//Right_Sideline[L_h_guai.row]
               red_find_y=real_distance_to_row(real_distance[L_h_guai.row]+12);//
                              if(red_find_y<imgInfo.top)red_find_y=imgInfo.top;
                            red_find_y1=L_h_guai.row;
               DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right - red_find_x,red_find_y1 - red_find_y);
            //    cv::Rect rect(red_find_x,red_find_y,Right_Sideline[L_h_guai.row],red_find_y1 - red_find_y);
            //    cv::rectangle(resizedFrame, rect, cv::Scalar(0, 255, 0), 1);


            if(!red_objects.empty())
               {
                // Flag.Redblock=1;
                red_x_mid=resize_cx;
                red_y_mid=resize_cy;
                err_picture=fabs(real_distance[red_y_mid]-real_distance[L_h_guai.row]);
                x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
                // if(err_picture<15)//&&abs(center.x-L_h_guai.column)<20
                // Flag.Redblock=1;
               }
      return true;
}
      return false;
}

static inline void UpdatePictureClassification()
{
                if(Flag.weapon>=1)
                {
                    printf("检测到 weapon\n");//左绕
                    Flag.infer = 0;
                    Flag.supply = 0;
                    Flag.vehicle = 0;
                    Flag.weapon = 0;
                    Flag.picture = 3;

                }

                if(Flag.supply>=1)
                {   printf("检测到 supply\n");//右绕
                    Flag.infer = 0;
                    Flag.supply= 0;
                    Flag.vehicle = 0;
                    Flag.weapon = 0;
                    Flag.picture = 4;

                }


                if(Flag.vehicle>=1)
                {   printf("检测到 vehicle\n");//交通工具
                    Flag.infer = 0;
                    Flag.supply = 0;
                    Flag.vehicle = 0;
                    Flag.weapon = 0;
                    Flag.picture = 5;


                }
}

static inline void RunPictureState2()
{
    if(Flag.picture==2)
    {
        Flag.infer = 1;
        if((L_h_guai.flag==1&&real_distance[MAX(R_h_guai.row,L_h_guai.row)]<30)||(R_h_guai.flag==1&&abs(recognize_distance2-real_distance[MAX(R_h_guai.row,L_h_guai.row)])<2))
        {
            if(!TryPictureState2BothCorners())
            {
                if(!TryPictureState2RightCorner())
                {
                    TryPictureState2LeftCorner();
                }
            }
            UpdatePictureClassification();
        }
    }
}

static inline void RunPictureState3()
{
    if(Flag.picture==3){

        if(distance_picture>30)
        {
            distance_picture=0;
            //    Image.Kp=3.5*3.0;
            //结束绕行
            Flag.picture = 6;//清除标志位
            distance_picture=0;
            printf("left\n");


        }
    }
}

static inline void RunPictureState4()
{
    if(Flag.picture==4){

        if(distance_picture>30)
        {
            distance_picture=0;
            //    Image.Kp=3.5*3.0;
            //结束绕行
            Flag.picture = 6;//清除标志位
            distance_picture=0;
            printf("右行结束\n");

        }
    }
}

static inline void RunPictureState5()
{
        if(Flag.picture==5){

        if(distance_picture>30)
        {
            distance_picture=0;
            // Image.Kp=3.5*3.0;
            //结束绕行
            Flag.picture = 6;//清除标志位
            distance_picture=0;
            printf("straight行结束\n");

        }
    }
}

static inline void RunPictureState6()
{
            if(Flag.picture==6){
        if(distance_picture>40)
        {
            distance_picture=0;
            // Image.Kp=3.5*3.0;
            //结束绕行
            Flag.picture = 0;//清除标志位
            distance_picture=0;
            printf("picture结束\n");

        }
            }
}

}  // namespace

void picture(void)
{
    ResetPictureObservations();
    UpdatePictureGeometry();
    RunPictureState0();
    RunPictureState1();
    RunPictureState2();
    RunPictureState3();
    RunPictureState4();
    RunPictureState5();
    RunPictureState6();
}



void protect(void)
{
}
