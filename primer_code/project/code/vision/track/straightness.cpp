#include "../internal/dependencies/straightness_dependencies.hpp"
#include <cstdlib>


int top_white_num;
int right_num = 0,r_num,left_num = 0,l_num = 0,R_l_lsoe=0,L_l_lose=0;
int L_loseline_l ,L_loseline_h,R_loseline_l ,R_loseline_h;
float k1,kL,kR;
uint16_t maxkuan_line;
void straight_judge(void)
{
              if(R_h_guai.flag==1||L_h_guai.flag==1)//&&(real_distance[imgInfo.top]-real_distance[R_h_guai.row])>25&&imgInfo.Both_lose==0
{
                    Find_right_Sideline(imgInfo.bottom-5,imgInfo.top+1);
                    Find_left_Sideline(imgInfo.bottom-5,imgInfo.top+1);
}


           maxkuan_line=15;
       uint16_t kuan_max = 0;
        for(int i = imgInfo.top;i < 52; i++)
        {
            if((uint16_t)white_width[i]> kuan_max&&white_width[i]<70)//
            {
                kuan_max = (uint16_t)white_width[i];
                maxkuan_line=(uint16_t)i;
            }
        }

        top_white_num=0;
    //    for(int i = imgInfo.top-2;i<imgInfo.top;i++)
    //    {
            for(int j = Left_Sideline[imgInfo.top + 2];j<Right_Sideline[imgInfo.top + 2];j++)
            {
                if(Image_Use[MAX(imgInfo.top-1,0)][j] == white)
                    top_white_num+=1;
            }
//        }
    int count=0;
    for(int i = imgInfo.bottom ;i>imgInfo.top+1;i--)
    {

        if(Left_Sideline_flag[i] == 0)
        {
            count++;
            if(count==1)
            L_loseline_h = i;
            L_loseline_l = i;
        }
        if(count==0)
        {
            L_loseline_h = 60;
            L_loseline_l = 60;
        }

    }
    count=0;
    for(int i = imgInfo.bottom ;i>imgInfo.top+1;i--)
    {

        if(Right_Sideline_flag[i] == 0)
        {
            count++;
            if(count==1)
            R_loseline_h = i;
            R_loseline_l = i;//最低点不更新
        }
        if(count==0)
        {
            R_loseline_h = 60;
            R_loseline_l = 60;
        }

    }
    r_num=0;
    float k = xielv_sideline(24, Right_Sideline[24], 34, Right_Sideline[34], 'k');
    float b = xielv_sideline(24, Right_Sideline[24], 34, Right_Sideline[34], 'b');

//    if(Flag.Huandao_R==5)
        kR=k;

        if(Flag.Huandao_R==6||Flag.Huandao_L==6)
        {
            for(int i = 18 ;i<45 ;i++)
            {
            if(abs(Right_Sideline[i]-k*i-b)>3)
            {
                r_num++;
            }
            }
        }

        else
        {
    for(int i = 12 ;i<36 ;i++)
    {
    if(abs(Right_Sideline[i]-k*i-b)>3)
    {
        r_num++;
    }
    }
        }

    l_num=0;
     k = xielv_sideline(24, Left_Sideline[24], 34, Left_Sideline[34], 'k');
     b = xielv_sideline(24, Left_Sideline[24], 34, Left_Sideline[34], 'b');

//     if(Flag.Huandao_L==5)
     kL=k;

     if(Flag.Huandao_R==6||Flag.Huandao_L==6)
     {
         for(int i = 18;i<45 ;i++)
         {
             if(abs(Left_Sideline[i]-k*i-b)>3)
         {
             l_num++;
         }
         }
     }



     else
     {
    for(int i = 12 ;i<36 ;i++)
    {
        if(abs(Left_Sideline[i]-k*i-b)>3)
    {
        l_num++;
    }
    }
     }

    if(r_num<3)
    {
        imgInfo.R_straight_flag=1;
    }
    else
    {
        imgInfo.R_straight_flag=0;
    }
    if(l_num<3)
    {
        imgInfo.L_straight_flag=1;
    }
    else
    {
        imgInfo.L_straight_flag=0;
    }



       imgInfo.Both_lose=0;
       for(int i = imgInfo.top +5;i<=50 ;i++)
       {
           if(Left_Sideline_flag[i]==0&&Right_Sideline_flag[i]==0)
           {
                imgInfo.Both_lose++;
           }
       }
}

