#include "../internal/dependencies/roundabout_dependencies.hpp"
#include <cstdlib>
float Yaw_Huandao,Yaw_Huandao_err,yaw_correct,distance_HUAN1;//1m=35000

void Huandao_L_imu()
{
    if(Flag.Huandao_L >1)
    {
    huandao_yaw_correct();
    }
    if(Flag.Huandao_L == 0 && Flag.Huandao_R == 0&&Flag.picture!=2&&Flag.picture!=3&&Flag.picture!=4&&Flag.picture!=5)
    {

    if(imgInfo.top <= 12&&r_num<3&& imgInfo.L_loselineSum-imgInfo.R_loselineSum > 3&&R_l_guai.flag==0&&R_h_guai.flag==0&&L_l_guai.flag==1&&L_l_guai.row>20)//
    {
        distance_HUAN1=real_distance[L_l_guai.row]/100*66;
        Flag.Huandao_L = 1;

    }
    }
    /*左环岛*/
    /* 进入条件 左环岛标志位为0 右环岛标志位为0 左下拐点存在 右边丢线行数小于10行 左边丢线行数大于0行 右边没有拐点 截止行在图像较上面（前面不在弯道）*/
    /*此处受图像影响 待图像畸变较小时可以加上左上拐点的上面两行没有边线，便于识别*/


    if(Flag.Huandao_L == 1)
       {
           //进入左环岛判定后进行第二次左环岛判定，左边上下两个拐点都存在
           if(L_l_guai.flag == 1 && L_h_guai.flag == 1)
           {
               //要求左下拐点至少要在图像下方一点
               if(L_l_guai.row > LCDH_1 / 2)
               {
                   float k = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'k');
                   float b = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'b');


                   //从左下拐点遍历到左上拐点，通过斜率进行补线
                   for (int i = L_l_guai.row; i >= L_h_guai.row; i--)
                   {
                       if (!Image_Use[i][(int8)(k * i + b)])
                           continue;
                       else
                           Left_Sideline[i] = k * i + b;
                   }
                   Flag.Buxian = 1;
               }
               //若左下拐点在图像偏中上部分，则以左上拐点到图像下方第五列来进行斜率补线
               else if(L_l_guai.row <= LCDH_1 / 2)
               {
                   float k = xielv_sideline(L_h_guai.row, L_h_guai.column, LCDH_1 - 2, 5, 'k');
                   float b = xielv_sideline(L_h_guai.row, L_h_guai.column, LCDH_1 - 2, 5, 'b');


                   for (int i = LCDH_1 - 2; i >= L_h_guai.row; i--)
                   {

                       if(!Image_Use[i][(int8)(k * i + b)])
                           continue;
                       else
                           Left_Sideline[i] = k * i + b;
                   }
               }

           }
           //若进入了左环岛二级判断仍只有左下拐点
           else if(L_l_guai.flag == 1)
           {
               //从左下拐点到顶部边线进行补线
               float k = xielv_sideline(L_l_guai.row, L_l_guai.column, imgInfo.bottom - 1, Left_Sideline[imgInfo.bottom - 1], 'k');
               float b = xielv_sideline(L_l_guai.row, L_l_guai.column, imgInfo.bottom - 1, Left_Sideline[imgInfo.bottom - 1], 'b');

               for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
               {
                   if (!Image_Use[i][(int8)(k * i + b)])
                       continue;
                   else
                       Left_Sideline[i] = k * i + b;

               }
               Flag.Buxian = 1;
           }
           else if(R_l_guai.flag == 1 || R_h_guai.flag == 1 || imgInfo.R_loselineSum > 10)//
           {
               Flag.Huandao_L = 0;
           }
           //若圆环二级判断左上左下拐点都没有
           else
           {
               //顶部到底部进行补线
               float k = xielv_sideline(imgInfo.top + 1, Left_Sideline[imgInfo.top + 1], LCDH_1 - 2, 5, 'k');
               float b = xielv_sideline(imgInfo.top + 1, Left_Sideline[imgInfo.top + 1], LCDH_1 - 2, 5, 'b');

               for (int i = LCDH_1 - 2; i >= imgInfo.top + 1; i--)
               {

                   if(!Image_Use[i][(int8)(k * i + b)])
                       continue;
                   else
                       Left_Sideline[i] = k * i + b;
               }
           }
           //若是左环岛二级判断底部三行左边线丢线了 且左边丢线行数大于25行，说明此时车身已经靠在圆环出口那里了
           if((Left_Sideline_flag[LCDH_1 - 4] == 0 && Left_Sideline_flag[LCDH_1 - 5] == 0) && imgInfo.L_loselineSum > 10&&distance>distance_HUAN1)
           {
               Flag.Huandao_L = 2;
               Yaw_Huandao=icm_data.yaw;
               distance=0;
               distance_HUAN1=0;
           }

       }

         if(Flag.Huandao_L == 2)
        {
              float k ;
              float b ;

              if( L_h_guai.flag == 1)
              {
              k = xielv_sideline(L_h_guai.row, L_h_guai.column, MIN(L_h_guai.row+5,57), Right_Sideline[MIN(L_h_guai.row+5,57)], 'k');
              b = xielv_sideline(L_h_guai.row, L_h_guai.column, MIN(L_h_guai.row+5,57), Right_Sideline[MIN(L_h_guai.row+5,57)], 'b');

             for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
             {
                 if(!Image_Use[i][(int8)(k * i + b)])
                    continue;
                 else
                 {
                     for(int m = 0; m <= 7; m++)
                     {
                         if(((k * i + b) + m) >= LCDW_1 - 1)
                         {
                             break;
                         }
                         else
                             Image_Use[i][(int8)(k * i + b) + m] = black;
                     }
                 }
             }

             Flag.Buxian = 1;
              }
            if( L_l_guai.flag == 1)//L_h_guai.flag == 1 &&
            {

                k=-0.5;
                b=L_l_guai.column-k*L_l_guai.row;
                for (int i = imgInfo.bottom-1; i >= L_l_guai.row; i--)
                {

                    if(!Image_Use[i][(int8)(k * i + b)])
                        continue;
                    else
                    {
                        for(int m = 0; m >=-7; m--)
                        {
                            if(((k * i + b) + m) <=0)
                            {
                                break;
                            }
                            else
                                Image_Use[i][(int8)(k * i + b) + m] = black;
                        }
                    }
                }



                Flag.Buxian = 1;

            }


            else
            {
                float k = xielv_sideline(3, 40, LCDH_1 - 2, 5, 'k');
                float b = xielv_sideline(3, 40, LCDH_1 - 2, 5, 'b');

                for (int i = LCDH_1 - 2; i >= imgInfo.top + 1; i--)
                {

                    if(!Image_Use[i][(int8)(k * i + b)])
                        continue;
                    else
                        Left_Sideline[i] = k * i + b;
                }

                k = xielv_sideline(3 , 38, LCDH_1 - 1, LCDW_1 - 25, 'k');
                b = xielv_sideline(3 , 38, LCDH_1 - 1, LCDW_1 - 25, 'b');

                for (int i = LCDH_1 - 2; i >= imgInfo.top + 1; i--)
                {

                    if(!Image_Use[i][(int8)(k * i + b)])
                        continue;
                    else
                        Right_Sideline[i] = k * i + b;
                }

                    Flag.Buxian = 1;
                }


            if(((Left_Sideline_flag[LCDH_1 - 2] == 1 && Left_Sideline_flag[LCDH_1 - 3] == 1 && Left_Sideline_flag[LCDH_1 - 4] == 1
                    && L_h_guai.flag) )&&distance>13.6)//&&distance>10|| !L_l_guai.flag
            {
                Flag.Huandao_L = 3;
                distance=0;
            }
            Get_ImageTop();
            Find_Sideline(imgInfo.bottom - 1, imgInfo.top + 1);
        }

         if(Flag.Huandao_L == 3)
            {
             if( L_h_guai.flag == 1)//L_h_guai.flag == 1 &&
             {
                    float k = xielv_sideline(L_h_guai.row, L_h_guai.column, MIN(L_h_guai.row+20,57),Right_Sideline[MIN(L_h_guai.row+20,57)], 'k');
                    float b = xielv_sideline(L_h_guai.row, L_h_guai.column, MIN(L_h_guai.row+20,57),Right_Sideline[MIN(L_h_guai.row+20,57)], 'b');

                    for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
                    {
                        if(!Image_Use[i][(int8)(k * i + b)])
                           continue;
                        else
                        {
                            for(int m = 0; m <= 7; m++)
                            {
                                if(((k * i + b) + m) >= LCDW_1 - 1)
                                {
                                    break;
                                }
                                else
                                    Image_Use[i][(int8)(k * i + b) + m] = black;
                            }
                            Flag.Buxian = 1;
                        }
                    }
                }
            //  else if(  L_h_guai.flag == 0&&R_h_guai.flag == 1)//L_h_guai.flag == 1 &&
            //  {
            //         float k = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 1, LCDW_1 - 20, 'k');
            //         float b = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 1, LCDW_1 - 20, 'b');

            //         for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
            //         {
            //             if(!Image_Use[i][(int8)(k * i + b)])
            //                continue;
            //             else
            //             {
            //                 for(int m = 0; m <= 7; m++)
            //                 {
            //                     if(((k * i + b) + m) >= LCDW_1 - 1)
            //                     {
            //                         break;
            //                     }
            //                     else
            //                         Image_Use[i][(int8)(k * i + b) + m] = black;
            //                 }
            //                 Flag.Buxian = 1;
            //             }
            //         }
            //     }
//             else
//             {
//                 Eerr_flag=1;
////                    float k = xielv_sideline(1, 0, imgInfo.bottom - 1, LCDW_1 - 20, 'k');
////                    float b = xielv_sideline(1, 0, imgInfo.bottom - 1, LCDW_1 - 20, 'b');
////
////                    for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
////                    {
////                        if(!Image_Use[i][(int8)(k * i + b)])
////                           continue;
////                        else
////                        {
////                            for(int m = 0; m <= 7; m++)
////                            {
////                                if(((k * i + b) + m) >= LCDW_1 - 1)
////                                {
////                                    break;
////                                }
////                                else
////                                    Image_Use[i][(int8)(k * i + b) + m] = black;
////                            }
//                            Flag.Buxian = 1;
////                        }
////                    }
//                }
                //环岛3是直接把USE图像数组改了，所以重新找截止行来确定手动补线后的截止行和边线，这时找到的图像就是一个大弯拐进环岛
                Get_ImageTop();
                Find_Sideline(imgInfo.bottom - 1, imgInfo.top + 1);

                //进入弯道
                if(Yaw_Huandao_err<-105)//(distance>30)&&imgInfo.Both_lose==0
                {
                    Flag.Huandao_L = 4;
//                    distance=0;
                }
            }

             if(Flag.Huandao_L == 4)
            {
                //找到右下拐点，即说明已经到了快要出弯的地方
                if(R_l_guai.flag)
                {
                    uint8 temp = 0;
                    //找到右赛道的最顶的边线，从那个点补到右下拐点上
//                    for(int m = imgInfo.bottom - 4; m > imgInfo.top + 3; m--)
//                    {
//                        if(!Left_Sideline_flag[m + 1] && !Left_Sideline_flag[m + 2] && !Left_Sideline_flag[m + 3] && !Left_Sideline_flag[m + 4] && !Left_Sideline_flag[m + 5] && Left_Sideline_flag[m - 1] && Left_Sideline_flag[m - 2])
//                        {
//                            temp =(uint8) m;
//                            break;
//                        }
//                    }
//                    if(temp == 0)
//                    {
                        temp = imgInfo.top + 1;
//                    }
                    float k = xielv_sideline(R_l_guai.row, R_l_guai.column, temp, 30, 'k');
                    float b = xielv_sideline(R_l_guai.row, R_l_guai.column, temp, 30, 'b');

                    for (int i = R_l_guai.row; i > imgInfo.top; i--)
                    {
                        for(int m = 0; m <= 13; m++)
                        {
                            if(((k * i + b) + m) >= LCDW_1 - 1)
                            {
                                break;
                            }
                            else
                            {
                                Image_Use[i][(uint8)(k * i + b) + m] = black;
                            }

                        }
                    }
                    Flag.Buxian = 1;
                }
            //    else if((!R_l_guai.flag) && abs(Left_Sideline[imgInfo.top+2]-Right_Sideline[imgInfo.top+2])>30 )
            //    {
            //        uint8 temp = 0;

            //         for(int m = imgInfo.bottom - 4; m > imgInfo.top + 1; m--)
            //         {
            //             if(!Left_Sideline_flag[m + 1] && !Left_Sideline_flag[m + 2] && !Left_Sideline_flag[m + 3] && !Left_Sideline_flag[m + 4] && !Left_Sideline_flag[m + 5] && Left_Sideline_flag[m - 1])
            //             {
            //                 temp =(uint8) m;
            //                 break;
            //             }
            //         }
            //         if(temp == 0)
            //         {
            //             temp = imgInfo.top + 1;
            //         }
            //        float k = xielv_sideline(LCDH_1 - 2, LCDW_1 - 2, temp, 30, 'k');
            //        float b = xielv_sideline(LCDH_1 - 2, LCDW_1 - 2, temp, 30, 'b');

            //        for (int i = LCDH_1 - 2; i >= temp - 2; i--)
            //        {
            //            for(int m = 0; m <= 4; m++)
            //            {
            //                if(((k * i + b) + m) >= LCDW_1 - 1)
            //                {
            //                    break;
            //                }
            //                else
            //                {
            //                    Image_Use[i][(uint8)(k * i + b) + m] = black;
            //                }

            //            }
            //        }
            //        Flag.Buxian = 1;
            //    }
                else
                {
                    Flag.Buxian = 1;
                }
                //没有右下拐点但还有右丢线情况，说明车已经跨过右拐点那里，这是从m点补到右下角
                // if(
                //         R_l_guai.flag == 0 && imgInfo.R_loselineSum > 15 && !Right_Sideline_flag[LCDH_1 - 3] && !Right_Sideline_flag[LCDH_1 - 4]
                //         && !Right_Sideline_flag[LCDH_1 - 5] && !Right_Sideline_flag[LCDH_1 - 6]
                //         && !Right_Sideline_flag[LCDH_1 - 7] && !Right_Sideline_flag[LCDH_1 - 8]
                // )
                // {
                //     Flag.Huandao_L = 5;
                // }

                if(Yaw_Huandao_err<110&&Yaw_Huandao_err>100)//(L_h_guai.flag==1&&imgInfo.R_straight_flag==1)&&imgInfo.top<10
                {
                    Flag.Huandao_L = 5;
                }
                Get_ImageTop();
                Find_Sideline(imgInfo.bottom - 1, imgInfo.top + 1);
                imgInfo.R_loselineSum = 0;
            }


              if(Flag.Huandao_L == 5)
                 {
                     uint8 temp = 0;
                     //找圆弧内白到黑的交界点
                     for(int m = imgInfo.bottom - 4; m > imgInfo.top + 3; m--)
                     {
                         if(!Left_Sideline_flag[m + 1] && !Left_Sideline_flag[m + 2] && !Left_Sideline_flag[m + 3]
                            && !Left_Sideline_flag[m + 4] && !Left_Sideline_flag[m + 5] && Left_Sideline_flag[m - 1])
                         {
                             temp = (uint8)m;
                             break;
                         }
                     }
                     if(temp == 0)
                     {
                         temp = imgInfo.top + 3;
                     }
                     float k = xielv_sideline(LCDH_1 - 2, LCDW_1 - 2, temp, 40, 'k');
                     float b = xielv_sideline(LCDH_1 - 2, LCDW_1 - 2, temp, 40, 'b');

                     //只要有右丢线情况就进行补线
                     if(imgInfo.R_loselineSum > 1)
                     {
                         for (int i = LCDH_1 - 2; i >= temp - 2; i--)
                         {
                             for(int m = 0; m <= 4; m++)
                             {
                                 if(((k * i + b) + m) >= LCDW_1 - 1)
                                 {
                                     break;
                                 }
                                 else
                                     Image_Use[i][(uint8)(k * i + b) + m] = black;
                             }
                         }
                         Flag.Buxian = 1;
                     }

                     //直到右边没有丢线情况并且左边出现拐点只后
                     if(Yaw_Huandao_err<25)//(L_h_guai.flag==1&&imgInfo.R_straight_flag==1)&&imgInfo.top<10
                     {
                         Flag.Huandao_L = 6;
                         Flag.Buxian = 0;
                     }
                     Get_ImageTop();
                     Find_Sideline(imgInfo.bottom - 1, imgInfo.top + 1);
                 }

                  if(Flag.Huandao_L == 6)
                 {
                     if(L_h_guai.flag == 1)
                     {
                         //从左拐点向下补线，防止再次入环
                         float k = xielv_sideline(L_h_guai.row + 5, L_h_guai.column, LCDH_1 - 2, 8, 'k');
                         float b = xielv_sideline(L_h_guai.row + 5, L_h_guai.column, LCDH_1 - 2, 8, 'b');

                         for (int i = LCDH_1 - 2; i >= L_h_guai.row; i--)
                         {

                             if(!Image_Use[i][(uint8)(k * i + b)])
                                 continue;
                             else
                                 Left_Sideline[i] = k * i + b;
                         }
                     }
                     else if(L_h_guai.flag == 0)
                     {
                         float k = xielv_sideline(imgInfo.top + 10, Left_Sideline[imgInfo.top + 10], LCDH_1 - 2, 5, 'k');
                         float b = xielv_sideline(imgInfo.top + 10, Left_Sideline[imgInfo.top + 10], LCDH_1 - 2, 5, 'b');

                         for (int i = LCDH_1 - 2; i >= imgInfo.top + 1; i--)
                         {

                             if(!Image_Use[i][(uint8)(k * i + b)])
                                 continue;
                             else
                                 Left_Sideline[i] = k * i + b;
                         }
                     }
                     else
                         Flag.Buxian = 0;

                     if(imgInfo.L_straight_flag==1&&distance>=10&&L_h_guai.flag==0)//imgInfo.R_loselineSum < 1//Left_Sideline_flag[imgInfo.bottom - 6] && Left_Sideline_flag[imgInfo.bottom - 7] && Left_Sideline_flag[imgInfo.bottom - 8]&&
                     {
                         Flag.Huandao_L = 0;
                         distance=0;
                     }
                 }

                  if((imgInfo.L_straight_flag==1&& imgInfo.R_straight_flag==1)&&L_h_guai.flag==0&&L_l_guai.flag==0&&R_h_guai.flag==0&&R_l_guai.flag==0&&abs(imgInfo.R_loselineSum-imgInfo.L_loselineSum)==0)//(Flag.Huandao_R>4&&  imgInfo.R_straight_flag==1)||
                  {
                                               Flag.Huandao_L = 0;
                  }
//                 if(Flag.Huandao_L && imgInfo.L_loselineSum < 10 && imgInfo.R_loselineSum < 10)
//                 {
//                     for(int i = imgInfo.bottom - 1; i > imgInfo.top + 1; i--)
//                     {
//                         if(i > imgInfo.top)
//                         {
//                             //右边界不发生突变且呈直线状态
//                             if(Right_Sideline[i] - Right_Sideline[i - 1] < 3 && Right_Sideline[i] >= Right_Sideline[i - 1])
//                             {
//                                 right_num ++;
//                             }
//                             //左边界不发生突变且呈直线状态
//                             if(Left_Sideline[i] - Left_Sideline[i - 1] < 3 && Left_Sideline[i] >= Left_Sideline[i - 1])
//                             {
//                                 left_num ++;
//                             }
//                         }
//                     }
//
//                     if(right_num > 50 && left_num > 50)
//                     {
//                         Flag.Huandao_L = 0;
//                         left_num = 0;
//                         right_num = 0;
//
//                     }
//                     else
//                     {
//                         left_num = 0;
//                         right_num = 0;
//                     }
//                 }


}










void Huandao_R_imu()
{

    if(Flag.Huandao_R >1)
    {
        huandao_yaw_correct();
    }
    if(Flag.Huandao_R == 0 && Flag.Huandao_L == 0&&Flag.picture!=2&&Flag.picture!=3&&Flag.picture!=4&&Flag.picture!=5)
    {

    if(imgInfo.top <= 12&&l_num<3&& imgInfo.R_loselineSum-imgInfo.L_loselineSum > 3&&L_l_guai.flag==0&&L_h_guai.flag==0&&R_l_guai.flag==1&&R_l_guai.row>20)//
    {
        distance_HUAN1=real_distance[R_l_guai.row]/100*66;
        Flag.Huandao_R = 1;

    }
    }
    /*右环岛*/
    /* 进入条件 左环岛标志位为0 右环岛标志位为0 左下拐点存在 右边丢线行数小于10行 左边丢线行数大于0行 右边没有拐点 截止行在图像较上面（前面不在弯道）*/
    /*此处受图像影响 待图像畸变较小时可以加上左上拐点的上面两行没有边线，便于识别*/


    if(Flag.Huandao_R == 1)
       {
           //进入左环岛判定后进行第二次左环岛判定，左边上下两个拐点都存在
           if(R_l_guai.flag == 1 && R_h_guai.flag == 1)
           {
               //要求左下拐点至少要在图像下方一点
               if(R_l_guai.row > LCDH_1 / 2)
               {
                   float k = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'k');
                   float b = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'b');


                   //从左下拐点遍历到左上拐点，通过斜率进行补线
                   for (int i = R_l_guai.row; i >= R_h_guai.row; i--)
                   {
                       if (!Image_Use[i][(int8)(k * i + b)])
                           continue;
                       else
                           Right_Sideline[i] = k * i + b;
                   }
                   Flag.Buxian = 1;
               }
               //若左下拐点在图像偏中上部分，则以左上拐点到图像下方第五列来进行斜率补线
               else if(R_l_guai.row <= LCDH_1 / 2)
               {
                   float k = xielv_sideline(R_h_guai.row, R_h_guai.column, LCDH_1 - 2, 88, 'k');
                   float b = xielv_sideline(R_h_guai.row, R_h_guai.column, LCDH_1 - 2, 88, 'b');


                   for (int i = LCDH_1 - 2; i >= R_h_guai.row; i--)
                   {

                       if(!Image_Use[i][(int8)(k * i + b)])
                           continue;
                       else
                           Right_Sideline[i] = k * i + b;
                   }
               }

           }
           //若进入了左环岛二级判断仍只有左下拐点
           else if(R_l_guai.flag == 1)
           {
               //从左下拐点到顶部边线进行补线
               float k = xielv_sideline(R_l_guai.row, R_l_guai.column, imgInfo.bottom - 1, Right_Sideline[imgInfo.bottom - 1], 'k');
               float b = xielv_sideline(R_l_guai.row, R_l_guai.column, imgInfo.bottom - 1, Right_Sideline[imgInfo.bottom - 1], 'b');

               for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
               {
                   if (!Image_Use[i][(int8)(k * i + b)])
                       continue;
                   else
                       Right_Sideline[i] = k * i + b;

               }
               Flag.Buxian = 1;
           }
           else if(L_l_guai.flag == 1 || L_h_guai.flag == 1 || imgInfo.L_loselineSum > 10)
           {
               Flag.Huandao_R = 0;
           }
           //若圆环二级判断左上左下拐点都没有
           else
           {
               //顶部到底部进行补线
               float k = xielv_sideline(imgInfo.top + 1, Right_Sideline[imgInfo.top + 1], LCDH_1 - 2, 88, 'k');
               float b = xielv_sideline(imgInfo.top + 1, Right_Sideline[imgInfo.top + 1], LCDH_1 - 2, 88, 'b');

               for (int i = LCDH_1 - 2; i >= imgInfo.top + 1; i--)
               {

                   if(!Image_Use[i][(int8)(k * i + b)])
                       continue;
                   else
                       Right_Sideline[i] = k * i + b;
               }
           }
           //若是左环岛二级判断底部三行左边线丢线了 且左边丢线行数大于25行，说明此时车身已经靠在圆环出口那里了
           if((Right_Sideline_flag[LCDH_1 - 4] == 0 && Right_Sideline_flag[LCDH_1 - 5] == 0) && imgInfo.R_loselineSum > 10&&distance>distance_HUAN1)
           {
               distance=0;
               distance_HUAN1=0;
               Flag.Huandao_R = 2;
               Yaw_Huandao=icm_data.yaw;
           }
       }

         if(Flag.Huandao_R == 2)
        {
                float k ;
                float b ;

                if( R_h_guai.flag == 1)//L_h_guai.flag == 1 &&
                {
                    k = xielv_sideline(R_h_guai.row, R_h_guai.column, MIN(R_h_guai.row+5,57), Left_Sideline[MIN(R_h_guai.row+5,57)], 'k');
                    b = xielv_sideline(R_h_guai.row, R_h_guai.column,MIN(R_h_guai.row+5,57), Left_Sideline[MIN(R_h_guai.row+5,57)], 'b');

                   for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
                   {
                       if(!Image_Use[i][(int8)(k * i + b)])
                          continue;
                       else
                       {
                           for(int m = 0; m >=-7; m--)
                           {
                               if(((k * i + b) + m) <=0)
                               {
                                   break;
                               }
                               else
                                   Image_Use[i][(int8)(k * i + b) + m] = black;
                           }
                       }
                   }

                   Flag.Buxian = 1;
                }
            //左上拐点存在，继续补线
            if( R_l_guai.flag == 1)//L_h_guai.flag == 1 &&
            {

                k=0.5;
                b=R_l_guai.column-k*R_l_guai.row;
                for (int i = imgInfo.bottom-1; i >= R_l_guai.row; i--)
                {

                    if(!Image_Use[i][(int8)(k * i + b)])
                        continue;
                    else
                    {
                        for(int m = 0; m <=7; m++)
                        {
                            if(((k * i + b) + m) >=93)
                            {
                                break;
                            }
                            else
                                Image_Use[i][(int8)(k * i + b) + m] = black;
                        }
                    }
                }



                Flag.Buxian = 1;


            }

            else
            {
                float k = xielv_sideline(3, 53, LCDH_1 - 2, 88, 'k');
                float b = xielv_sideline(3, 53, LCDH_1 - 2, 88, 'b');

                for (int i = LCDH_1 - 2; i >= imgInfo.top + 1; i--)
                {

                    if(!Image_Use[i][(int8)(k * i + b)])
                        continue;
                    else
                        Right_Sideline[i] = k * i + b;
                }

                k = xielv_sideline(3 , 55, LCDH_1 - 1, 24, 'k');
                b = xielv_sideline(3 , 55, LCDH_1 - 1, 24, 'b');

                for (int i = LCDH_1 - 2; i >= imgInfo.top + 1; i--)
                {

                    if(!Image_Use[i][(int8)(k * i + b)])
                        continue;
                    else
                        Left_Sideline[i] = k * i + b;
                }

                    Flag.Buxian = 1;
                }


            if(((Right_Sideline_flag[LCDH_1 - 2] == 1 && Right_Sideline_flag[LCDH_1 - 3] == 1 && Right_Sideline_flag[LCDH_1 - 4] == 1
                    && R_h_guai.flag) )&&distance>13.6)//|| !R_l_guai.flag
            {
                Flag.Huandao_R = 3;
                distance=0;
            }
            Get_ImageTop();
            Find_Sideline(imgInfo.bottom - 1, imgInfo.top + 1);

        }

         if(Flag.Huandao_R == 3)
            {

             if( R_h_guai.flag == 1)//L_h_guai.flag == 1 &&
             {
                    float k = xielv_sideline(R_h_guai.row, R_h_guai.column, MIN(R_h_guai.row+20,57), Left_Sideline[MIN(R_h_guai.row+20,57)], 'k');
                    float b = xielv_sideline(R_h_guai.row, R_h_guai.column, MIN(R_h_guai.row+20,57), Left_Sideline[MIN(R_h_guai.row+20,57)], 'b');

                    for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
                    {
                        if(!Image_Use[i][(int8)(k * i + b)])
                           continue;
                        else
                        {
                            for(int m = 0; m >=-7; m--)
                            {
                                if(((k * i + b) + m) <=0)
                                {
                                    break;
                                }
                                else
                                    Image_Use[i][(int8)(k * i + b) + m] = black;
                            }
                            Flag.Buxian = 1;
                        }
                    }
                }
            //  else if(  R_h_guai.flag == 0&&L_h_guai.flag == 1)//L_h_guai.flag == 1 &&
            //  {
            //         float k = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 1,19, 'k');
            //         float b = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 1, 19, 'b');

            //         for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
            //         {
            //             if(!Image_Use[i][(int8)(k * i + b)])
            //                continue;
            //             else
            //             {
            //                 for(int m = 0; m >=- 7; m--)
            //                 {
            //                     if(((k * i + b) + m) <=0)
            //                     {
            //                         break;
            //                     }
            //                     else
            //                         Image_Use[i][(int8)(k * i + b) + m] = black;
            //                 }
            //                 Flag.Buxian = 1;
            //             }
            //         }
            //     }

//             else
//             {
//                 Eerr_flag=1;
//
////                    float k = xielv_sideline(1, 0, imgInfo.bottom - 1, 19, 'k');
////                    float b = xielv_sideline(1, 0, imgInfo.bottom - 1, 19, 'b');
////
////                    for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
////                    {
////                        if(!Image_Use[i][(int8)(k * i + b)])
////                           continue;
////                        else
////                        {
////                            for(int m = 0; m >= -7; m--)
////                            {
////                                if(((k * i + b) + m) <=0)
////                                {
////                                    break;
////                                }
////                                else
////                                    Image_Use[i][(int8)(k * i + b) + m] = black;
////                            }
//                            Flag.Buxian = 1;
//                        }
//                    }
//                }

                //环岛3是直接把USE图像数组改了，所以重新找截止行来确定手动补线后的截止行和边线，这时找到的图像就是一个大弯拐进环岛
                Get_ImageTop();
                Find_Sideline(imgInfo.bottom - 1, imgInfo.top + 1);

                //进入弯道
                if(Yaw_Huandao_err>105)//(distance>30)&&imgInfo.Both_lose==0
                {
                   Flag.Huandao_R = 4;
//                    distance=0;
                }
            }

             if(Flag.Huandao_R == 4)
            {
                //找到右下拐点，即说明已经到了快要出弯的地方
                if(L_l_guai.flag)
                {
                    uint8 temp = 0;
                        temp = imgInfo.top + 1;
                    float k = xielv_sideline(L_l_guai.row, L_l_guai.column, temp, 63, 'k');
                    float b = xielv_sideline(L_l_guai.row, L_l_guai.column, temp, 63, 'b');

                    for (int i = L_l_guai.row; i > imgInfo.top; i--)
                    {
                        for(int m = 0; m >=-13; m--)
                        {
                            if(((k * i + b) + m) <= 0)
                            {
                                break;
                            }
                            else
                            {
                                Image_Use[i][(uint8)(k * i + b) + m] = black;
                            }

                        }
                    }
                    Flag.Buxian = 1;
                }
            //    else if((!L_l_guai.flag) &&  abs(Left_Sideline[imgInfo.top+2]-Right_Sideline[imgInfo.top+2])>30 )
            //    {
            //        uint8 temp = 0;

            //     //    for(int m = imgInfo.bottom - 4; m > imgInfo.top + 1; m--)
            //     //    {
            //     //        if(!Right_Sideline_flag[m + 1] && !Right_Sideline_flag[m + 2] && !Right_Sideline_flag[m + 3] && !Right_Sideline_flag[m + 4] && !Right_Sideline_flag[m + 5] && Right_Sideline_flag[m - 1])
            //     //        {
            //     //            temp =(uint8) m;
            //     //            break;
            //     //        }
            //     //    }
            //     //    if(temp == 0)
            //     //    {
            //            temp = imgInfo.top + 1;
            //     //    }
            //        float k = xielv_sideline(LCDH_1 - 2, 1, temp, 63, 'k');
            //        float b = xielv_sideline(LCDH_1 - 2, 1, temp, 63, 'b');

            //        for (int i = LCDH_1 - 2; i >= temp - 2; i--)
            //        {
            //            for(int m = 0; m >=-4; m--)
            //            {
            //                if(((k * i + b) + m) <= 0)
            //                {
            //                    break;
            //                }
            //                else
            //                {
            //                    Image_Use[i][(uint8)(k * i + b) + m] = black;
            //                }

            //            }
            //        }
            //        Flag.Buxian = 1;
            //    }
                else
                {
                                        Flag.Buxian = 1;
                }
                // if(
                //        imgInfo.L_loselineSum > 15 && !Left_Sideline_flag[LCDH_1 - 3] && !Left_Sideline_flag[LCDH_1 - 4]
                //         && !Left_Sideline_flag[LCDH_1 - 5] && !Left_Sideline_flag[LCDH_1 - 6]
                //         && !Left_Sideline_flag[LCDH_1 - 7] && !Left_Sideline_flag[LCDH_1 - 8]
                // )
                // {
                //     Flag.Huandao_R = 5;
                // }

                if(Yaw_Huandao_err>-110&&Yaw_Huandao_err<-100)//(L_h_guai.flag==1&&imgInfo.R_straight_flag==1)&&imgInfo.top<10
                {
                    Flag.Huandao_R = 5;
                }

                Get_ImageTop();
                Find_Sideline(imgInfo.bottom - 1, imgInfo.top + 1);
                imgInfo.L_loselineSum = 0;
            }


              if(Flag.Huandao_R == 5)
                 {
                     uint8 temp = 0;
                     //找圆弧内白到黑的交界点
                     for(int m = imgInfo.bottom - 4; m > imgInfo.top + 3; m--)
                     {
                         if(!Right_Sideline_flag[m + 1] && !Right_Sideline_flag[m + 2] && !Right_Sideline_flag[m + 3]
                            && !Right_Sideline_flag[m + 4] && !Right_Sideline_flag[m + 5] && Right_Sideline_flag[m - 1])
                         {
                             temp = (uint8)m;
                             break;
                         }
                     }
                     if(temp == 0)
                     {
                         temp = imgInfo.top + 3;
                     }
                     float k = xielv_sideline(LCDH_1 - 2, 1, temp, 53, 'k');
                     float b = xielv_sideline(LCDH_1 - 2, 1, temp, 53, 'b');

                     //只要有右丢线情况就进行补线
                     if(imgInfo.L_loselineSum > 1)
                     {
                         for (int i = LCDH_1 - 2; i >= temp - 2; i--)
                         {
                             for(int m = 0; m >=-4; m--)
                             {
                                 if(((k * i + b) + m) <= 0)
                                 {
                                     break;
                                 }
                                 else
                                     Image_Use[i][(uint8)(k * i + b) + m] = black;
                             }
                         }
                         Flag.Buxian = 1;
                     }

                     //直到右边没有丢线情况并且左边出现拐点只后
                     if(Yaw_Huandao_err>-25)//(R_h_guai.flag==1&&imgInfo.L_straight_flag==1)&&imgInfo.top<10
                     {
                         Flag.Huandao_R = 6;
                         Flag.Buxian = 0;
                     }
                     Get_ImageTop();
                     Find_Sideline(imgInfo.bottom - 1, imgInfo.top + 1);
                 }

                  if(Flag.Huandao_R == 6)
                 {
                     if(R_h_guai.flag == 1)
                     {
                         //从左拐点向下补线，防止再次入环
                         float k = xielv_sideline(R_h_guai.row + 5, R_h_guai.column, LCDH_1 - 2, 85, 'k');
                         float b = xielv_sideline(R_h_guai.row + 5, R_h_guai.column, LCDH_1 - 2, 85, 'b');

                         for (int i = LCDH_1 - 2; i >= R_h_guai.row; i--)
                         {

                             if(!Image_Use[i][(uint8)(k * i + b)])
                                 continue;
                             else
                                 Right_Sideline[i] = k * i + b;
                         }
                     }
                     else if(R_h_guai.flag == 0)
                     {
                         float k = xielv_sideline(imgInfo.top + 10, Right_Sideline[imgInfo.top + 10], LCDH_1 - 2, 88, 'k');
                         float b = xielv_sideline(imgInfo.top + 10, Right_Sideline[imgInfo.top + 10], LCDH_1 - 2, 88, 'b');

                         for (int i = LCDH_1 - 2; i >= imgInfo.top + 1; i--)
                         {

                             if(!Image_Use[i][(uint8)(k * i + b)])
                                 continue;
                             else
                                 Right_Sideline[i] = k * i + b;
                         }
                     }
                     else
                         Flag.Buxian = 0;

                     if(imgInfo.R_straight_flag==1&&distance>=10&&R_h_guai.flag==0)//imgInfo.R_loselineSum < 1//Left_Sideline_flag[imgInfo.bottom - 6] && Left_Sideline_flag[imgInfo.bottom - 7] && Left_Sideline_flag[imgInfo.bottom - 8]&&
                     {
                         Flag.Huandao_R = 0;
                         distance=0;
                     }
                 }

                  if((imgInfo.L_straight_flag==1&& imgInfo.R_straight_flag==1)&&L_h_guai.flag==0&&L_l_guai.flag==0&&R_h_guai.flag==0&&R_l_guai.flag==0&&abs(imgInfo.R_loselineSum-imgInfo.L_loselineSum)==0)//(Flag.Huandao_R>4&&  imgInfo.R_straight_flag==1)||
                  {
                                               Flag.Huandao_R = 0;
                  }
//                 if(Flag.Huandao_R && imgInfo.R_loselineSum < 10 && imgInfo.L_loselineSum < 10)
//                 {
//                     for(int i = imgInfo.bottom - 1; i > imgInfo.top + 1; i--)
//                     {
//                         if(i > imgInfo.top)
//                         {
//                             //右边界不发生突变且呈直线状态
//                             if(Left_Sideline[i] - Left_Sideline[i - 1] < 3 && Left_Sideline[i] >= Left_Sideline[i - 1])
//                             {
//                                 right_num ++;
//                             }
//                             //左边界不发生突变且呈直线状态
//                             if(Right_Sideline[i] - Right_Sideline[i - 1] < 3 && Right_Sideline[i] >= Right_Sideline[i - 1])
//                             {
//                                 right_num ++;
//                             }
//                         }
//                     }
//
//                     if(left_num > 50 && right_num > 50)
//                     {
//                         Flag.Huandao_R = 0;
//                         left_num = 0;
//                         right_num = 0;
//
//                     }
//                     else
//                     {
//                         left_num = 0;
//                         right_num = 0;
//                     }
//                 }


}
