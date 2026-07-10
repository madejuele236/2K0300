#include "internal/vision_stage_contracts.hpp"
#include <cmath>
#include <cstdlib>
/***************************************************获取最长白列***************************************************/

int Mid_Line[LCDH_0] = {LCDW_0 / 2};     // 中线数组，初始化所有元素为屏幕宽度的一半
int Last_Mid_Line[LCDH_0] = {0};           // 上一次中线数组，初始化所有元素为0

/* 获取截止行和最长白列 */
void Get_ImageTop(void)
{

    int8 len;                            // 临时变量，用于存储白色像素列的长度
    uint8 j;                             // 循环变量，用于遍历列
    imgInfo.white_num = 0;               // 初始化最长白色像素列的长度为0
    imgInfo.top = 0;                     // 初始化图像的有效顶部行号为0
    imgInfo.max_column = 0;              // 初始化最长白色像素列的列号为0

    uint8 l_side=0 , r_side=0 ;                    // 默认左边界（对应循环终止条件 i==2）
                                                   // 默认右边界（对应循环终止条件 i==LCDW_1-3）

    static uint8 flag_m = 0;             // 静态标志变量，用于判断是否是第一次执行

    if(flag_m == 0)                      // 如果是第一次执行
    {
        Last_Mid_Line[imgInfo.bottom - 1] = LCDW_1 / 2; // 设置上一次中线位置为屏幕长度的一半
        flag_m = 1;                      // 将标志变量设置为1，表示已执行过一次
    }

//    if(Mode_1)                           // 如果处于模式1
//    {
        imgInfo.bottom = LCDH_1 - 1;     // 设置图像的底部行号为屏幕高度减1

        if(Last_Mid_Line[imgInfo.bottom - 1] > LCDW_1 - 1 || Last_Mid_Line[imgInfo.bottom - 1] < 0)
        {                                // 如果上一次中线位置超出屏幕宽度范围
            Last_Mid_Line[imgInfo.bottom - 1] = LCDW_1 / 2; // 重新设置上一次中线位置为屏幕宽度的一半
        }

        // 从上一次中线位置向左搜索白色区域的左边界
        for(int i = Last_Mid_Line[imgInfo.bottom - 1]; i > 1; i--)
        {
           if((Image_Use[imgInfo.bottom - 1][i] == white
               && Image_Use[imgInfo.bottom - 1][i - 1] == black
               && Image_Use[imgInfo.bottom - 1][i - 2] == black)
                   || (i==2))//寻找到白黑黑跳变点 或者找到最左边（会在图像四周画上图像，所以i=2为最左边）
           {
               l_side = (uint8)i;               // 找到左边界，退出循环
               break;
           }
        }

        // 从上一次中线位置向右搜索白色区域的右边界
        for(int i = Last_Mid_Line[imgInfo.bottom - 1]; i < LCDW_1 - 2; i++)
        {
           if((Image_Use[imgInfo.bottom - 1][i] == white
               && Image_Use[imgInfo.bottom - 1][i + 1] == black
               && Image_Use[imgInfo.bottom - 1][i + 2] == black)
                   || (i==LCDW_1 - 3))
           {
               r_side =(uint8) i;               // 找到右边界，退出循环
               break;
           }
        }
            // 在左右边界之间，每隔4列检查一次最长白色像素列
            for (j = l_side; j <= r_side; j += 1)
            {
                for (len = imgInfo.bottom - 1; len >= imgInfo.top && Image_Use[len][j]; len--);
                                             // 从底部向上搜索，直到找到非白色像素或到达顶部

                len = LCDH_1 - len;          // 计算白色像素列的长度（从底部到顶部的距离）
                if (imgInfo.white_num < len)
                {
                    imgInfo.white_num = len; // 更新最长白色像素列的长度
                    imgInfo.max_column = j;  // 更新最长白色像素列的列号
                }
            }
        // 确定截止行（图像有效部分的顶部）
        if ((LCDH_1 - imgInfo.white_num) >= 0)
            imgInfo.top = LCDH_1 - imgInfo.white_num + 1; // 根据最长白色像素列的长度计算截止行
        else
            imgInfo.top = 0;             // 如果计算出的截止行小于0，则设置为0
//    }
}

int Find_Top(int line)
{
    for(int i = imgInfo.bottom - 1; i <= imgInfo.top; i--)
    {
        if(Image_Use[i - 1][line] == white && Image_Use[i][line] == black && Image_Use[i + 1][line] == black)
        {
            return i;
        }
    }
    return 0;
}


/**
 * @name: xielv_sideline
 * @details: 计算斜率
 * @param {uint8 x1, uint8 y1    第一个点的坐标
 *         uint8 x2, uint8 y2    第二个点的坐标
 *         uint8 data            返回的值 斜率 k 或 常数 b，只能给大小写的k或b，给其他值则返回k
 *          }
 * @return: 返回的值 斜率 k 或 常数 b
 **/
float xielv_sideline(int x1, int y1, int x2, int y2, char data)
{
    float k = 0.0, b = 0.0;

    k = (float)(y2 - y1) / (x2 - x1);
    b = (float)(y1 - k * x1);

    if (data == 'b' || data == 'B')
        return b;
    else if (data == 'k' || data == 'K')
        return k;
    else
        return k;
}

int Left_Sideline[LCDH_0] = {0};    //左边线数组
unsigned char Left_Sideline_flag[LCDH_0] = {0};   //左边线标志位

int Right_Sideline[LCDH_0] = {LCDH_0 - 2};   //右边线数组
unsigned char Right_Sideline_flag[LCDH_0] = {0};  //右边线标志位

unsigned char white_width[LCDH_0] = {0}; //每行的白点数（宽度）
unsigned char starith_white_width[LCDH_0] = {0}; //每行的白点数（宽度）
/* 寻找边线 */

void Find_Sideline(uint8 Start_row, uint8 End_row)
{
    int i = 0, j = 0;
    imgInfo.L_loselineSum = 0; // 初始化左侧丢失线（未检测到边线）的计数器为0

//    else if(Mode_1) // 如果处于模式1
//    {
        for(i = Start_row; i >= End_row; i--) // 从起始行向结束行遍历（逆序）
        {
            Left_Sideline_flag[i] = 0; // 初始化当前行的左侧边线标志为0（未检测到）
            Right_Sideline_flag[i] = 0; // 初始化当前行的右侧边线标志为0（未检测到）

            if(Image_Use[i][imgInfo.max_column] == black) // 如果最长白列的当前行像素为黑色（可能是阴影或障碍物）
            {
                Left_Sideline[i] = Left_Sideline[i + 1]; // 使用上一行的左侧边线位置
                Right_Sideline[i] = Right_Sideline[i + 1]; // 使用上一行的右侧边线位置
                break; // 跳出循环，因为当前行被认为是无效的
            }

            // 左边线检测 从最长白列往左找线
            for(j = imgInfo.max_column; j > 1; j--)
            {
                /* 正常白黑黑跳变点 或者白黑白但距离上一次记录的左侧边线位置小于3列，则认为找到了左侧边线 因为用的压缩图像 可能会有噪点*///
                if((Image_Use[i][j] == white && Image_Use[i][j - 1] == black && Image_Use[i][j - 2] == black)//&& (i<= (imgInfo.top+10)||(i> (imgInfo.top+10)&&(j - Left_Sideline[i+1] < 30)))
                    || (Image_Use[i][j] == white && Image_Use[i][j - 1] == black && Image_Use[i][j - 2] == white && j - Left_Sideline[i+1] < 3))//
                {
                    Left_Sideline[i] = j; // 记录左侧边线的列号
                    Left_Sideline_flag[i] = 1; // 设置左侧边线标志为1（已检测到）
                    break; // 找到左侧边线后退出循环
                }
            }
            if(j == 1 && Left_Sideline_flag[i] == 0) // 如果遍历到第一列仍未检测到左侧边线
            {
                Left_Sideline[i] = 1; // 将左侧边线设置为第一列
                Left_Sideline_flag[i] = 0; // 左侧边线标志保持为0（未真实检测到）
                imgInfo.L_loselineSum ++; // 左侧丢失线计数器加1
            }

            for(j = imgInfo.max_column; j < LCDW_1 - 2; j++)//
            {
                if((Image_Use[i][j] == white && Image_Use[i][j + 1] == black && Image_Use[i][j + 2] == black)//&& (i<=(imgInfo.top+10)||(i> (imgInfo.top+10)&&(Right_Sideline[i + 1] - j < 30)))
                    || (Image_Use[i][j] == white && Image_Use[i][j + 1] == black && Image_Use[i][j + 2] == white && Right_Sideline[i + 1] - j < 3))
                {
                    Right_Sideline[i] = j; // 记录右侧边线的列号
                    Right_Sideline_flag[i] = 1; // 设置右侧边线标志为1（已检测到）
                    break; // 找到右侧边线后退出循环
                }
            }
            if(j == LCDW_1 - 2 && Right_Sideline_flag[i] == 0) // 如果遍历到最后一列前两列仍未检测到右侧边线
            {
                Right_Sideline[i] = LCDW_1 - 2; // 将右侧边线设置为最后一列前两列（通常是屏幕边界前的安全位置）
                Right_Sideline_flag[i] = 0; // 右侧边线标志保持为0（未真实检测到）
                imgInfo.R_loselineSum ++; // 右侧丢失线计数器加1
            }

            white_width[i] = Right_Sideline[i] - Left_Sideline[i]; // 计算并记录当前行的白色区域宽度
        }
      //  printf("Before Avoid: %d\n", Left_Sideline[40]);
//    }
}

void Find_left_Sideline(uint8 Start_row, uint8 End_row)
{
    imgInfo.L_loselineSum = 0; // 初始化左侧丢失线（未检测到边线）的计数器为0
    int i = 0, j = 0;

        for(i = Start_row; i >= End_row; i--) // 从起始行向结束行遍历（逆序）
        {
                       if(Image_Use[i][Left_Sideline[i+1]+7] == white)
            {
            Left_Sideline_flag[i] = 0; // 初始化当前行的左侧边线标志为0（未检测到）

            // if(Image_Use[i][max_column] == black) // 如果最长白列的当前行像素为黑色（可能是阴影或障碍物）
            // {
            //     Left_Sideline[i] = Left_Sideline[i + 1]; // 使用上一行的左侧边线位置
            //     break; // 跳出循环，因为当前行被认为是无效的
            // }

            // 左边线检测 从最长白列往左找线

            for(j = Left_Sideline[i+1]+7; j > 1; j--)
            {
                /* 正常白黑黑跳变点 或者白黑白但距离上一次记录的左侧边线位置小于3列，则认为找到了左侧边线 因为用的压缩图像 可能会有噪点*///
                if((Image_Use[i][j] == white && Image_Use[i][j - 1] == black && Image_Use[i][j - 2] == black)//&& (i<= (imgInfo.top+10)||(i> (imgInfo.top+10)&&(j - Left_Sideline[i+1] < 30)))
                    || (Image_Use[i][j] == white && Image_Use[i][j - 1] == black && Image_Use[i][j - 2] == white && j - Left_Sideline[i+1] < 3))//
                {
                    Left_Sideline[i] = j; // 记录左侧边线的列号
                    Left_Sideline_flag[i] = 1; // 设置左侧边线标志为1（已检测到）
                    break; // 找到左侧边线后退出循环
                }
            }
            if(j == 1 && Left_Sideline_flag[i] == 0) // 如果遍历到第一列仍未检测到左侧边线
            {
                Left_Sideline[i] = 1; // 将左侧边线设置为第一列
                Left_Sideline_flag[i] = 0; // 左侧边线标志保持为0（未真实检测到）
            }
           }

        }
        for(int i=imgInfo.bottom-1;i>=(imgInfo.top+ 1);i--)
        {
            if( Left_Sideline_flag[i] ==0)
              imgInfo.L_loselineSum ++; // 左侧丢失线计数器加1
             white_width[i] = Right_Sideline[i] - Left_Sideline[i]; // 计算并记录当前行的白色区域宽度
        }
}

void Find_right_Sideline(uint8 Start_row, uint8 End_row)
{
    imgInfo.R_loselineSum = 0; // 初始化右侧丢失线计数器为0
    int i = 0, j = 0;

    for(i = Start_row; i >= End_row; i--) // 从起始行向结束行逆序遍历
    {
        // ===================== 修复 1：防止 j 为负数（最关键！）=====================
        int start_j = Right_Sideline[i+1] - 7;
        if (start_j < 0) start_j = 0;  // 强制 >=0，不越界

        if(Image_Use[i][start_j] == white)
        {
            Right_Sideline_flag[i] = 0;

            // ===================== 修复 2：安全起始点，不会负数 =====================
            for(j = start_j; j < LCDW_1 - 2; j++)
            {
                // ===================== 修复 3：右边线正确跳变判断 =====================
                if((Image_Use[i][j] == white && Image_Use[i][j + 1] == black && Image_Use[i][j + 2] == black)
                    || (Image_Use[i][j] == white && Image_Use[i][j + 1] == black && Image_Use[i][j + 2] == white && Right_Sideline[i + 1] - j < 3))
                {
                    Right_Sideline[i] = j;
                    Right_Sideline_flag[i] = 1;
                    break;
                }
            }

            // ===================== 修复 4：统一的未检测到边界处理 =====================
            if(j >= LCDW_1 - 2 && Right_Sideline_flag[i] == 0)
            {
                Right_Sideline[i] = LCDW_1 - 2;
                Right_Sideline_flag[i] = 0;
            }
        }
    }

    // ===================== 修复 5：丢行统计 移到外面（和左边完全一致）=====================
    for(int i = imgInfo.bottom - 1; i >= imgInfo.top + 1; i--)
    {
        if(Right_Sideline_flag[i] == 0)
            imgInfo.R_loselineSum ++;

        white_width[i] = Right_Sideline[i] - Left_Sideline[i];
    }
}

struct Guaidian L_l_guai, L_h_guai, R_l_guai, R_h_guai;  //拐点信息结构体
int Guai_row = 0;

void Find_Guaidian(void)
{
    L_l_guai.flag = 0;
    L_h_guai.flag = 0;
    R_l_guai.flag = 0;
    R_h_guai.flag = 0;


    if(L_l_guai.flag == 0)
        L_l_guai.row = LCDH_1 - 1;
    if(R_l_guai.flag == 0)
        R_l_guai.row = LCDH_1 - 1;

    //找左上，右上拐点
    for(int i = imgInfo.top + 2; i <= imgInfo.bottom - 5; i++)
    {

        /**************左上拐点**************/
        if(
             Left_Sideline[i] > 3 &&
             (
                  /*    *0
                      0100
                           */
                 (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0]
                  && Image_Use[i + 1][Left_Sideline[i] - 1] && !Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0000
                           */
                  (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                   && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && !Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      00010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0]&& Image_Use[i + 1][Left_Sideline[i] + 1] && !Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
//                   ||/*    *0
//                          01000
//                            */
// (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0]
// && Image_Use[i + 1][Left_Sideline[i] - 1] && !Image_Use[i + 1][Left_Sideline[i] - 3]
// && Image_Use[i + 1][Left_Sideline[i] - 2])

             )
             && white_width[i + 2] - white_width[i - 0] > 5
             && xielv_sideline(i, Left_Sideline[i], i + 1, Left_Sideline[i + 1], 'k') < 1
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Left_Sideline[i - 1] - Left_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Left_Sideline[i], i - 3, Left_Sideline[i - 3], 'k') - xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k')) > 0.5
//             && xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k') < 1
             && L_h_guai.flag == 0
             && i < L_l_guai.row
        )
        {
            L_h_guai.row = i + 1;
            L_h_guai.column = (uint8)Left_Sideline[i];
            L_h_guai.flag = 1;

//            if(R_h_guai.flag)
//                break;

        }

        /**************右上拐点**************/
        if(
             Right_Sideline[i] < LCDW_1 - 3 &&
             (
                   /*  0*
                       0010
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && !Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                       0000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                       0100
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && !Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                      01000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 1] && !Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
//                    ||/*    0*
//                          00010
//                     */
// (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0]
// && Image_Use[i + 1][Right_Sideline[i] + 1] && !Image_Use[i + 1][Right_Sideline[i] + 3]
// && Image_Use[i + 1][Right_Sideline[i] + 2])

             )
             && white_width[i + 2] - white_width[i - 0] > 5
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Right_Sideline[i - 1] - Right_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') - xielv_sideline(i, Right_Sideline[i], i + 2, Right_Sideline[i + 2], 'k')) > 0.5
             && xielv_sideline(i, Right_Sideline[i], i + 1, Right_Sideline[i + 1], 'k') > -1
             && R_h_guai.flag == 0
             && i < R_l_guai.row
           )
        {
            R_h_guai.row = i + 1;
            R_h_guai.column = (uint8)Right_Sideline[i];
            R_h_guai.flag = 1;

//            if(L_h_guai.flag)
//                break;
        }

    }


//    else if(Mode_1)
//    {
        //找左下，右下拐点
        for(int i = imgInfo.bottom - 3; i >= imgInfo.top + 3 && Flag.Huandao_L != 3 && Flag.Huandao_R != 3; i--)
        {
            if(L_h_guai.flag == 0)
                L_h_guai.row = 1;
            if(R_h_guai.flag == 0)
                R_h_guai.row = 1;
            /**************左下拐点**************/
            if(
                 Left_Sideline[i] > 3 &&
                 (
                      /*  0000
                            *0
                                */
                     (Image_Use[i][Left_Sideline[i] +0] && Image_Use[i - 1][Left_Sideline[i] + 0]
                      && Image_Use[i - 1][Left_Sideline[i] - 1] && Image_Use[i - 1][Left_Sideline[i] - 2]  && Image_Use[i - 1][Left_Sideline[i] -3])
                     ||
                     /*  0100
                           *0
                               */
                     (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] - 1]
                      && !Image_Use[i - 1][Left_Sideline[i] - 2] && Image_Use[i - 1][Left_Sideline[i] - 3])
                      ||
                      /*  0010
                            *0
                                */
                      (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] + 0] && !Image_Use[i - 1][Left_Sideline[i] - 1]
                       && Image_Use[i - 1][Left_Sideline[i] - 2] && Image_Use[i - 1][Left_Sideline[i] - 3])
                 )
                 /*上面三行的白行都比这一行多*/
                 && white_width[i - 2] - white_width[i + 1] > 10
                 && white_width[i - 2] > white_width[i] && white_width[i - 3] > white_width[i] && white_width[i - 4] > white_width[i]
                 && i > L_h_guai.row
                 && L_l_guai.flag == 0
            )
            {
                L_l_guai.row = i - 1;
                L_l_guai.column = (uint8)Left_Sideline[i];
                L_l_guai.flag = 1;

//                if(R_l_guai.flag)
//                    break;

            }

            /**************右下拐点**************/
            if(
                 Right_Sideline[i] < LCDW_1 - 3 &&
                 (
                         /*
                              0000
                              0*
                          */
                         (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0]
                         && Image_Use[i - 1][Right_Sideline[i] + 1] && Image_Use[i - 1][Right_Sideline[i] + 2] && Image_Use[i - 1][Right_Sideline[i] + 3])
                         ||
                         /*
                              0010
                              0*
                          */
                         (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] + 1]
                          && !Image_Use[i - 1][Right_Sideline[i] + 2] && Image_Use[i - 1][Right_Sideline[i] + 3])
                          ||
                          /*
                               0001
                               0*
                           */
                          (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] + 1]
                           && Image_Use[i - 1][Right_Sideline[i] + 2] && !Image_Use[i - 1][Right_Sideline[i] + 3])
                 )
                 /*上面三行的白行都比这一行多*/
                 && white_width[i - 2] > white_width[i] && white_width[i - 3] > white_width[i]
                 && white_width[i - 2] - white_width[i + 1] > 10
//                 && abs(xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') - xielv_sideline(i, Right_Sideline[i], i + 2, Right_Sideline[i + 2], 'k')) > 0.5
//                 && xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') > 1
                 && i > R_h_guai.row
                 && R_l_guai.flag == 0
            )
            {
                R_l_guai.row = i - 1;
                R_l_guai.column = (uint8)Right_Sideline[i];
                R_l_guai.flag = 1;

//                if(L_l_guai.flag)
//                    break;
            }
        }

//    }
int l_guai_max=0,r_guai_max=93;//,l_guai_line,r_guai_line
if(Flag.Huandao_R == 4)
{
    for(int i = imgInfo.top + 2; i <54; i++)
    {
            if(Left_Sideline[i]> l_guai_max)//
            {
                l_guai_max = Left_Sideline[i];
                L_l_guai.row = i ;
            }
    }

                L_l_guai.column = (uint8)Left_Sideline[L_l_guai.row]+3;
                L_l_guai.flag = 1;
}
if(Flag.Huandao_L == 4)
{
    for(int i = imgInfo.top + 2; i <54; i++)
    {
            if(Right_Sideline[i]< r_guai_max)//
            {
                r_guai_max = Right_Sideline[i];
                R_l_guai.row = i ;
            }
    }

                R_l_guai.column = (uint8)Right_Sideline[R_l_guai.row]-3;
                R_l_guai.flag = 1;
}
        // for(int i = imgInfo.bottom - 3; i >= imgInfo.top + 3 && Flag.Huandao_L != 3 && Flag.Huandao_R != 3; i--)



}


void Find_Guaidian1(void)
{
                        // printf("Fla:%d\n", white_width[37 + 2] - white_width[37 - 0]);
    L_l_guai.flag = 0;
    L_h_guai.flag = 0;
    R_l_guai.flag = 0;
    R_h_guai.flag = 0;


    if(L_l_guai.flag == 0)
        L_l_guai.row = LCDH_1 - 1;
    if(R_l_guai.flag == 0)
        R_l_guai.row = LCDH_1 - 1;

    //找左上，右上拐点
    for(int i = imgInfo.top + 2; i <= imgInfo.bottom - 5; i++)
    {

        /**************左上拐点**************/
        if(
             Left_Sideline[i] > 3 &&
             (
                  /*    *0
                      0100
                           */
                 (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0]
                  && Image_Use[i + 1][Left_Sideline[i] - 1] && !Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0000
                           */
                  (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                   && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && !Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      00010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0]&& Image_Use[i + 1][Left_Sideline[i] + 1] && !Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
//                   ||/*    *0
//                          01000
//                            */
// (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0]
// && Image_Use[i + 1][Left_Sideline[i] - 1] && !Image_Use[i + 1][Left_Sideline[i] - 3]
// && Image_Use[i + 1][Left_Sideline[i] - 2])

             )
             && white_width[i + 2] - white_width[i - 0] > 5
             && xielv_sideline(i, Left_Sideline[i], i + 1, Left_Sideline[i + 1], 'k') < 1
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Left_Sideline[i - 1] - Left_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Left_Sideline[i], i - 3, Left_Sideline[i - 3], 'k') - xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k')) > 0.5
//             && xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k') < 1
            //  && L_h_guai.flag == 0
            //  && i < L_l_guai.row
        )
        {
            L_h_guai.row = i + 1;
            L_h_guai.column = (uint8)Left_Sideline[i];
            L_h_guai.flag = 1;

//            if(R_h_guai.flag)
//                break;

        }

        /**************右上拐点**************/
        if(
             Right_Sideline[i] < LCDW_1 - 3 &&
             (
                   /*  0*
                       0010
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && !Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                       0000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                       0100
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && !Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                      01000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 1] && !Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
//                    ||/*    0*
//                          00010
//                     */
// (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0]
// && Image_Use[i + 1][Right_Sideline[i] + 1] && !Image_Use[i + 1][Right_Sideline[i] + 3]
// && Image_Use[i + 1][Right_Sideline[i] + 2])

             )
             && white_width[i + 2] - white_width[i - 0] > 5
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Right_Sideline[i - 1] - Right_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') - xielv_sideline(i, Right_Sideline[i], i + 2, Right_Sideline[i + 2], 'k')) > 0.5
             && xielv_sideline(i, Right_Sideline[i], i + 1, Right_Sideline[i + 1], 'k') > -1
            //  && R_h_guai.flag == 0
            //  && i < R_l_guai.row
           )
        {
            R_h_guai.row = i + 1;
            R_h_guai.column = (uint8)Right_Sideline[i];
            R_h_guai.flag = 1;

//            if(L_h_guai.flag)
//                break;
        }

    }


//    else if(Mode_1)
//    {
        //找左下，右下拐点
        for(int i = imgInfo.bottom - 3; i >= imgInfo.top + 3 && Flag.Huandao_L != 3 && Flag.Huandao_R != 3; i--)
        {
            if(L_h_guai.flag == 0)
                L_h_guai.row = 1;
            if(R_h_guai.flag == 0)
                R_h_guai.row = 1;
            /**************左下拐点**************/
            if(
                 Left_Sideline[i] > 3 &&
                 (
                      /*  0000
                            *0
                                */
                     (Image_Use[i][Left_Sideline[i] +0] && Image_Use[i - 1][Left_Sideline[i] + 0]
                      && Image_Use[i - 1][Left_Sideline[i] - 1] && Image_Use[i - 1][Left_Sideline[i] - 2]  && Image_Use[i - 1][Left_Sideline[i] -3])
                     ||
                     /*  0100
                           *0
                               */
                     (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] - 1]
                      && !Image_Use[i - 1][Left_Sideline[i] - 2] && Image_Use[i - 1][Left_Sideline[i] - 3])
                      ||
                      /*  0010
                            *0
                                */
                      (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] + 0] && !Image_Use[i - 1][Left_Sideline[i] - 1]
                       && Image_Use[i - 1][Left_Sideline[i] - 2] && Image_Use[i - 1][Left_Sideline[i] - 3])
                 )
                 /*上面三行的白行都比这一行多*/
                 && white_width[i - 2] - white_width[i + 1] > 10
                 && white_width[i - 2] > white_width[i] && white_width[i - 3] > white_width[i] && white_width[i - 4] > white_width[i]
                 && i > L_h_guai.row
                 && L_l_guai.flag == 0
            )
            {
                L_l_guai.row = i - 1;
                L_l_guai.column = (uint8)Left_Sideline[i];
                L_l_guai.flag = 1;

//                if(R_l_guai.flag)
//                    break;

            }

            /**************右下拐点**************/
            if(
                 Right_Sideline[i] < LCDW_1 - 3 &&
                 (
                         /*
                              0000
                              0*
                          */
                         (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0]
                         && Image_Use[i - 1][Right_Sideline[i] + 1] && Image_Use[i - 1][Right_Sideline[i] + 2] && Image_Use[i - 1][Right_Sideline[i] + 3])
                         ||
                         /*
                              0010
                              0*
                          */
                         (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] + 1]
                          && !Image_Use[i - 1][Right_Sideline[i] + 2] && Image_Use[i - 1][Right_Sideline[i] + 3])
                          ||
                          /*
                               0001
                               0*
                           */
                          (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] + 1]
                           && Image_Use[i - 1][Right_Sideline[i] + 2] && !Image_Use[i - 1][Right_Sideline[i] + 3])
                 )
                 /*上面三行的白行都比这一行多*/
                 && white_width[i - 2] > white_width[i] && white_width[i - 3] > white_width[i]
                 && white_width[i - 2] - white_width[i + 1] > 10
//                 && abs(xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') - xielv_sideline(i, Right_Sideline[i], i + 2, Right_Sideline[i + 2], 'k')) > 0.5
//                 && xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') > 1
                 && i > R_h_guai.row
                 && R_l_guai.flag == 0
            )
            {
                R_l_guai.row = i - 1;
                R_l_guai.column = (uint8)Right_Sideline[i];
                R_l_guai.flag = 1;

//                if(L_l_guai.flag)
//                    break;
            }
        }

//    }
int l_guai_max=0,r_guai_max=93;//,l_guai_line,r_guai_line
if(Flag.Huandao_R == 4)
{
    for(int i = imgInfo.top + 2; i <54; i++)
    {
            if(Left_Sideline[i]> l_guai_max)//
            {
                l_guai_max = Left_Sideline[i];
                L_l_guai.row = i ;
            }
    }

                L_l_guai.column = (uint8)Left_Sideline[L_l_guai.row]+3;
                L_l_guai.flag = 1;
}
if(Flag.Huandao_L == 4)
{
    for(int i = imgInfo.top + 2; i <54; i++)
    {
            if(Right_Sideline[i]< r_guai_max)//
            {
                r_guai_max = Right_Sideline[i];
                R_l_guai.row = i ;
            }
    }

                R_l_guai.column = (uint8)Right_Sideline[R_l_guai.row]-3;
                R_l_guai.flag = 1;
}
        // for(int i = imgInfo.bottom - 3; i >= imgInfo.top + 3 && Flag.Huandao_L != 3 && Flag.Huandao_R != 3; i--)



}


struct Guaidian  L_h_guai1,  R_h_guai1;  //拐点信息结构体

void Find_l_h_Guaidian(void)
{
    L_h_guai1.flag = 0;
    R_h_guai1.flag = 0;
            L_h_guai1.row = 0;
            // L_h_guai1.column =0;
    //找左上，右上拐点
    for(int i = imgInfo.top + 2; i <= imgInfo.bottom - 5; i++)
    {

        /**************左上拐点**************/
        if(
             Left_Sideline[i] > 3 &&
             (
                  /*    *0
                      0100
                           */
                 (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0]
                  && Image_Use[i + 1][Left_Sideline[i] - 1] && !Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0000
                           */
                  (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                   && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && !Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      00010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0]&& Image_Use[i + 1][Left_Sideline[i] + 1] && !Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
             )
             && white_width[i + 2] - white_width[i - 0] > 9
             && xielv_sideline(i, Left_Sideline[i], i + 1, Left_Sideline[i + 1], 'k') < 1
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Left_Sideline[i - 1] - Left_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Left_Sideline[i], i - 3, Left_Sideline[i - 3], 'k') - xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k')) > 0.5
//             && xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k') < 1
             && L_h_guai1.flag == 0
             && i < L_l_guai.row
        )
        {
            L_h_guai1.row = i + 1;
            L_h_guai1.column = (uint8)Left_Sideline[i];
            L_h_guai1.flag = 1;


        }


    }
}


void Find_r_h_Guaidian(void)
{
    R_h_guai1.flag = 0;
    L_h_guai1.flag = 0;
            R_h_guai1.row = 0;
            // R_h_guai1.column =0;
    //找左上，右上拐点
    for(int i = imgInfo.top + 2; i <= imgInfo.bottom - 5; i++)
    {
        /**************右上拐点**************/
        if(
             Right_Sideline[i] < LCDW_1 - 3 &&
             (
                   /*  0*
                       0010
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && !Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                       0000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                       0100
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && !Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                      01000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 1] && !Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
             )
             && white_width[i + 2] - white_width[i - 0] > 9
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Right_Sideline[i - 1] - Right_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') - xielv_sideline(i, Right_Sideline[i], i + 2, Right_Sideline[i + 2], 'k')) > 0.5
             && xielv_sideline(i, Right_Sideline[i], i + 1, Right_Sideline[i + 1], 'k') > -1
             && R_h_guai1.flag == 0
             && i < R_l_guai.row
           )
        {
            R_h_guai1.row = i + 1;
            R_h_guai1.column = (uint8)Right_Sideline[i];
            R_h_guai1.flag = 1;

//            if(L_h_guai.flag)
//                break;
        }


    }
}

/***************************************************找中线**********************************************************/
void Find_Midline(){
//   if(imgInfo.L_loselineSum==0&&imgInfo.R_loselineSum==0){

    for(uint16 i = imgInfo.bottom-1;i>imgInfo.top;i--){
      Mid_Line[i] = (Left_Sideline[i] + Right_Sideline[i])/2;
    }
//   }
        for(uint16 i = imgInfo.bottom - 1; i > imgInfo.top; i--)
        {
            Last_Mid_Line[i] = Mid_Line[i];
        }
}


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


/***************************************************线性回归计算中线斜率**********************************************/
float B,A;
void regression(int startline,int endline)
{

  int i=0,SumX=0,SumY=0,SumLines = 0;
  float SumUp=0,SumDown=0,avrX=0,avrY=0;
  SumLines=endline-startline;   // startline 为开始行， //endline 结束行 //SumLines

  for(i=startline;i<endline;i++)
  {
    SumX+=Mid_Line[i];//列号（X坐标）
    SumY+=i;    //行号（Y坐标）
  }
  avrX=(float)SumX/SumLines;     //X平均值
  avrY=(float)SumY/SumLines;     //Y平均值
  SumUp=0;
  SumDown=0;
  for(i=startline;i<endline;i++)
  {
    SumUp+=(Mid_Line[i]-avrX)*(i-avrY);
    SumDown+=(i-avrY)*(i-avrY);
  }
  if(SumDown==0)
    B=0;
  else{
    B=(float)(SumUp/SumDown);
    A=avrX - B*avrY;  //截距
  }

}
/***************************************************曲率计算********************************************************/
float curvature;
void calculateCurvature(float x1,float y1,float x2,float y2,float x3,float y3){

  // 计算三角形的边长
  float a = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
  float b = sqrt(pow(x3 - x2, 2) + pow(y3 - y2, 2));
  float c = sqrt(pow(x3 - x1, 2) + pow(y3 - y1, 2));

  // 计算三角形的半周长
  float s = (a + b + c) / 2;

  // 计算三角形的面积（使用海伦公式）
  float area = sqrt(s * (s - a) * (s - b) * (s - c));

  // 计算外接圆半径
  float radius = (a * b * c) / (4 * area);

  // 计算曲率（曲率是半径的倒数）
  curvature =(float) 1 / radius*100;

}
