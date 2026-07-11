#include "../internal/dependencies/sidelines_dependencies.hpp"

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

