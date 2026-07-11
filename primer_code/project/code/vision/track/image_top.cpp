#include "../internal/dependencies/image_top_dependencies.hpp"
/***************************************************获取最长白列***************************************************/

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
