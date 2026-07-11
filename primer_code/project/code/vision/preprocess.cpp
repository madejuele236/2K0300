#include "internal/dependencies/preprocess_dependencies.hpp"
#include <cmath>
uint8_t Threshold_deal(uint8 image[LCDH_1][LCDW_1],uint16 col,uint16 row) {
  #define GrayScale 255
  uint16 width = col;
  uint16 height = row;
  int pixelCount[GrayScale];
  float pixelPro[GrayScale];
  int i, j, pixelSum = width * height;
//  uint8 threshold = 0;
  uint8* data =(uint8*)  image;  //指向像素数据的指针
  for (i = 0; i < GrayScale; i++) {
    pixelCount[i] = 0;
    pixelPro[i] = 0;
  }

  uint32 gray_sum = 0;
  //统计灰度级中 每个像素在整幅图像中的个数
  for (i = 0; i < height; i += 1) {
    for (j = 0; j < width; j += 1) {
      // if((sun_mode&&data[i*width+j]<pixel_threshold)||(!sun_mode))
      //{
      pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
      gray_sum += (int)data[i * width + j];  //灰度值总和
      //}
    }
  }

  //计算每个像素值的点在整幅图像中的比例
  for (i = 0; i < GrayScale; i++) {
    pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }


  //遍历灰度级[0,255]
  float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
  w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
  for (j = 0; j < 180; j++) {
    w0 +=
        pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和 即背景部分的比例
    u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值

    w1 = 1 - w0;
    u1tmp = gray_sum / pixelSum - u0tmp;

    u0 = u0tmp / w0;    //背景平均灰度
    u1 = u1tmp / w1;    //前景平均灰度
    u = u0tmp + u1tmp;  //全局平均灰度
    deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
    if (deltaTmp > deltaMax) {
      deltaMax = deltaTmp;
      Threshold = (uint8)j;
    }
    if (deltaTmp < deltaMax) break;
  }

  return (uint8)Threshold;
}


/******************************************************图像二值化************************************************/


uint8 Threshold_static = 70;
void Get01change_dajin() {
//    if(run_flag==0){
        Threshold = Threshold_deal(Image_Use, LCDW_1, LCDH_1);
//    }
  if (Threshold < Threshold_static)
    Threshold = (uint8)Threshold_static;
  uint8 i, j = 0;
  int thre;
  for (i = 0; i < LCDH_1; i++) {
    for (j = 0; j < LCDW_1; j++) {
      if (j <= 18)
        thre = Threshold - 10;
      else if ((j > 82 && j <= 88))
        thre = Threshold - 10;
      else if (j >= 76)
        thre = Threshold - 10;
      else
        thre = Threshold;

      if (Image_Use[i][j] >(thre))         //数值越大，显示的内容越多，较浅的图像也能显示出来
          Image_Use[i][j] = 255;  //白
      else
          Image_Use[i][j] = 0;  //黑
    }
  }
//     for (i = 50; i < 60; i++) {
//     for (j = LCDW_1/2-15; j < LCDW_1/2+15; j++) {

//           Image_Use[i][j] = 255;  //白

//     }
//   }

    // //图像最小范围 35~60
    // int roi_resize_x = roix1*0.5;
    // int roi_resize_y = roiy1*0.5;
    // int roi_y_2 = roi_resize_y + 25;
    // int roi_x_2 = roi_resize_x + 25;
    // if(roi_resize_x < 35) roi_resize_x = 35;//确保在赛道范围内 同时不要越界
    // if(roi_x_2 > 65) roi_x_2 = 65;

    // if(roi_y_2 > 60) roi_y_2 = 60;
    // if(roi_y_2<0)   roi_y_2 = 0;

    // if(Flag.Redblock == 1)
    // {
    //     for(int i = roi_resize_y;i<roi_y_2;i++)
    //     {
    //         for(int j = roi_resize_x;j<roi_x_2;j++)
    //         {
    //             Image_Use[i][j] = 255;
    //         }
    //     }
    // }

}


void my_sobel(unsigned char imageIn[LCDH_1][LCDW_1], unsigned char imageOut[LCDH_1][LCDW_1])
{
    short KERNEL_SIZE = 3;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = LCDW_1 - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = LCDH_1 - KERNEL_SIZE / 2;
    short i, j;
    short temp[2];
    short temp1 ,temp2 ;
    //for(i = 0; i < Compress_H; i++)//算的更慢不过对比了全局图像
    for (i = yStart; i < yEnd; i++)   //有点的跳跃
       {
           //for(j = 0; j < Compress_W; j++)//算的更慢不过对比了全局图像
           for (j = xStart; j < xEnd; j++)  //有点的跳跃
           {
               /* 计算不同方向梯度幅值  */
               temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
               - (short) 2*imageIn[i][j - 1] + (short) 2*imageIn[i][j + 1]       // {-2, 0, 2},
               - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];    // {-1, 0, 1}};

               temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -2, -1},
               - (short) 2*imageIn[i - 1][j] + (short) 2*imageIn[i + 1][j]       // { 0,  0,  0},
               - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];    // { 1,  2,  1}};

               temp[0] = fabs(temp[0]);
               temp[1] = fabs(temp[1]);

               temp1 = temp[0] + temp[1] ;

               temp2 =  (short) imageIn[i - 1][j - 1] + (short)2* imageIn[i - 1][j] + (short) imageIn[i - 1][j + 1]
                       + (short)2* imageIn[i][j - 1] + (short) imageIn[i][j] + (short) 2*imageIn[i][j + 1]
                       + (short) imageIn[i + 1][j - 1] + (short) 2*imageIn[i + 1][j] + (short) imageIn[i + 1][j + 1];

               if (temp1 > temp2 / 12.0f)
               {
                   imageOut[i][j] = black;
               }
              else
              {
                  imageOut[i][j] = white;
              }
           }
       }

}

/******************************************************图像二值化************************************************/




void my_sobel_dajin(unsigned char imageIn[LCDH_1][LCDW_1], unsigned char imageOut[LCDH_1][LCDW_1])
{

    //    if(run_flag==0){
        Threshold = Threshold_deal(Image_Use, LCDW_1, LCDH_1);
//    }
  if (Threshold < Threshold_static)
    Threshold = (uint8)Threshold_static;

    short KERNEL_SIZE = 3;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = LCDW_1 - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = LCDH_1 - KERNEL_SIZE / 2;
    short i, j;
    short temp[2];
    short temp1 ,temp2 ;
    //for(i = 0; i < Compress_H; i++)//算的更慢不过对比了全局图像
    for (i = yStart; i < yEnd; i++)   //有点的跳跃
       {
           //for(j = 0; j < Compress_W; j++)//算的更慢不过对比了全局图像
           for (j = xStart; j < xEnd; j++)  //有点的跳跃
           {
               /* 计算不同方向梯度幅值  */
               temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
               - (short) 2*imageIn[i][j - 1] + (short) 2*imageIn[i][j + 1]       // {-2, 0, 2},
               - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];    // {-1, 0, 1}};

               temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -2, -1},
               - (short) 2*imageIn[i - 1][j] + (short) 2*imageIn[i + 1][j]       // { 0,  0,  0},
               - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];    // { 1,  2,  1}};

               temp[0] = fabs(temp[0]);
               temp[1] = fabs(temp[1]);

               temp1 = temp[0] + temp[1] ;

            //    temp2 =  (short) imageIn[i - 1][j - 1] + (short)2* imageIn[i - 1][j] + (short) imageIn[i - 1][j + 1]
            //            + (short)2* imageIn[i][j - 1] + (short) imageIn[i][j] + (short) 2*imageIn[i][j + 1]
            //            + (short) imageIn[i + 1][j - 1] + (short) 2*imageIn[i + 1][j] + (short) imageIn[i + 1][j + 1];

               if (temp1 > Threshold*2.0)
               {
                   imageOut[i][j] = 0;
               }
              else
              {
                  imageOut[i][j] = 255;
              }
           }
       }

  int thre;
  for (i = 0; i < LCDH_1; i++) {
    for (j = 0; j < LCDW_1; j++) {
      if (j <= 18)
        thre = Threshold - 10;
      else if ((j > 82 && j <= 88))
        thre = Threshold - 10;
      else if (j >= 76)
        thre = Threshold - 10;
      else
        thre = Threshold;

      if (imageIn[i][j] <(thre))         //数值越大，显示的内容越多，较浅的图像也能显示出来
          imageOut[i][j] = 0;  //hei
    //   else
    //    imageOut[i][j] = 1;  //hei
    }
  }
}


/***************************************************给图像两边画黑框*************************************************/

void Draw_BlackSideline(uint8_t Image_1[LCDH_1][LCDW_1])
{
    int i = 0;
        for(i = imgInfo.bottom - 1; i > imgInfo.top; i--)
        {
            Image_1[i][0] = black;
            Image_1[i][LCDW_1 - 1] = black;
        }
}
