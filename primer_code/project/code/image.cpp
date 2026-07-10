#include "zf_common_headfile.hpp"
#include "ww_transmission.h"
#define UVC_PATH       "/dev/video0"  
#define MODEL_INPUT_WIDTH         40      // 模型输入图像宽度（根据实际模型调整）
#define MODEL_INPUT_HEIGHT        40      // 模型输入图像高度（根据实际模型调整）
#define MODEL_INPUT_CHANNEL       3       // 模型输入图像通道数（RGB=3）
#define MODEL_OUTPUT_CLASS_NUM    3       // 模型输出类别数（根据实际任务调整）
#define MODEL_INPUT_SIZE          (MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT * MODEL_INPUT_CHANNEL)  // 输入总元素数
#define TFLITE_OP_RESOLVER_MAX_NUM 20     // 算子解析器最大支持算子数
#define TENSOR_ARENA_SIZE         (128 * 1024)  // 张量空间大小（单位：字节，根据模型大小调整）
static uint8_t tensor_arena[TENSOR_ARENA_SIZE];
static tflite::MicroInterpreter* interpreter = nullptr;
static tflite::MicroMutableOpResolver<20> resolver;
const char* class_labels[] = {"materials","traffic","weapon"}; //需要与train.py提示顺序一致
zf_device_uvc uvc_dev;//初始化摄像头对象
uint8_t Image_Zip[LCDH_1][LCDW_1] = {0}; //压缩后的图像数组
uint8_t Image_Use[LCDH_1][LCDW_1] = {0};//(经过大津法，膨胀，腐蚀后的)二值化的图像
cv::VideoCapture cap;
unsigned char Image_IFS[LCDH_1][LCDW_1];
LQ_NCNN classifier;
uint8_t Threshold = 0;  //大津法求出的阈值
struct imageInformation imgInfo;
struct YuanSu Flag;
cv::Mat frame, grayFrame, binaryFrame,resizedFrame,flippedFrame,translatedFrame,translationMatrix,float_img,resized_img;
TransmissionStreamServer camera_server;
int roi_x_1= 0;
int roi_y_1 = 0;
int ROI_SIZE = 60;
char txt[80];
bool zf_init_flag = false;
bool first_frame_logged = false;
#define ENABLE_FIRST_FRAME_DEBUG     true
int picture_cnt = 0;
bool first_flag = true;
int roix1,roiy1 = 0;
using namespace cv;
cv::Mat lq_frame;
//绕行相关
bool picture_yaw_init = false;//记录第一次进入绕行逻辑的标志位
float Yaw_picture = 0;//记录检测到图片时的初始yaw值
float Yaw_picture_diff = 0;//当前的YAW值相较于Yaw_picture的差值
float Yaw_picture_err = 0;
float Yaw_picture_target = 0;//绕行的目标偏差值
float encoder_val = 0;//用于记录编码器的值 以实现分阶段运行
int resize_cx,resize_cy = 0;
//处理陀螺仪角度跳变
float Yaw_correct(float current_yaw,float target_yaw)
{
    float diff = current_yaw - target_yaw;
    if(diff >=180)
    {
        diff =  diff - 360;
    }
    if (diff<=-180)
    {
        diff =  diff + 360;
    }
    return diff;
    
}

const float atan_deg_tab[49] = 
{
    0.00f, 45.00f, 63.43f, 71.57f, 75.96f, 78.69f, 80.54f, 81.87f, 82.87f, 83.66f,
    84.29f, 84.81f, 85.24f, 85.60f, 85.91f, 86.19f, 86.42f, 86.63f, 86.82f, 86.99f,
    87.14f, 87.27f, 87.40f, 87.51f, 87.61f, 87.71f, 87.80f, 87.88f, 87.95f, 88.03f,
    88.09f, 88.15f, 88.21f, 88.26f, 88.32f, 88.36f, 88.41f, 88.45f, 88.49f, 88.53f,
    88.57f, 88.60f, 88.64f, 88.67f, 88.70f, 88.73f, 88.75f, 88.78f, 88.81f
};
float actan_err(float err)
{
    float actan_err;
    if(err>0) actan_err=atan_deg_tab[(int)err];
    if(err<=0) actan_err=-atan_deg_tab[-(int)err];
  return actan_err;
}
// float real_distance[60] =
// {
//      536.00, 413.00, 330.50, 276.40, 235.00, 205.80, 179.20, 156.40, 142.40, 132.40,
//      118.20, 107.40, 99.00, 91.60, 83.50, 78.60, 73.70,  68.00,  62.40,  56.60,
//      54.80,  51.00,  47.00,  44.00,  42.00,  39.00,  36.20,  34.00,  31.50,  30.40,
//      28.20,  26.60,  24.80,  23.20,  21.90,  20.60,  19.40,  18.40,  17.40,  16.10,
//      15.20,  14.20,  13.40,  12.80,  12.00,  11.20,  10.60,  9.80,  8.80,  8.00, 
//      7.20,  6.70,  6.20,  5.40,  4.60,  4.00,  3.50,  2.80,  2.00,  1.20,
// };

// float real_distance[60] =
// {
//      750.00-3,590.00-3, 460.00-3, 366.00-3, 301.50-3, 262.00-3, 226.00-3, 199.00-3, 177.00-3, 160.0-3,
//      144.00-3, 132.00-3, 122.00-3, 113.0-30, 103.90-3, 97.00-3, 90.00-3,  82.50-3,  78.00-3,  73.00-3,
//      67.50-3,  64.00-3,  60.30-3,  55.80-3,  53.40-3,  49.90-3,  46.00-3,  44.30-3,  41.90-3,  38.90-3,
//      37.10-3,  35.50-3,  33.00-3,  31.80-3,  30.10-3,  28.00-3,  26.60-3,  25.40-3,  23.00-3,  22.10-3,
//      21.20-3,  19.80-3,  18.80-3,  18.00-3,  16.00-3,  15.20-3,  14.50-3,  13.00-3,  12.80-3,  12.00-3, 
//      10.70-3,  9.60-3,  9.10-3,  8.80-3,  8.00-3,  7.10-3,  6.50-3,  6.00-3,  5.00-3,  3.80-3,
// };

// float real_distance[60] =
// {
//      500,410, 350, 300.0, 268.0, 226.5, 193.0, 172.0, 153.5, 138.6,
//      126.0, 115.0, 106.2, 97.8, 90.6, 84.6, 79.0,  73.8,  68.8,  64.9,
//      60.8,  58.0,  54.0,  51.0,  48.0,  45.7,  42.5,  40.0,  38.0,  36.5,
//      34.5,  32.4,  30.8,  29.5,  28.1,  26.3,  25.0,  24.0,  22.3,  21.2,
//      20.4,  19.2,  18.0,  17.0,  16.00,  15.20,  14.4,  13.6,  13.0,  12.0, 
//      11.2,  10.6,  10.0,  9.5,  8.7,  8.2,  7.8,  7,  6.5,  3.80-3,
// };

float real_distance[60] =
{
     485,410, 345, 292.0, 244.5, 213.8, 184.0, 161.0, 141.5, 127.0,
     115.0, 104.5, 95.6, 88.0, 82.0, 77.0, 71.7,  67.0,  62.5,  58.0,
     54.7,  51.50,  48.2,  45.3,  42.5,  40.3,  38.0,  36.5,  34.6,  32.5,
     31.0,  29.5,  27.2,  26.5,  24.8,  23.3,  22.1,  21.0,  20.0,  19.0,
     18.0,  17.0,  16.0,  14.9,  14.0,  13.10,  12.6,  12.0,  11.0,  10.5, 
     10.0,  9.3,  8.7,  8.2,  7.5,  6.9,  6.5,  6.0,  5.5,  5,
};

int real_distance_to_row(float distance) {
    float min_diff = 10000.0f;  // 初始化为一个很大的数
    int nearest_index = 0;
    
    for (int i = 0; i < 60; i++) {
        float diff = fabsf(real_distance[i] - distance);
        if (diff < min_diff) {
            min_diff = diff;
            nearest_index = i;
        }
    }
    
    return nearest_index;
}

void imgInfoInit(void)
{
    imgInfo.bottom = LCDH_1 - 1;
    imgInfo.lastMid = LCDW_1 / 2;
    imgInfo.top = 0;
    imgInfo.L_loselineSum = 0;
    imgInfo.R_loselineSum = 0;
}

void debug_log_printf_callback(const char* s)
{
    if (s == NULL) return;
    printf("%s", s);
}

void zf_model_init()
{
    RegisterDebugLogCallback(debug_log_printf_callback);

    tflite::InitializeTarget();

    const tflite::Model* model = ::tflite::GetModel(loong_cnn_model_simple_tflite);
    TFLITE_CHECK_EQ(model->version(), TFLITE_SCHEMA_VERSION);

    // 注册算子
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddMaxPool2D();
    resolver.AddFullyConnected();
    resolver.AddRelu6();
    resolver.AddSoftmax();
    resolver.AddReshape();
    resolver.AddShape();
    resolver.AddQuantize();
    resolver.AddDequantize();
    resolver.AddCast();
    resolver.AddSqueeze();
    resolver.AddExpandDims();
    resolver.AddConcatenation();
    resolver.AddTranspose();
    resolver.AddStridedSlice();
    resolver.AddPack();
    resolver.AddLogistic();
    resolver.AddMean();
    resolver.AddAdd();

    // ✅ 正确创建 interpreter
    interpreter = new tflite::MicroInterpreter(
        model, resolver, tensor_arena, TENSOR_ARENA_SIZE);

    TfLiteStatus status = interpreter->AllocateTensors();

    if (status != kTfLiteOk)
    {
        printf("❌ AllocateTensors 失败！\n");
        while (1);
    }

    printf("✅ TFLite 初始化完成\n");
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      优化的大津法
//  @param      image  图像数组
//  @param      clo    宽
//  @param      row    高
//  @param      pixel_threshold 阈值分离
//  @return     uint8
//  @since      2021.6.23
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
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

/* 补线 */
void Buxian(void)
{

  
    Flag.Buxian = 0;
//    else if(Mode_1)
//    {
        //四个拐点都找到
        if(L_l_guai.flag && L_h_guai.flag && R_l_guai.flag && R_h_guai.flag && abs(R_h_guai.column - L_h_guai.column) > 5)//
        {

            float kl, kr, bl, br;

            if(L_h_guai.row < L_l_guai.row)
            {
                kl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'k');
                bl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'b');

                for (int i = L_h_guai.row; i <= L_l_guai.row; i++)
                {
                    Left_Sideline[i] = kl * i + bl;
                    Left_Sideline_flag[i] = 1;
                }

                Flag.Buxian = 1;
            }

            if(R_h_guai.row < R_l_guai.row)
            {
                kr = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'k');
                br = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'b');

                for (int i = R_h_guai.row; i <= R_l_guai.row; i++)
                {
                    Right_Sideline[i] = kr * i + br;
                    Right_Sideline_flag[i] = 1;
                }

                Flag.Buxian = 1;
            }

//            Flag.Shizi=1;
        }

//        if(Flag.Shizi==1)
//        {
//            if(distance>30){Flag.Shizi=0;distance=0;}
        //找到左上，左下，右上
         if(L_l_guai.flag && L_h_guai.flag && R_h_guai.flag && abs(R_h_guai.column - L_h_guai.column) > 5)//
        {
            float kl, kr, bl, br;

            if(L_h_guai.row < L_l_guai.row)
            {
                kl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'k');
                bl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'b');

                for (int i = L_h_guai.row; i <= L_l_guai.row; i++)
                {
                    Left_Sideline[i] = kl * i + bl;
                    Left_Sideline_flag[i] = 1;
                }

                Flag.Buxian = 1;
            }

            if(R_h_guai.row < L_l_guai.row)
            {
                kr = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 5, Right_Sideline[imgInfo.bottom - 5], 'k');
                br = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 5, Right_Sideline[imgInfo.bottom - 5], 'b');

                for (int i = R_h_guai.row; i <= imgInfo.bottom - 5; i++)
                {
                    Right_Sideline[i] = kr * i + br;
                    Right_Sideline_flag[i] = 1;
                    if (
                            (Right_Sideline[i] == Right_Sideline[i + 1] && Right_Sideline[i] == Right_Sideline[i + 2])
                            || !Image_Use[i + 1][(uint8)(kr * (i + 1) + br)]
                        )
                        break;
                }

                Flag.Buxian = 1;
            }
        }


        else if(R_l_guai.flag && R_h_guai.flag && L_h_guai.flag && abs(R_h_guai.column - L_h_guai.column) > 5)//
        {
            float kl, kr, bl, br;

            if(L_h_guai.row < R_l_guai.row)
            {
                kl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 5, Left_Sideline[imgInfo.bottom - 5], 'k');
                bl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 5, Left_Sideline[imgInfo.bottom - 5], 'b');

                for (int i = L_h_guai.row; i <= imgInfo.bottom - 5; i++)
                {
                    Left_Sideline[i] = kl * i + bl;
                    Left_Sideline_flag[i] = 1;
                    if(
                            (Left_Sideline[i] == Left_Sideline[i + 1] && Left_Sideline[i] == Left_Sideline[i + 2])
                            || !Image_Use[i + 1][(uint8)(kl * (i + 1) + bl)]
                        )
                        break;
                }

                Flag.Buxian = 1;
            }

            if(R_h_guai.row < R_l_guai.row)
            {
                kr = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'k');
                br = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'b');

                for (int i = R_h_guai.row; i <= R_l_guai.row; i++)
                {
                    Right_Sideline[i] = kr * i + br;
                    Right_Sideline_flag[i] = 1;
                }

                Flag.Buxian = 1;
            }

        }

        else if(R_h_guai.flag && L_h_guai.flag && abs(R_h_guai.column - L_h_guai.column) > 5)//
        {
            float kl, kr, bl, br;

            kl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 5, Left_Sideline[imgInfo.bottom - 5] - 3, 'k');
            bl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 5, Left_Sideline[imgInfo.bottom - 5] - 3, 'b');

            for (int i = L_h_guai.row; i <= imgInfo.bottom - 5; i++)
            {
                Left_Sideline[i] = kl * i + bl;
                Left_Sideline_flag[i] = 1;
                if(
                        (Left_Sideline[i] == Left_Sideline[i + 1] && Left_Sideline[i] == Left_Sideline[i + 2])
                        || !Image_Use[i + 1][(uint8)(kl * (i + 1) + bl)]
                    )
                    break;
            }

            kr = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 5, Right_Sideline[imgInfo.bottom - 5] - 3, 'k');
            br = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 5, Right_Sideline[imgInfo.bottom - 5] - 3, 'b');

            for (int i = R_h_guai.row; i < imgInfo.bottom - 5; i++)
            {
                Right_Sideline[i] = kr * i + br;
                Right_Sideline_flag[i] = 1;
                if (
                        (Right_Sideline[i] == Right_Sideline[i + 1] && Right_Sideline[i] ==  Right_Sideline[i + 2])
                        || !Image_Use[i + 1][(uint8)(kr * (i + 1) + br)]
                    )
                    break;
            }

            Flag.Buxian = 1;
        }

        else if(L_l_guai.flag && L_h_guai.flag)
        {
            float kl, bl;
            if(L_h_guai.row < L_l_guai.row)
            {
                kl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'k');
                bl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'b');

                for (int i = L_h_guai.row; i <= L_l_guai.row; i++)
                {
                    Left_Sideline[i] = kl * i + bl;
                    Left_Sideline_flag[i] = 1;
                }
            }

            Flag.Buxian = 1;
        }
        else if(R_l_guai.flag && R_h_guai.flag)
        {
            float kr, br;

            if(R_h_guai.row < R_l_guai.row)
            {
                kr = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'k');
                br = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'b');

                for (int i = R_h_guai.row; i <= R_l_guai.row; i++)
                {
                    Right_Sideline[i] = kr * i + br;
                    Right_Sideline_flag[i] = 1;
                }
            }

            Flag.Buxian = 1;
        }


       //只找到左上
       
       else if(L_h_guai.flag&&imgInfo.Both_lose>0)
       {
           float kl, bl;

           kl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 3, Left_Sideline[imgInfo.bottom - 3], 'k');
           bl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 3, Left_Sideline[imgInfo.bottom - 3], 'b');

           for (int i = L_h_guai.row; i <= imgInfo.bottom - 5; i++)
           {
               Left_Sideline[i] = kl * i + bl;
               Left_Sideline_flag[i] = 1;
               if(
                       (Left_Sideline[i] == Left_Sideline[i + 1] && Left_Sideline[i] == Left_Sideline[i + 2])
                       || !Image_Use[i + 1][(uint8)(kl * (i + 1) + bl)]
                  )
                  break;
            }

            Flag.Buxian = 1;
       }

       //只找到右上
       else if(R_h_guai.flag&&imgInfo.Both_lose>0)
       {
               float kr, br;

               kr = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 3, Right_Sideline[imgInfo.bottom - 3], 'k');
               br = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 3, Right_Sideline[imgInfo.bottom - 3], 'b');

               for (int i = R_h_guai.row; i < imgInfo.bottom - 5; i++)
               {
                   Right_Sideline[i] = kr * i + br;
                   Right_Sideline_flag[i] = 1;
                   if (
                           (Right_Sideline[i] == Right_Sideline[i + 1] && Right_Sideline[i] ==  Right_Sideline[i + 2])
                           || !Image_Use[i + 1][(uint8)(kr * (i + 1) + br)]
                       )
                       break;
               }

               Flag.Buxian = 1;
       }

//        else if(L_l_guai.flag && R_l_guai.flag)
//        {
//            float kr, br;
//
//            kr = xielv_sideline(R_l_guai.row, R_l_guai.column, imgInfo.top + 5, Right_Sideline[imgInfo.top + 5], 'k');
//            br = xielv_sideline(R_l_guai.row, R_l_guai.column, imgInfo.top + 5, Right_Sideline[imgInfo.top + 5], 'b');
//
//            for (int i = R_l_guai.row; i > imgInfo.top + 5; i--)
//            {
//                Right_Sideline[i] = kr * i + br;
//                Right_Sideline_flag[i] = 1;
//                if (
//                        (Right_Sideline[i] == Right_Sideline[i + 1] && Right_Sideline[i] ==  Right_Sideline[i + 2])
//                        || !Image_Use[i + 1][(uint8)(kr * (i + 1) + br)]
//                    )
//                    break;
//            }
//
//
//            kr = xielv_sideline(L_l_guai.row, L_l_guai.column, imgInfo.top + 5, Left_Sideline[imgInfo.top + 5], 'k');
//            br = xielv_sideline(L_l_guai.row, L_l_guai.column, imgInfo.top + 5, Left_Sideline[imgInfo.top + 5], 'b');
//
//            for (int i = L_l_guai.row; i > imgInfo.top + 5; i--)
//            {
//                Left_Sideline[i] = kr * i + br;
//                Left_Sideline_flag[i] = 1;
//                if (
//                        (Left_Sideline[i] == Left_Sideline[i + 1] && Left_Sideline[i] ==  Left_Sideline[i + 2])
//                        || !Image_Use[i + 1][(uint8)(kr * (i + 1) + br)]
//                    )
//                    break;
//            }
//
//            Flag.Buxian = 1;
//        }

        else
            Flag.Buxian = 0;
//    }
//        else
//            Flag.Buxian = 0;
}


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



int red_find_x,red_find_y,red_find_y1;
int picture_first_num,picture_second_num,picture_third_num,maxlong_colume,colume_long[94], long_max,picture_white=0,picture_black=0,jump_point1;
float black_ratio;
// int red_find_x,red_find_y,red_find_y1;
// int picture_first_num,picture_second_num,picture_third_num,maxlong_colume,colume_long[94], long_max,picture_white=0,picture_black=0,jump_point1;
// float black_ratio;
int red_x_mid,red_y_mid,x_err_red;
float err_picture;
int red_left,red_right;
float real_picture_distance,recognize_distance,recognize_distance2;
void picture(void)
{
        picture_white=0;
        picture_black=0;
        jump_point1=0;
        black_ratio=0;
        err_picture=100;
        x_err_red=200;



                   float K1 = xielv_sideline(55, Left_Sideline[55], 45, Left_Sideline[45], 'k');
                   float B1 = xielv_sideline(55, Left_Sideline[55], 45, Left_Sideline[45], 'b');
                    float K2 = xielv_sideline(55, Right_Sideline[55], 45, Right_Sideline[45], 'k');
                   float B2 = xielv_sideline(55, Right_Sideline[55], 45, Right_Sideline[45], 'b');
                   red_left=K1*MAX(R_h_guai.row,L_h_guai.row)+B1;
                   red_right=K2*MAX(R_h_guai.row,L_h_guai.row)+B2;

                // recognize_distance=Now_Speed*0.10+40;

                recognize_distance=Now_Speed*0.1+40;
                recognize_distance2=30;
    if(Flag.picture==0)
{

      if(R_h_guai.flag==1&&L_h_guai.flag==1)//&&(real_distance[imgInfo.top]-real_distance[R_h_guai.row])>25&&imgInfo.Both_lose==0 
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


                
}




     else if(R_h_guai.flag==1){
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
}

      else if(L_h_guai.flag==1){
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

}}






               if(Flag.Redblock==1){Flag.picture=2;}
    //         if(x_err_red>5&&imgInfo.R_straight_flag==0&&imgInfo.R_straight_flag==1)
    //         {
    //            Flag.small_rock =1;
    //         }

    //         if(x_err_red>5&&imgInfo.R_straight_flag==1&&imgInfo.R_straight_flag==0)
    //         {
    //            Flag.small_rock =2;
    //         }

    // if(Flag.small_rock ==1||Flag.small_rock ==2)
    // {
    //     if(distance>10)
    //     {
    //         distance=0;
    //         Flag.small_rock =0;
    //     }
    // }

}





    if(Flag.picture==1){
        
    }
    if(Flag.picture==2){

        Flag.infer = 1;
        if((L_h_guai.flag==1&&real_distance[MAX(R_h_guai.row,L_h_guai.row)]<30)||(R_h_guai.flag==1&&abs(recognize_distance2-real_distance[MAX(R_h_guai.row,L_h_guai.row)])<2))//&&fabs(Now_Speed)<10
        {
    //    Image.Kp/=2.5;
           if(R_h_guai.flag==1&&L_h_guai.flag==1)//&&(real_distance[imgInfo.top]-real_distance[R_h_guai.row])>25&&imgInfo.Both_lose==0 
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


                
}





      else if(R_h_guai.flag==1&&abs(recognize_distance2-real_distance[R_h_guai.row])<2)//&&fabs(Now_Speed)<10&&R_h_guai.row>real_distance_to_row(80)&&(real_distance[imgInfo.top]-real_distance[R_h_guai.row])>60&&real_distance[imgInfo.top]>100
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


}

      else if(L_h_guai.flag==1&&abs(recognize_distance2-real_distance[L_h_guai.row])<2)//&&fabs(Now_Speed)<10&&L_h_guai.row>real_distance_to_row(80)&&(real_distance[imgInfo.top]-real_distance[L_h_guai.row])>60&&real_distance[imgInfo.top]>100
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
}

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
    }

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



void protect(void)
{
// if(Flag.Zebra_cross != 1&&Flag.Zebra_cross != 4&&run_flag==1)//&&Flash.mtv_exposure_time>=100
// {
//     if(imgInfo.top >=57||fabs(icm_data.gyro_z)>30)//&&Flag.Zhangai != 1
//     {
//         run_flag =2;
//         printf("异常触发保护\r\n");
//     }//||Speed_Encoder_l>2000||Speed_Encoder_r>2000||Speed_now>1500


// }
}
lq_camera_ex cam(320,240,156,LQ_CAMERA_0CPU_MJPG,LQ_CAMERA_PATH);
void image_init(void)
{  
    
    std::string model_param = "tiny_classifier_fp32.ncnn.param";//tiny_classifier_fp32.ncnn.param
    std::string model_bin   = "tiny_classifier_fp32.ncnn.bin";//tiny_classifier_fp32.ncnn.bin
    int input_width    = 60;
    int input_height   = 60;
    std::vector<std::string> labels = {"supply", "vehicle", "weapon"};
        // 归一化参数（ImageNet标准）
     float mean_vals[3] = {123.675f, 116.28f, 103.53f};
     float norm_vals[3] = {0.01712475f, 0.017507f, 0.01742919f};

    classifier.SetModelPath(model_param, model_bin);
    classifier.SetInputSize(input_width, input_height);
    classifier.SetLabels(labels);
    classifier.SetNormalize(mean_vals, norm_vals);
    classifier.Init();
    cam.start_collect();
    if(cam.is_cam_opened())
    {
        printf("Camera opened successfully!\n");
    }
    else{
        printf("Camera opened failed!\n");
        return;
    }
    cam.set_exposure_manual(90);
    printf("龙邱摄像头宽度:%d\n",cam.get_camera_width());
    printf("龙邱摄像头高度:%d\n",cam.get_camera_height());
    printf("龙邱摄像头帧率:%d\n",cam.get_camera_fps());
    //预分配内存
    grayFrame.create(LCDH_0, LCDW_0, CV_8UC1);
    resizedFrame.create(LCDH_1, LCDW_1, CV_8UC1);
    translatedFrame.create(LCDH_1, LCDW_1, CV_8UC1);
    binaryFrame.create(LCDH_1, LCDW_1, CV_8UC1);
    translationMatrix = (cv::Mat_<float>(2, 3) << 1, 0, 0, 0, 1, 0);//将捕获图像向右平移3个像素点
   camera_server.start_server(8080);//打开图传服务器

}


uint16_t jump_point,finish_flag;
float distance_cross,distance,distance_picture;


void distance_judge(void)//1m=66
{
           if(Flag.Huandao_L==1||Flag.Huandao_R==1||Flag.Huandao_L==2||Flag.Huandao_R==2||Flag.Huandao_R==6||Flag.Huandao_L==6
                  ||Flag.small_rock||Flag.Zebra_cross==1||Flag.Zebra_cross==2||Flag.Zebra_cross==4||Flag.ramp!=0)//||Flag.Huandao_R==3||Flag.Huandao_L==3
    distance+=(float)(encoder_L.count_now+encoder_R.count_now)/2/350;
    if(Flag.picture==3||Flag.picture==4||Flag.picture==5||Flag.picture==6)
    {
    distance_picture+=(float)(encoder_L.count_now+encoder_R.count_now)/2/350;
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
            run_flag=2;
            esc_pwm.set_duty(500);
        }

    }

 }






uint16_t small_rock_r_num,small_rock_l_num;
void small_rock(void)
{
    if(Flag.small_rock ==0)
    {
                   float K1 = xielv_sideline(55, Left_Sideline[55], 45, Left_Sideline[45], 'k');
                   float B1 = xielv_sideline(55, Left_Sideline[55], 45, Left_Sideline[45], 'b');
                    float K2 = xielv_sideline(55, Right_Sideline[55], 45, Right_Sideline[45], 'k');
                   float B2 = xielv_sideline(55, Right_Sideline[55], 45, Right_Sideline[45], 'b');
                   red_left=K1*MAX(R_h_guai.row,L_h_guai.row)+B1;
                   red_right=K2*MAX(R_h_guai.row,L_h_guai.row)+B2;


      if(R_h_guai.flag==1&&L_h_guai.flag==1)//&&(real_distance[imgInfo.top]-real_distance[R_h_guai.row])>25&&imgInfo.Both_lose==0 
{
 
            if(MAX(R_h_guai.row,L_h_guai.row)>real_distance_to_row(40)&&MAX(R_h_guai.row,L_h_guai.row)<real_distance_to_row(10)&&
            (real_distance[imgInfo.top]-real_distance[MAX(R_h_guai.row,L_h_guai.row)])>60&&real_distance[imgInfo.top]>100) 
            {
                    Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//94//320
               red_find_y=real_distance_to_row(real_distance[MAX(R_h_guai.row,L_h_guai.row)]+12);//red_find_y到R_h_guai.row（*4)
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
                // Flag.Redblock=1;
               }
            } 


                
}





      if(R_h_guai.flag==1&&R_h_guai.row>real_distance_to_row(40)&&R_h_guai.row<real_distance_to_row(10)&&
      (real_distance[imgInfo.top]-real_distance[R_h_guai.row])>60&&real_distance[imgInfo.top]>100)
{


                    Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//94//320
               red_find_y=real_distance_to_row(real_distance[R_h_guai.row]+12);//red_find_y到R_h_guai.row（*4)
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


}

      else if(L_h_guai.flag==1&&L_h_guai.row>real_distance_to_row(40)&&L_h_guai.row<real_distance_to_row(10)
      &&(real_distance[imgInfo.top]-real_distance[L_h_guai.row])>60&&real_distance[imgInfo.top]>100)
{



                    Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//Right_Sideline[L_h_guai.row]
               red_find_y=real_distance_to_row(real_distance[L_h_guai.row]+12);//
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







                


}




            if(x_err_red>5&&imgInfo.R_straight_flag==0&&imgInfo.R_straight_flag==1)
            {
               Flag.small_rock =1;
            }

            if(x_err_red>5&&imgInfo.R_straight_flag==1&&imgInfo.R_straight_flag==0)
            {
               Flag.small_rock =2;
            }



    }


    if(Flag.small_rock ==1||Flag.small_rock ==2)
    {
        if(distance>10)
        {
            distance=0;
            Flag.small_rock =0;
        }
    }
}



float ramp_line,ramp_err;
void ramp(void)
{

//    float k = xielv_sideline(30, (int)Dir_Err[30], 45, (int)Dir_Err[45], 'k');
//    float b = xielv_sideline(30, (int)Dir_Err[30], 45, (int)Dir_Err[45], 'b');
if(Flag.ramp==0)
{
     
    if(imgInfo.top<20)
  {
        ramp_err=0;
            //  ramp_err=50;
       for(int i = imgInfo.top+5;i<55 ;i++)
    {
            //   if(fabs(Dir_Err[i]-k*i-b)>2)
            // {
            //        if(i<50||(Right_Sideline_flag[i]==1&&Left_Sideline_flag[i]==1))
            //            ramp_line++;
            // }
          ramp_err+=Dir_Err[i];
    }
       if(dl1x_distance_raw>50&&dl1x_distance_raw<1500&&fabs(ramp_err)<30)//ramp_line==0&&&&B<0.2
    {
           Flag.ramp=2;

    }

  }
}
// if(Flag.ramp==1)
// {
//     if(distance>30)
//  {
//         Flag.ramp=2;
//  }
// }
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
 /***************************************************检测红色矩形块******************************************************/

 std::vector<RedObject> red_objects;
 
 int red_area = 0;
  #define MIN_RED_AREA 0//最小红色区域面积 用于滤除红色噪点
 #define MIN_AREA_FOR_BARRIER 250 //障碍物面积
int red_points_num;
int alpha = 0.1;
cv::Point center(-1,-1);
cv::Point last_center(-1,-1);
 void DetectRedBlock(cv::Mat &src,int roi_x,int roi_y,int width,int height)
{
    red_objects.clear();//先清元素
    red_area = 0;
    red_points_num = 0;
    long long sum_x = 0;
    long long sum_y = 0;
    if(src.empty()) 
    {
        printf("DetectRedBlock src is empty\n");
        return;
    }
    if(roi_x<0 || roi_y<0||roi_x >= src.cols || roi_y >= src.rows)  return;
    if(width <= 0 || height <= 0 || width>src.cols||height>src.rows) return;
    if(roi_x + width > src.cols || roi_y + height > src.rows) return;
    cv::Rect roi(roi_x,roi_y,width,height);

    cv::Rect roi_rect;
    cv::Mat roi_src = src(roi);
    cv::Mat mask;
    mask.create(roi_src.size(), CV_8UC1);


    for (int y = 0; y < roi_src.rows; ++y)
    {
        const cv::Vec3b* src_ptr = roi_src.ptr<cv::Vec3b>(y);
        uchar* mask_ptr = mask.ptr<uchar>(y);

        for (int x = 0; x < roi_src.cols; ++x)
        {
            int b = src_ptr[x][0];
            int g = src_ptr[x][1];
            int r = src_ptr[x][2];

            if (r > Flash.debug_rgb_r_min && (r - g) > Flash.debug_rgb_rg_diff && (r - b) > Flash.debug_rgb_rb_diff){
                red_points_num++;
                sum_x += x;
                sum_y += y;
                mask_ptr[x] = 255;
            }
            else{
                mask_ptr[x] = 0;
            }
        }
    }

  
if (red_points_num >= 5)
    {
        int cx = sum_x / red_points_num;
        int cy = sum_y / red_points_num;
        
        resize_cx = cx + roi_x;
        resize_cy = cy + roi_y;
        // 映射到原图坐标
        if(resize_cx*3.4>320) return;
        if(resize_cy*4>240) return;
        center= cv::Point((resize_cx)*3.4, (resize_cy)*4);//修改1

        if(last_center != cv::Point(-1, -1))
        {
            int dx = center.x - last_center.x;
            int dy = center.y - last_center.y;
            double dist = sqrt(dx * dx + dy * dy);
            // if(dist>80){
            //     printf("本帧无效\n");
            //     return;
            // }
            if(dist<16)
            {
                center = last_center;
            }
        }
        last_center = center;
        red_area = red_points_num;
        red_objects.push_back({center,red_area});

        // 可选：画点
        //cv::circle(lq_frame, center, 3, cv::Scalar(0, 255, 0), -1);
        //printf("检测到红色块\n");
       // printf("Red Center: (%d, %d), Area: %d,red_points_num: %d, sum_x:%d\n",center.x, center.y, red_area,red_points_num,sum_x);
    }
    else{
        center = cv::Point(-1,-1);
        last_center = cv::Point(-1,-1);
        resize_cx = 0;
        resize_cy = 0;
    }

    if(!red_objects.empty()&&Flag.infer ==1)
    {   
   auto &obj = red_objects[0];

        if (obj.center.x < 0 || obj.center.y < 0 || 
        obj.center.x >= lq_frame.cols || obj.center.y >= lq_frame.rows||obj.center.y < 0||obj.center.x < 0) 
        {
        return;
        }

        if(!GenerateROI(obj.center, roi_rect, lq_frame)){
            return;
        }
        if (roi_rect.x < 0 || roi_rect.y < 0 ||
            roi_rect.width <= 0 || roi_rect.height <= 0 ||
            roi_rect.x + roi_rect.width > lq_frame.cols ||
            roi_rect.y + roi_rect.height > lq_frame.rows)
        {
                std::cout << "BAD ROI: "
              << roi_rect << " | img: "
              << lq_frame.cols << "x"
              << lq_frame.rows << std::endl;
                return;
        }
            cv::Mat roi_img = lq_frame(roi_rect);//截图送入模型
              if(roi_img.empty()||roi_img.rows<=0||roi_img.cols<=0)
                 {
                    return;
                 }
                // cv::rectangle(lq_frame, roi_rect, cv::Scalar(255, 0, 0), 1);
                 float confidence;
                 auto start_time = std::chrono::high_resolution_clock::now();

                 real_picture_distance=real_distance[MAX(L_h_guai.row,R_h_guai.row)];
                 printf("real_picture_distance:%f\n",real_picture_distance);

                 std::string result = classifier.Infer(roi_img,confidence);
                auto end_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> elapsed_ms = end_time - start_time;
                 printf("检测结果:%s, 置信度: %.1f,推理耗时: %.2f ms\n",result.c_str(),confidence,elapsed_ms.count());


                if(result == "supply" && confidence>40)
                {
                    Flag.supply++;
                    Flag.weapon = 0;
                    Flag.vehicle = 0;
                }
                if(result == "weapon" && confidence>40)
                {
                    Flag.supply = 0;
                    Flag.weapon++;
                    Flag.vehicle = 0;
                }
                if(result == "vehicle" && confidence>40)
                {
                    Flag.supply = 0;
                    Flag.weapon = 0;
                    Flag.vehicle++;
                }
        
    }

}


//标注ROI区域
bool GenerateROI(const cv::Point &center, cv::Rect &roi, const cv::Mat &src)
{
    if (center.x < 0 || center.y < 0) {
        return false;  // 点坐标无效
    }
    
    if (src.empty()) {
        return false;  // 图像为空
    }
    int half = ROI_SIZE/2;
    roi_x_1 = center.x - half;
    roi_y_1 = center.y - (int)half*2;//把中心的y坐标稍微向上移动一些

    if (roi_x_1 < 0 || roi_y_1 < 0)
        return false;

    if (roi_x_1 + ROI_SIZE > src.cols)
        return false;

    if (roi_y_1+ ROI_SIZE > src.rows)
        return false;

    roi = cv::Rect(roi_x_1, roi_y_1, ROI_SIZE, ROI_SIZE);
    return true;
}
/***************************************************红色标注******************************************************/
enum ClassType{
    CLASS_SUPPLIES = 0,
    CLASS_VEHICLE,
    CLASS_WEAPON,
    CLASS_UNKNOWN
};

ClassType GetClassID(const std::string &cls){
    if(cls == "supplies")
        return CLASS_SUPPLIES;
    else if(cls == "vehicle")
        return CLASS_VEHICLE;
    else if(cls == "weapon")
        return CLASS_WEAPON;
    return CLASS_UNKNOWN;
}
ClassType id;

#define SEARCH_BOTTOM_RATIO          1.0f   // 1.0=全图搜索；0.5=只搜下半部分
#define MIN_RED_AREA                 40     // 最小红块面积

// HSV 红色阈值
#define LOWER_RED_H1                 0
#define UPPER_RED_H1                 10
#define LOWER_RED_H2                 160
#define UPPER_RED_H2                 179
#define LOWER_RED_S                  120
#define LOWER_RED_V                  70

#define ROI_OFFSET_X                 0
#define ROI_OFFSET_Y                 -10
cv::Rect crop_rect;
cv::Mat src_img;
cv::Mat roi_img;         // 纯ROI
cv::Rect red_rect;       // 红块框
cv::Point roi_center;    // ROI中心
//限幅函数
static inline int clamp_int(int v, int low, int high)
{
    if (v < low)  return low;
    if (v > high) return high;
    return v;
}

 bool detect_red_and_crop_roi(const cv::Mat& src_img,
                             cv::Mat& roi_img,
                             cv::Rect& best_rect,
                             cv::Rect& crop_rect,
                             cv::Point& roi_center)
{
    roi_img.release();
    best_rect = cv::Rect();
    crop_rect = cv::Rect();
    roi_center = cv::Point(-1, -1);

    if (src_img.empty()) return false;

    // 只在指定区域搜索
    int roi_y = static_cast<int>(src_img.rows * (1.0f - SEARCH_BOTTOM_RATIO));
    roi_y = clamp_int(roi_y, 0, src_img.rows - 1);

    cv::Rect search_roi(0, roi_y, src_img.cols, src_img.rows - roi_y);
    cv::Mat src_search = src_img(search_roi);

    // 转HSV找红色
    cv::Mat hsv, mask1, mask2, mask;
    cv::cvtColor(src_search, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv,
            Scalar(LOWER_RED_H1, LOWER_RED_S, LOWER_RED_V),
            Scalar(UPPER_RED_H1, 255, 255),
            mask1);

    cv::inRange(hsv,
            Scalar(LOWER_RED_H2, LOWER_RED_S, LOWER_RED_V),
            Scalar(UPPER_RED_H2, 255, 255),
            mask2);

    mask = mask1 | mask2;

    // 开闭运算去噪         打开后会降低图像帧率 20帧左右
    // static cv::Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    // cv::morphologyEx(mask, mask, MORPH_OPEN, kernel);
    // cv::morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    // 查找轮廓
    std::vector<std::vector<Point>> contours;
    cv::findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return false;

    // 选择面积最大的红块
    double max_area = 0.0;
    cv::Rect max_rect;

    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        if (area < MIN_RED_AREA) continue;

        cv::Rect rect = cv::boundingRect(contours[i]);
        if (area > max_area)
        {
            max_area = area;
            max_rect = rect;
        }
    }

    if (max_area < MIN_RED_AREA) return false;

    // 转回原图坐标
    max_rect.y += roi_y;
    best_rect = max_rect;

    int red_cx = best_rect.x + best_rect.width / 2;// 红块中心x坐标
    int red_cy = best_rect.y + best_rect.height / 2;// 红块中心y坐标

    int cx = red_cx + ROI_OFFSET_X;
    int cy = red_cy + ROI_OFFSET_Y;
    roi_center = Point(cx, cy);// ROI中心点
    //printf("cx:%d,cy:%d\n",cx,cy);

    roix1 = cx - MODEL_INPUT_WIDTH / 2;
    roiy1 = cy - MODEL_INPUT_HEIGHT / 2;

    roix1 = clamp_int(roix1, 0, src_img.cols - MODEL_INPUT_WIDTH);//ROI框左上和角x坐标
    roiy1 = clamp_int(roiy1, 0, src_img.rows - MODEL_INPUT_HEIGHT);//ROI框左上角y坐标

    crop_rect = cv::Rect(roix1, roiy1, MODEL_INPUT_WIDTH, MODEL_INPUT_HEIGHT);

    if (crop_rect.x < 0 || crop_rect.y < 0 ||
        crop_rect.x + crop_rect.width > src_img.cols ||
        crop_rect.y + crop_rect.height > src_img.rows)
    {
        return false;
    }

    roi_img = src_img(crop_rect);

    return true;
}



//绕行阶段 转角 直行 回线
// void avoid_process()
// {
//     if(Flag.picture == 0||Flag.picture == 4)
//     {
//         Flag.picture = 0;
//         return;
//     }
//     //右行
//     if(Flag.picture == 2)
//     {
//         //encoder_abs 已经开始累加
//         if(!picture_yaw_init)
//         {
//             picture_yaw_init = true;
//             Yaw_picture = icm_data.yaw;//记录当前的yaw值
//             encoder_val = encoder_abs;//记录初始的编码器位置
//             Yaw_picture_target = -25.0f;//目标偏差值
//         }
//         float current_distance = encoder_abs - encoder_val;
//         //转向阶段
//         if(current_distance<=4000)
//         {   //printf("开始转弯\n");
//             Yaw_picture_target = -25.0f*(current_distance/4000);
//         }
//         //保持最大偏向角直行
//         else if(4000<current_distance<=6000)
//         {  // printf("开始直行\n");
//             Yaw_picture_target = -25.0f;
//         }
//         //回线阶段 回线阶段的误差不对
//         else if(6000<current_distance<=9000)
//         {   //printf("开始回线\n");
//             Yaw_picture_target = -25.0f+25.0f*(current_distance-6000)/3000;
//         }
//         Yaw_picture_diff = Yaw_correct(icm_data.yaw,Yaw_picture);//实际偏差值
//         Yaw_picture_err = Yaw_picture_target - Yaw_picture_diff;//陀螺仪的纠正值
//         Dir_err=Yaw_picture_err;

//     }
// }

/***************************************************图像处理******************************************************/
void ImageDeal()
{   
    lq_frame = cam.get_frame_raw();
    if(lq_frame.empty()){
        return; 
    }
      cv::resize(lq_frame,resizedFrame,cv::Size(LCDW_1,LCDH_1),0,0,cv::INTER_AREA);

    // RedBlockProcess(resizedFrame);
    // camera_server.update_frame_mat(lq_frame);//打开图传服务器

      cv::cvtColor(resizedFrame,grayFrame,cv::COLOR_BGR2GRAY);

    //   cv::warpAffine(grayFrame, translatedFrame, translationMatrix, grayFrame.size(), 
    //                  cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

//   //  将OpenCV图像数据复制到图像数组 采用memcpy函数加快处理速度
    for (int i = 0; i < LCDH_1; i++)
    {
        uint8_t *p = grayFrame.ptr<uint8_t>(i);
        for(int w = 0; w < LCDW_1; w++)
        {
            Image_Use[i][w] = p[w];

        }
    }
         Get01change_dajin();

        // my_sobel(Image_Zip,Image_Use); //压缩后的图像数组,

        // my_sobel_dajin(Image_Zip,Image_Use);

        imgInfoInit();

        Get_ImageTop();

        Draw_BlackSideline(Image_Use);//画边线

        Find_Sideline(imgInfo.bottom-1,imgInfo.top+ 1);//找边线

        if(Flag.Huandao_L>0||Flag.Huandao_R>0)
        Find_Guaidian();  //找拐点
        else
        Find_Guaidian1();  //找拐点

        straight_judge();

        zebra_corssing();

        if(Flag.Zebra_cross==3)
        {
        ramp();
        }

        picture();

        if(Flag.Huandao_L==0&&Flag.Huandao_R==0)
        small_rock();
        // }

        // if(Flag.Huandao_L!=1&&Flag.Huandao_R!=1)


       Huandao_R_imu();
       Huandao_L_imu();


       if(Flag.Huandao_L!=1&&Flag.Huandao_R!=1&&Flag.Huandao_L!=2&&Flag.Huandao_R!=2&&Flag.Huandao_L!=3&&Flag.Huandao_R!=3
       &&Flag.Huandao_L!=4&&Flag.Huandao_R!=4&&Flag.Huandao_L!=5&&Flag.Huandao_R!=5&&Flag.Huandao_L!=6&&Flag.Huandao_R!=6)
        Buxian();

        Find_Midline();

     //   dynamic_forward();



        Err_Sum();




        protect();
}