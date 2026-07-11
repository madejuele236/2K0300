#include "internal/dependencies/facts_dependencies.hpp"
#include "vision_queries.hpp"
#include "zf_device_uvc.hpp"
#include "tensorflow/lite/core/c/common.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/micro/cortex_m_generic/debug_log_callback.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "loong_cnn_model_simple.h"
#include <cmath>
#include <cstdio>
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
uint8_t Threshold = 0;  //大津法求出的阈值
struct imageInformation imgInfo;
struct YuanSu Flag;
cv::Mat frame, grayFrame, binaryFrame,resizedFrame,flippedFrame,translatedFrame,translationMatrix,float_img,resized_img;
int roi_x_1= 0;
int roi_y_1 = 0;
int ROI_SIZE = 60;
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

namespace primer::vision {

VisionControlLiveView ObserveVisionControlLiveView()
{
    return {Flag, imgInfo, L_h_guai, R_h_guai, real_distance,
            Dir_err, D_ERR, jump_point};
}

VisionPresentationLiveView ObserveVisionPresentationLiveView()
{
    return {Image_Use, Left_Sideline, Right_Sideline, Mid_Line,
            Flag, imgInfo, L_l_guai, L_h_guai, R_l_guai, R_h_guai,
            L_h_guai1, R_h_guai1, real_distance, resizedFrame,
            Dir_err, distance, Yaw_Huandao_err, black_ratio, jump_point,
            maxlong_colume, long_max, jump_point1, picture_white,
            picture_black, red_find_x, red_find_y};
}

RoundaboutYawState AccessRoundaboutYawState()
{
    return {Yaw_Huandao, yaw_correct, Yaw_Huandao_err};
}

int VisionDynamicForward()
{
    return forward1;
}

void SetVisionDynamicForward(int value)
{
    forward1 = value;
}

float VisionRoundaboutYaw()
{
    return Yaw_Huandao;
}

void SetVisionRoundaboutYawCorrection(float corrected_yaw, float yaw_error)
{
    yaw_correct = corrected_yaw;
    Yaw_Huandao_err = yaw_error;
}

}  // namespace primer::vision

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
