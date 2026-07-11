#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include "internal/dependencies/red_target_dependencies.hpp"
using namespace cv;
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
