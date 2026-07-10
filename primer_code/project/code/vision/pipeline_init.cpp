#include "internal/vision_stage_contracts.hpp"
#include "internal/camera_service.hpp"
#include "internal/classifier_service.hpp"
#include "internal/transport_service.hpp"
#include <cstdio>
#include <string>
#include <vector>
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
