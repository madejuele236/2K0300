#ifndef IMAGE_H
#define IMAGE_H

#include "vision/vision_facts.hpp"
#include "control.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

extern PID steering_pid;

void ImageDeal(void);
void image_init(void);
void distance_judge(void);
void DetectRedBlock(cv::Mat &src,int roi_x,int roi_y,int width,int height);
bool InTrack(int x,int y);
bool GenerateROI(const cv::Point &center, cv::Rect &roi, const cv::Mat &src);
void RedBlockProcess(cv::Mat &src);
int ArgMax(int *arr,int n);
void avoid_process(void);
void Draw_BlackSideline(uint8_t Image_1[LCDH_1][LCDW_1]);
void Find_Sideline(uint8 Start_row, uint8 End_row);
void zf_model_init(void);
void debug_log_printf_callback(const char* s);
float Yaw_correct(float current_yaw,float target_yaw);
int real_distance_to_row(float distance);
float actan_err(float err);

#endif
