#pragma once

#include "../vision_facts.hpp"
#include "legacy_vision_constants.hpp"

#define MODEL_INPUT_WIDTH 40
#define MODEL_INPUT_HEIGHT 40

int real_distance_to_row(float distance);

void imgInfoInit(void);
uint8_t Threshold_deal(uint8 image[LCDH_1][LCDW_1], uint16 col, uint16 row);
void Get01change_dajin(void);
void my_sobel(unsigned char imageIn[LCDH_1][LCDW_1], unsigned char imageOut[LCDH_1][LCDW_1]);
void my_sobel_dajin(unsigned char imageIn[LCDH_1][LCDW_1], unsigned char imageOut[LCDH_1][LCDW_1]);
void Get_ImageTop(void);
int Find_Top(int line);
float xielv_sideline(int x1, int y1, int x2, int y2, char data);
void Find_Sideline(uint8 Start_row, uint8 End_row);
void Find_left_Sideline(uint8 Start_row, uint8 End_row);
void Find_right_Sideline(uint8 Start_row, uint8 End_row);
void Find_Guaidian(void);
void Find_Guaidian1(void);
void Find_l_h_Guaidian(void);
void Find_r_h_Guaidian(void);
void Find_Midline(void);
void straight_judge(void);
void regression(int startline, int endline);
void calculateCurvature(float x1, float y1, float x2, float y2, float x3, float y3);
void Huandao_L_imu(void);
void Huandao_R_imu(void);
void Buxian(void);
void dynamic_forward(void);
void Err_Sum(void);
void picture(void);
void protect(void);
void zebra_corssing(void);
void small_rock(void);
void ramp(void);

void DetectRedBlock(cv::Mat &src, int roi_x, int roi_y, int width, int height);
bool GenerateROI(const cv::Point &center, cv::Rect &roi, const cv::Mat &src);
void Draw_BlackSideline(uint8_t Image_1[LCDH_1][LCDW_1]);
