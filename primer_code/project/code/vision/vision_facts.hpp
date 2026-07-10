#pragma once

#include "zf_common_typedef.hpp"

#include <opencv2/core.hpp>
#include <vector>

#ifndef MIN
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#endif
#ifndef MAX
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#endif

#define LCDH_0 120
#define LCDW_0 160
#define LCDH_1 60
#define LCDW_1 94
#define FPS 120
#define white 255
#define black 0

struct RedObject {
    cv::Point center;
    int area;
};

struct imageInformation {
    uint16 bottom=0;
    uint16 top=0;
    uint16 lastMid=0;
    uint16 white_num=0;
    uint16 max_column=0;
    uint16 Control_Row=0;
    uint16 L_loselineSum=0;
    uint16 R_loselineSum=0;
    uint16 L_track_row=0;
    uint16 R_track_row=0;
    uint16 L_straight_flag=0;
    uint16 R_straight_flag=0;
    uint16 L_lose_r_num=0;
    uint16 R_lose_L_num=0;
    uint16 Both_lose=0;
    float err_sum=0;
};

struct Guaidian {
    uint8 row=0;
    uint8 column=0;
    uint8 flag=0;
};

struct Wandian {
    uint8 row=0;
    uint8 column=0;
    uint8 flag=0;
};

struct YuanSu {
    uint8 Buxian=0;
    uint8 Shizi=0;
    uint8 Huandao_L=0;
    uint8 Huandao_R=0;
    uint8 Zhangai=0;
    uint8 NO_Zhangai=0;
    uint8 Zebra_cross=0;
    uint8 ramp=0;
    uint8 small_rock=0;
    uint8 Redblock=0;
    uint8 picture=0;
    uint8 picture_dir=0;
    uint8 avoid=0;
    uint8 infer=0;
    uint8_t jiansu=0;
    uint8 supply=0;
    uint8 weapon=0;
    uint8 vehicle=0;
};

extern uint8_t Image_Zip[LCDH_1][LCDW_1];
extern uint8_t Image_Use[LCDH_1][LCDW_1];
extern unsigned char Image_IFS[LCDH_1][LCDW_1];
extern int Left_Sideline[LCDH_0], Right_Sideline[LCDH_0], zhangai_Rnum, zhangai_Lnum;
extern int Mid_Line[LCDH_0];
extern unsigned char Left_Sideline_flag[LCDH_0];
extern std::vector<RedObject> red_objects;
extern int red_area;
extern Guaidian L_l_guai, L_h_guai, R_l_guai, R_h_guai, R_h_guai1, L_h_guai1;
extern Wandian L_l_wan, L_h_wan, R_l_wan, R_h_wan;
extern YuanSu Flag;
extern imageInformation imgInfo;
extern cv::Mat frame, grayFrame, binaryFrame, resizedFrame, flippedFrame, translatedFrame, translationMatrix;
extern cv::Mat lq_frame;
extern cv::Point center;
extern float Dir_err, Last_Dir_err, Dir_Err[60], D_ERR;
extern char txt[80];
extern float real_distance[60];
extern float curvature, distance, distance_picture, ramp_err, recognize_distance2;
extern uint16_t jump_point, maxkuan_line;
extern int avoid_state, x_err_red;
extern int forward, forward1, red_x_mid, red_y_mid;
extern float Yaw_Huandao, Yaw_Huandao_err, yaw_correct, distance_HUAN1, black_ratio, err_picture, real_picture_distance;
extern int right_num, r_num, left_num, l_num, R_l_lsoe, L_l_lose, picture_first_num, picture_second_num, maxlong_colume, long_max, jump_point1, picture_white, picture_black, red_find_x, red_find_y, red_find_y1;
