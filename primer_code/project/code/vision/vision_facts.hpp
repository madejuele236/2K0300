#pragma once

#include "zf_common_typedef.hpp"

#include <opencv2/core.hpp>

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
