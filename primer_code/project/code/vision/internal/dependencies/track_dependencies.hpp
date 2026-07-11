#pragma once

#include "../vision_stage_contracts.hpp"

extern uint8_t Image_Use[LCDH_1][LCDW_1];
extern int Left_Sideline[LCDH_0];
extern int Right_Sideline[LCDH_0];
extern int Mid_Line[LCDH_0];
extern int Last_Mid_Line[LCDH_0];
extern unsigned char Left_Sideline_flag[LCDH_0];
extern unsigned char Right_Sideline_flag[LCDH_0];
extern unsigned char white_width[LCDH_0];
extern unsigned char starith_white_width[LCDH_0];
extern Guaidian L_l_guai, L_h_guai, R_l_guai, R_h_guai, R_h_guai1, L_h_guai1;
extern YuanSu Flag;
extern imageInformation imgInfo;
extern float real_distance[60];
extern float curvature;
extern float B;
extern float A;
extern uint16_t maxkuan_line;
extern int right_num, r_num, left_num, l_num, R_l_lsoe, L_l_lose;
