#pragma once

#include "../vision_stage_contracts.hpp"

extern uint8_t Image_Use[LCDH_1][LCDW_1];
extern int Left_Sideline[LCDH_0];
extern int Right_Sideline[LCDH_0];
extern unsigned char Left_Sideline_flag[LCDH_0];
extern unsigned char Right_Sideline_flag[LCDH_0];
extern Guaidian L_l_guai, L_h_guai, R_l_guai, R_h_guai;
extern YuanSu Flag;
extern imageInformation imgInfo;
extern float distance;
