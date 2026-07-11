#pragma once

#include "../../../port/roundabout_yaw_port.hpp"
#include "../../vision_facts.hpp"
#include "../legacy_vision_constants.hpp"

extern uint8_t Image_Use[LCDH_1][LCDW_1];
extern int Left_Sideline[LCDH_0], Right_Sideline[LCDH_0];
extern unsigned char Left_Sideline_flag[LCDH_0];
extern unsigned char Right_Sideline_flag[LCDH_0];
extern Guaidian L_l_guai, L_h_guai, R_l_guai, R_h_guai;
extern YuanSu Flag;
extern imageInformation imgInfo;
extern float real_distance[60];
extern float distance;
extern float Yaw_Huandao, Yaw_Huandao_err, yaw_correct;
extern int right_num, r_num, left_num, l_num;

float xielv_sideline(int x1, int y1, int x2, int y2, char data);
void Get_ImageTop(void);
void Find_Sideline(uint8 Start_row, uint8 End_row);
void Buxian(void);
