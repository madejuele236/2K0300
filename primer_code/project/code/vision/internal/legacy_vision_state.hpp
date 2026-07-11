#pragma once

#include "../vision_facts.hpp"

#include <vector>

// Compatibility state retained for the unchanged primer vision algorithms.
// Only vision implementation files may include this header. Cross-layer code
// observes facts and requests the few supported mutations through
// vision_queries.hpp.
extern uint8_t Image_Zip[LCDH_1][LCDW_1];
extern uint8_t Image_Use[LCDH_1][LCDW_1];
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
extern float real_distance[60];
extern float curvature, distance, distance_picture, ramp_err, recognize_distance2;
extern uint16_t jump_point, maxkuan_line;
extern int avoid_state, x_err_red;
extern int forward, forward1, red_x_mid, red_y_mid;
extern float Yaw_Huandao, Yaw_Huandao_err, yaw_correct, distance_HUAN1, black_ratio, err_picture, real_picture_distance;
extern int right_num, r_num, left_num, l_num, R_l_lsoe, L_l_lose, picture_first_num, picture_second_num, maxlong_colume, long_max, jump_point1, picture_white, picture_black, red_find_x, red_find_y, red_find_y1;
