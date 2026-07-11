#pragma once

#include "port/vision_geometry.hpp"
#include "zf_common_typedef.hpp"

inline constexpr int LCDH_1 = primer::port::vision::kProcessedHeight;
inline constexpr int LCDW_1 = primer::port::vision::kProcessedWidth;

extern unsigned char Image_IFS[LCDH_1][LCDW_1];
extern char txt[80];
extern uint8_t key1_state,key2_state,key3_state,key4_state,key5_state,key6_state,long_key;
extern uint16_t key7_state, key8_state,key7_last_state,key8_last_state;
extern uint8_t key1_last_state,key2_last_state,key3_last_state,key4_last_state,key5_last_state,key6_last_state;
extern int8_t oled_flag,last_oled_flag;
extern int16_t Parameter_flag;
extern uint16_t esc_duty;
