#include "presentation/internal/presentation_state.hpp"

unsigned char Image_IFS[LCDH_1][LCDW_1];
char txt[80];
uint8_t key1_state,key2_state,key3_state,key4_state,key5_state,key6_state,long_key;
uint16_t key7_state, key8_state,key7_last_state,key8_last_state;
uint8_t key1_last_state,key2_last_state,key3_last_state,key4_last_state,key5_last_state,key6_last_state;
int8_t oled_flag,last_oled_flag;
int16_t Parameter_flag = 1;
uint16_t esc_duty = 550;
