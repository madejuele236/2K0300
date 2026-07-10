#ifndef FLASH_H
#define FLASH_h

#define PARAM_FILE_NAME "params_config.txt"

struct FlashInformation
{
    // float dajin_or_sober;
    // float mtv_exposure_time;//300
    // float mtv_gain_time;
    // float bridge_time_short;//30
    // float bridge_time_high;//30
    // float bridge_state;//2
    // float jump_distance;//6

    // float huandao_desicion;
    // float mode;
    // float forward0;//22
    // float speed;//950
    // float P_turn;//0.120

    // float high_bridge_speed;//350
    // float short_bridge_speed;//350
    // float bridge_location;//9
    // float distance_crrossing;//15
    // float P_bridge_err;//0.6
    // float P_pitch;//1.0
    // float jiansu_speed;

    // float zhangai;
    // float podao;
    // float podao_distance;
    // float podao_jiansu_distance;
    // float small_err;

    // float zebra;
    // float podao_speed;
    // float zebra_jiasu;
    // float speed_decision;
    // float canshu9;
    float kp;
    float ki;
    float kd;
    int small_err;
    int picture_err;
    int forward;
    int debug_rgb_r_min;
    int debug_rgb_rb_diff;
    int debug_rgb_rg_diff;

};
extern struct FlashInformation Flash;

void Param_Init(void);
void Param_SaveSingle(const char* key, float value); // 注意：这里统一用 float 接收，整数会自动转小数
void Param_SaveAll(void);


#define SaveInt(key, val)   Param_SaveSingle(key, (float)(val))
#define SaveFloat(key, val) Param_SaveSingle(key, (float)(val))

#endif