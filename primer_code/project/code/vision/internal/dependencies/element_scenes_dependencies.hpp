#pragma once

#include "../../vision_facts.hpp"
#include "../legacy_vision_constants.hpp"
#include "../../../port/vision_stage_ports.hpp"

#include <vector>

extern uint8_t Image_Use[LCDH_1][LCDW_1];
extern int Left_Sideline[LCDH_0], Right_Sideline[LCDH_0];
extern unsigned char Left_Sideline_flag[LCDH_0];
extern unsigned char Right_Sideline_flag[LCDH_0];
extern std::vector<RedObject> red_objects;
extern Guaidian L_h_guai, R_h_guai;
extern YuanSu Flag;
extern imageInformation imgInfo;
extern cv::Mat resizedFrame;
extern float Dir_Err[60];
extern float real_distance[60];
extern float ramp_err;
extern float err_picture;
extern int red_x_mid, red_y_mid, x_err_red;
extern int red_left, red_right;
extern int red_find_x, red_find_y, red_find_y1;
extern int resize_cx, resize_cy;

int real_distance_to_row(float distance);
float xielv_sideline(int x1, int y1, int x2, int y2, char data);
void DetectRedBlock(cv::Mat &src, int roi_x, int roi_y, int width, int height);

struct LegacyEscBinding {
    void set_duty(int duty) const { primer::port::SetVisionEscDuty(duty); }
};

struct LegacyRunFlagBinding {
    LegacyRunFlagBinding &operator=(int8_t value)
    {
        primer::port::SetVisionRunMode(value);
        return *this;
    }
};

struct LegacyDistanceRawBinding {
    operator int16_t() const { return primer::port::VisionDistanceRaw(); }
};

static constexpr LegacyEscBinding esc_pwm{};
static LegacyRunFlagBinding run_flag{};
static constexpr LegacyDistanceRawBinding dl1x_distance_raw{};

#define encoder_L (primer::port::VisionLeftEncoder())
#define encoder_R (primer::port::VisionRightEncoder())
