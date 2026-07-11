#pragma once

#include "../vision_stage_contracts.hpp"
#include "../../../port/vision_stage_ports.hpp"

extern int Left_Sideline[LCDH_0];
extern int Right_Sideline[LCDH_0];
extern YuanSu Flag;
extern imageInformation imgInfo;
extern float real_distance[60];
extern float curvature;
extern float B;
extern float A;
extern uint16_t maxkuan_line;

static primer::port::VisionDirectionControllerPort &Image =
    primer::port::VisionDirectionController();
static float &Image_out = primer::port::MutableVisionImageOutput();

inline float Image_PID_Calculate(
    primer::port::VisionDirectionControllerPort *controller, float expect,
    float feedback)
{
    return primer::port::CalculateVisionDirection(controller, expect, feedback);
}

struct SteeringMasterSpeedBinding {
    operator float() const { return primer::port::VisionMasterSpeed(); }
};

static constexpr SteeringMasterSpeedBinding Master_Speed{};
