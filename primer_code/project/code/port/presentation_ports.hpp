#pragma once

#include "port/vision_live_views.hpp"
#include "zf_common_typedef.hpp"

#include <cstdint>

namespace primer::port::presentation {

struct DigitalKey {
    uint8 (*read)(unsigned index);
    unsigned index;
    uint8 get_level() const { return read(index); }
};

struct AnalogKey {
    uint16 (*read)(unsigned index);
    unsigned index;
    uint16 convert() const { return read(index); }
};

struct Display {
    void (*clear_command)();
    void (*gray_image_command)(uint16, uint16, const uint8 *, uint16, uint16,
                               uint16, uint16, uint8);
    void (*string_command)(uint16, uint16, const char *);

    void clear() const { clear_command(); }
    void show_gray_image(uint16 x, uint16 y, const uint8 *image, uint16 width,
                         uint16 height, uint16 display_width,
                         uint16 display_height, uint8 threshold) const
    {
        gray_image_command(x, y, image, width, height, display_width,
                           display_height, threshold);
    }
    void show_string(uint16 x, uint16 y, const char *text) const
    {
        string_command(x, y, text);
    }
};

struct ParameterLiveView {
    int &debug_rgb_r_min;
    int &debug_rgb_rb_diff;
    int &debug_rgb_rg_diff;
};

struct ImuLiveView {
    const float &gyro_y;
    const float &gyro_z;
    const float &yaw;
};

struct TelemetryLiveView {
    const int32_t &left_encoder_total;
    const int32_t &right_encoder_total;
    const int16_t &encoder_distance;
};

DigitalKey DigitalKeyAt(unsigned index);
AnalogKey AnalogKeyAt(unsigned index);
Display OperatorDisplay();
void SetEscDuty(int duty);
int8_t RunFlag();
void SetRunFlag(int8_t value);
void SaveParameters();
ParameterLiveView ObserveParameters();
ImuLiveView ObserveImu();
const volatile int16 &ObserveDistanceRaw();
TelemetryLiveView ObserveTelemetry();
primer::port::vision::PresentationLiveView ObserveVision();
int DistanceToRow(float distance);

}  // namespace primer::port::presentation
