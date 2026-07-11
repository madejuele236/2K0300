#pragma once

#include "zf_common_typedef.hpp"

#include <cstdint>

namespace primer::port::presentation {

struct DigitalKey {
    unsigned index;
    uint8 get_level() const;
};

struct AnalogKey {
    unsigned index;
    uint16 convert() const;
};

struct Display {
    void clear() const;
    void show_gray_image(uint16 x, uint16 y, const uint8 *image, uint16 width,
                         uint16 height, uint16 display_width,
                         uint16 display_height, uint8 threshold) const;
    void show_string(uint16 x, uint16 y, const char *text) const;
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

constexpr DigitalKey DigitalKeyAt(unsigned index) { return {index}; }
constexpr AnalogKey AnalogKeyAt(unsigned index) { return {index}; }
constexpr Display OperatorDisplay() { return {}; }
void SetEscDuty(int duty);
int8_t RunFlag();
void SetRunFlag(int8_t value);
void SaveParameters();
ParameterLiveView ObserveParameters();
ImuLiveView ObserveImu();
const volatile int16 &ObserveDistanceRaw();
TelemetryLiveView ObserveTelemetry();
int DistanceToRow(float distance);

}  // namespace primer::port::presentation
