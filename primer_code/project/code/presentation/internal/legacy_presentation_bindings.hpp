#pragma once

#include "port/presentation_ports.hpp"

// The original presentation routines intentionally remain token-identical.
// These TU-local bindings translate their historical names to owner APIs;
// no mutable owner state is published through a public header.
namespace {

const auto presentation_facts = primer::port::presentation::ObserveVision();
const auto runtime_telemetry = primer::port::presentation::ObserveTelemetry();

static const auto key_1 = primer::port::presentation::DigitalKeyAt(0);
static const auto key_2 = primer::port::presentation::DigitalKeyAt(1);
static const auto key_3 = primer::port::presentation::DigitalKeyAt(2);
static const auto key_4 = primer::port::presentation::DigitalKeyAt(3);
static const auto key_5 = primer::port::presentation::DigitalKeyAt(4);
static const auto key_6 = primer::port::presentation::DigitalKeyAt(5);
static const auto key_7 = primer::port::presentation::AnalogKeyAt(0);
static const auto key_8 = primer::port::presentation::AnalogKeyAt(1);
static const auto ips200 = primer::port::presentation::OperatorDisplay();

struct LegacyEscBinding {
    void set_duty(int duty) const { primer::port::presentation::SetEscDuty(duty); }
};

struct LegacyRunFlagBinding {
    operator int8_t() const { return primer::port::presentation::RunFlag(); }
    LegacyRunFlagBinding &operator=(int8_t value)
    {
        primer::port::presentation::SetRunFlag(value);
        return *this;
    }
};

static constexpr LegacyEscBinding esc_pwm{};
static LegacyRunFlagBinding run_flag{};
static auto Flash = primer::port::presentation::ObserveParameters();
static const auto icm_data = primer::port::presentation::ObserveImu();
static const volatile int16 &dl1x_distance_raw =
    primer::port::presentation::ObserveDistanceRaw();

inline void Param_SaveAll()
{
    primer::port::presentation::SaveParameters();
}

inline int real_distance_to_row(float distance)
{
    return primer::port::presentation::DistanceToRow(distance);
}

static const int32_t &encode_l_total = runtime_telemetry.left_encoder_total;
static const int32_t &encode_r_total = runtime_telemetry.right_encoder_total;
static const int16_t &encoder_abs = runtime_telemetry.encoder_distance;

static const auto &Image_Use = presentation_facts.binary_image;
static const auto &Left_Sideline = presentation_facts.left_sideline;
static const auto &Right_Sideline = presentation_facts.right_sideline;
static const auto &Mid_Line = presentation_facts.midline;
static const auto &Flag = presentation_facts.elements;
static const auto &imgInfo = presentation_facts.image;
static const auto &L_l_guai = presentation_facts.left_low_corner;
static const auto &L_h_guai = presentation_facts.left_high_corner;
static const auto &R_l_guai = presentation_facts.right_low_corner;
static const auto &R_h_guai = presentation_facts.right_high_corner;
static const auto &L_h_guai1 = presentation_facts.left_high_corner_secondary;
static const auto &R_h_guai1 = presentation_facts.right_high_corner_secondary;
static const auto &real_distance = presentation_facts.row_distance;
static const auto &resizedFrame = presentation_facts.resized_frame;
static const auto &Dir_err = presentation_facts.direction_error;
static const auto &distance = presentation_facts.distance;
static const auto &Yaw_Huandao_err = presentation_facts.roundabout_yaw_error;
static const auto &black_ratio = presentation_facts.black_ratio;
static const auto &jump_point = presentation_facts.jump_point;
static const auto &maxlong_colume = presentation_facts.maxlong_column;
static const auto &long_max = presentation_facts.long_max;
static const auto &jump_point1 = presentation_facts.jump_point_secondary;
static const auto &picture_white = presentation_facts.picture_white;
static const auto &picture_black = presentation_facts.picture_black;
static const auto &red_find_x = presentation_facts.red_find_x;
static const auto &red_find_y = presentation_facts.red_find_y;

}  // namespace
