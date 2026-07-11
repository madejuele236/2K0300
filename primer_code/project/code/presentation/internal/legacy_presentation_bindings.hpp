#pragma once

#include "port/presentation_ports.hpp"
#include "port/vision_observation.hpp"

// The original presentation routines intentionally remain token-identical.
// These TU-local bindings translate their historical names to owner APIs;
// no mutable owner state is published through a public header.
namespace {

static constexpr auto key_1 = primer::port::presentation::DigitalKeyAt(0);
static constexpr auto key_2 = primer::port::presentation::DigitalKeyAt(1);
static constexpr auto key_3 = primer::port::presentation::DigitalKeyAt(2);
static constexpr auto key_4 = primer::port::presentation::DigitalKeyAt(3);
static constexpr auto key_5 = primer::port::presentation::DigitalKeyAt(4);
static constexpr auto key_6 = primer::port::presentation::DigitalKeyAt(5);
static constexpr auto key_7 = primer::port::presentation::AnalogKeyAt(0);
static constexpr auto key_8 = primer::port::presentation::AnalogKeyAt(1);
static constexpr auto ips200 = primer::port::presentation::OperatorDisplay();

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

inline void Param_SaveAll()
{
    primer::port::presentation::SaveParameters();
}

inline int real_distance_to_row(float distance)
{
    return primer::port::presentation::DistanceToRow(distance);
}

}  // namespace

// Private compatibility spellings: each owner view is obtained only when the
// corresponding legacy expression is evaluated in show.cpp.
#define Flash (primer::port::presentation::ObserveParameters())
#define icm_data (primer::port::presentation::ObserveImu())
#define dl1x_distance_raw (primer::port::presentation::ObserveDistanceRaw())
#define encode_l_total (primer::port::presentation::ObserveTelemetry().left_encoder_total)
#define encode_r_total (primer::port::presentation::ObserveTelemetry().right_encoder_total)
#define encoder_abs (primer::port::presentation::ObserveTelemetry().encoder_distance)
#define Image_Use (primer::port::vision::ObserveBinaryImage())
#define Left_Sideline (primer::port::vision::ObserveLeftSideline())
#define Right_Sideline (primer::port::vision::ObserveRightSideline())
#define Mid_Line (primer::port::vision::ObserveMidline())
#define Flag (primer::port::vision::ObserveElementFacts())
#define imgInfo (primer::port::vision::ObserveImageInformation())
#define L_l_guai (primer::port::vision::ObserveLeftLowCorner())
#define L_h_guai (primer::port::vision::ObserveLeftHighCorner())
#define R_l_guai (primer::port::vision::ObserveRightLowCorner())
#define R_h_guai (primer::port::vision::ObserveRightHighCorner())
#define L_h_guai1 (primer::port::vision::ObserveLeftHighCornerSecondary())
#define R_h_guai1 (primer::port::vision::ObserveRightHighCornerSecondary())
#define real_distance (primer::port::vision::ObserveRowDistance())
#define resizedFrame (primer::port::vision::ObserveResizedFrame())
#define Dir_err (primer::port::vision::ObserveDirectionError())
#define distance (primer::port::vision::ObserveDistance())
#define Yaw_Huandao_err (primer::port::vision::ObserveRoundaboutYawError())
#define black_ratio (primer::port::vision::ObserveBlackRatio())
#define jump_point (primer::port::vision::ObserveJumpPoint())
#define maxlong_colume (primer::port::vision::ObserveMaxlongColumn())
#define long_max (primer::port::vision::ObserveLongMax())
#define jump_point1 (primer::port::vision::ObserveJumpPointSecondary())
#define picture_white (primer::port::vision::ObservePictureWhite())
#define picture_black (primer::port::vision::ObservePictureBlack())
#define red_find_x (primer::port::vision::ObserveRedFindX())
#define red_find_y (primer::port::vision::ObserveRedFindY())
