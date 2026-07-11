#pragma once

#include "estimation/imu_estimator.h"
#include "parameters/parameter_store.h"
#include "platform/device_platform.h"
#include "runtime/runtime_state.hpp"
#include "vision/vision_queries.hpp"

// The original presentation routines intentionally remain token-identical.
// These TU-local bindings translate their historical names to owner APIs;
// no mutable owner state is published through a public header.
namespace {

const primer::vision::VisionPresentationFacts presentation_facts =
    primer::vision::ObserveVisionPresentationFacts();
const primer::runtime::RuntimeTelemetry runtime_telemetry =
    primer::runtime::ObserveRuntimeTelemetry();

static auto &key_1 = primer::platform::DigitalKey(0);
static auto &key_2 = primer::platform::DigitalKey(1);
static auto &key_3 = primer::platform::DigitalKey(2);
static auto &key_4 = primer::platform::DigitalKey(3);
static auto &key_5 = primer::platform::DigitalKey(4);
static auto &key_6 = primer::platform::DigitalKey(5);
static auto &key_7 = primer::platform::AnalogKey(0);
static auto &key_8 = primer::platform::AnalogKey(1);
static auto &ips200 = primer::platform::Display();

struct LegacyEscBinding {
    void set_duty(int duty) const { primer::platform::SetEscDuty(duty); }
};

struct LegacyRunFlagBinding {
    operator int8_t() const { return primer::runtime::RunFlag(); }
    LegacyRunFlagBinding &operator=(int8_t value)
    {
        primer::runtime::SetRunFlag(value);
        return *this;
    }
};

static constexpr LegacyEscBinding esc_pwm{};
static LegacyRunFlagBinding run_flag{};
static FlashInformation &Flash = primer::parameters::MutableParameters();
static const icm_param_t &icm_data = primer::estimation::CurrentImuEstimate();
static const volatile int16 &dl1x_distance_raw =
    primer::platform::DistanceRawSignal();

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
