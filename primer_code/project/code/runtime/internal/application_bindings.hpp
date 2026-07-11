#pragma once

#include "control/pid_controller.h"
#include "estimation/imu_estimator.h"
#include "platform/device_platform.h"
#include "runtime/runtime_state.hpp"
#include "vision/vision_queries.hpp"
#include "runtime/internal/legacy_math_macros.hpp"

// Private compatibility bindings for the token-identical application loop.
// Every cross-owner name below resolves through a public query or command.
namespace {

const primer::control::RuntimeControlState application_control =
    primer::control::AccessRuntimeControlState();
const primer::vision::VisionControlLiveView application_vision =
    primer::vision::ObserveVisionControlLiveView();

static auto &Dis_1 = application_control.distance_controller;
static auto &Velocity_L = application_control.left_velocity_controller;
static auto &Velocity_R = application_control.right_velocity_controller;
static auto &speed_goal = application_control.speed_goal;
static auto &Dis_Out = application_control.distance_output;
static auto &Dis_Speed = application_control.distance_speed;
static auto &Image_out = application_control.image_output;
static auto &v_left_target = application_control.left_velocity_target;
static auto &v_right_target = application_control.right_velocity_target;
static auto &PWM_L = application_control.left_pwm;
static auto &PWM_R = application_control.right_pwm;
static const auto &Master_Speed = application_control.master_speed;

static auto &encoder_L = primer::platform::LeftEncoder();
static auto &encoder_R = primer::platform::RightEncoder();
static auto &it_time = primer::runtime::CycleCounter();
static volatile int16 &dl1x_distance_raw =
    primer::platform::MutableDistanceRawSignal();
static const auto &icm_data = primer::estimation::CurrentImuEstimate();

static const auto &Flag = application_vision.elements;
static const auto &imgInfo = application_vision.image;
static const auto &L_h_guai = application_vision.left_high_corner;
static const auto &R_h_guai = application_vision.right_high_corner;
static const auto &real_distance = application_vision.row_distance;
static const auto &Dir_err = application_vision.direction_error;
static const auto &D_ERR = application_vision.steering_difference_error;
static const auto &jump_point = application_vision.jump_point;

struct LegacyDistanceSensorBinding {
    int16 get_distance() const { return primer::platform::ReadDistanceSensor(); }
};

struct LegacyPitBinding {
    void init_ms(uint32_t milliseconds, void (*callback)(void)) const
    {
        primer::platform::StartPeriodicTimer(milliseconds, callback);
    }
};

struct LegacyBeeperBinding {
    void set_level(int level) const { primer::platform::SetBeeper(level != 0); }
};

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

struct LegacyDynamicForwardBinding {
    operator int() const { return primer::vision::VisionDynamicForward(); }
    LegacyDynamicForwardBinding &operator=(int value)
    {
        primer::vision::SetVisionDynamicForward(value);
        return *this;
    }
};

static constexpr LegacyDistanceSensorBinding dl1x_dev{};
static constexpr LegacyPitBinding pit_timer{};
static constexpr LegacyBeeperBinding beep{};
static constexpr LegacyEscBinding esc_pwm{};
[[maybe_unused]] static LegacyRunFlagBinding run_flag{};
static LegacyDynamicForwardBinding forward1{};

}  // namespace
