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

#define PRIMER_APPLICATION_CONTROL() (primer::control::AccessRuntimeControlState())
#define PRIMER_APPLICATION_VISION() (primer::vision::ObserveVisionControlLiveView())
#define Dis_1 (PRIMER_APPLICATION_CONTROL().distance_controller)
#define Velocity_L (PRIMER_APPLICATION_CONTROL().left_velocity_controller)
#define Velocity_R (PRIMER_APPLICATION_CONTROL().right_velocity_controller)
#define speed_goal (PRIMER_APPLICATION_CONTROL().speed_goal)
#define Dis_Out (PRIMER_APPLICATION_CONTROL().distance_output)
#define Dis_Speed (PRIMER_APPLICATION_CONTROL().distance_speed)
#define Image_out (PRIMER_APPLICATION_CONTROL().image_output)
#define v_left_target (PRIMER_APPLICATION_CONTROL().left_velocity_target)
#define v_right_target (PRIMER_APPLICATION_CONTROL().right_velocity_target)
#define PWM_L (PRIMER_APPLICATION_CONTROL().left_pwm)
#define PWM_R (PRIMER_APPLICATION_CONTROL().right_pwm)
#define Master_Speed (PRIMER_APPLICATION_CONTROL().master_speed)
#define encoder_L (primer::platform::LeftEncoder())
#define encoder_R (primer::platform::RightEncoder())
#define it_time (primer::runtime::CycleCounter())
#define dl1x_distance_raw (primer::platform::MutableDistanceRawSignal())
#define icm_data (primer::estimation::CurrentImuEstimate())
#define Flag (PRIMER_APPLICATION_VISION().elements)
#define imgInfo (PRIMER_APPLICATION_VISION().image)
#define L_h_guai (PRIMER_APPLICATION_VISION().left_high_corner)
#define R_h_guai (PRIMER_APPLICATION_VISION().right_high_corner)
#define real_distance (PRIMER_APPLICATION_VISION().row_distance)
#define Dir_err (PRIMER_APPLICATION_VISION().direction_error)
#define D_ERR (PRIMER_APPLICATION_VISION().steering_difference_error)
#define jump_point (PRIMER_APPLICATION_VISION().jump_point)
