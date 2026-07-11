#pragma once

#include "control/pid_controller.h"
#include "estimation/imu_estimator.h"
#include "platform/camera/camera_service.hpp"
#include "platform/device_platform.h"
#include "runtime/runtime_state.hpp"
#include "runtime/internal/legacy_math_macros.hpp"

// Public-command adapters for the token-identical lifecycle routines.
namespace {

struct LegacyMotorDriverBinding {
    unsigned index;
    void set_duty(int duty) const
    {
        primer::platform::SetMotorDriverDuty(index, duty);
    }
    void get_dev_info(pwm_info *) const
    {
        primer::platform::CaptureMotorDriverInfo(index);
    }
};

struct LegacyPitBinding {
    void stop() const { primer::platform::StopPeriodicTimer(); }
};

struct LegacyEscBinding {
    void set_duty(int duty) const { primer::platform::SetEscDuty(duty); }
    void get_dev_info(pwm_info *) const { primer::platform::CaptureEscInfo(); }
};

struct LegacyCameraBinding {
    void stop_collect() const { primer::platform::StopCamera(); }
};

struct LegacyImuBinding {
    void init() const { primer::platform::InitializeImu(); }
};

struct LegacyDistanceBinding {
    void init() const { primer::platform::InitializeDistanceSensor(); }
};

struct LegacyDisplayBinding {
    void init(const char *path) const { primer::platform::InitializeDisplay(path); }
};

struct LegacyEncoderDeviceBinding {
    unsigned index;
    int32_t get_count() const { return primer::platform::ReadEncoderCount(index); }
    void clear_count() const { primer::platform::ClearEncoderCount(index); }
};

struct LegacyRunFlagBinding {
    operator int8_t() const { return primer::runtime::RunFlag(); }
    LegacyRunFlagBinding &operator=(int8_t value)
    {
        primer::runtime::SetRunFlag(value);
        return *this;
    }
};

static constexpr LegacyMotorDriverBinding drv8701e_pwm_1{0};
static constexpr LegacyMotorDriverBinding drv8701e_pwm_2{1};
static pwm_info drv8701e_pwm_1_info{};
static pwm_info drv8701e_pwm_2_info{};
static constexpr LegacyPitBinding pit_timer{};
static constexpr LegacyEscBinding esc_pwm{};
static pwm_info esc_info{};
static constexpr LegacyCameraBinding cam{};
static constexpr LegacyImuBinding imu_dev{};
static constexpr LegacyDistanceBinding dl1x_dev{};
static constexpr LegacyDisplayBinding ips200{};
static constexpr LegacyEncoderDeviceBinding encoder_dir_1{0};
static constexpr LegacyEncoderDeviceBinding encoder_dir_2{1};
[[maybe_unused]] static LegacyRunFlagBinding run_flag{};

}  // namespace

#define PRIMER_ENCODER_CONTROL() (primer::control::AccessEncoderControlState())
#define PRIMER_RUNTIME_COUNTERS() (primer::runtime::AccessRuntimeCounters())
#define encoder_L (primer::platform::LeftEncoder())
#define encoder_R (primer::platform::RightEncoder())
#define Master_Speed (PRIMER_ENCODER_CONTROL().master_speed)
#define Now_Speed (PRIMER_ENCODER_CONTROL().current_speed)
#define Tpm_Dis (PRIMER_ENCODER_CONTROL().encoder_speed_difference)
#define G_dis (PRIMER_ENCODER_CONTROL().gyro_speed_difference)
#define Dis_Speed (PRIMER_ENCODER_CONTROL().fused_speed_difference)
#define last_G_dis (PRIMER_ENCODER_CONTROL().last_gyro_speed_difference)
#define LPF_Speed (primer::runtime::SpeedFusionWeight())
#define encode_l_total (PRIMER_RUNTIME_COUNTERS().left_encoder_total)
#define encode_r_total (PRIMER_RUNTIME_COUNTERS().right_encoder_total)
#define encoder_abs (PRIMER_RUNTIME_COUNTERS().encoder_distance)
#define icm_data (primer::estimation::CurrentImuEstimate())
