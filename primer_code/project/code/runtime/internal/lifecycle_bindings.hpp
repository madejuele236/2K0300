#pragma once

#include "control/pid_controller.h"
#include "estimation/imu_estimator.h"
#include "platform/camera/camera_service.hpp"
#include "platform/device_platform.h"
#include "runtime/runtime_state.hpp"
#include "runtime/internal/legacy_math_macros.hpp"

// Public-command adapters for the token-identical lifecycle routines.
namespace {

const primer::control::EncoderControlState encoder_control =
    primer::control::AccessEncoderControlState();
const primer::runtime::RuntimeCounters runtime_counters =
    primer::runtime::AccessRuntimeCounters();

static auto &encoder_L = primer::platform::LeftEncoder();
static auto &encoder_R = primer::platform::RightEncoder();
static auto &Master_Speed = encoder_control.master_speed;
static auto &Now_Speed = encoder_control.current_speed;
static auto &Tpm_Dis = encoder_control.encoder_speed_difference;
static auto &G_dis = encoder_control.gyro_speed_difference;
static auto &Dis_Speed = encoder_control.fused_speed_difference;
static auto &last_G_dis = encoder_control.last_gyro_speed_difference;
static auto &LPF_Speed = primer::runtime::SpeedFusionWeight();
static auto &encode_l_total = runtime_counters.left_encoder_total;
static auto &encode_r_total = runtime_counters.right_encoder_total;
static auto &encoder_abs = runtime_counters.encoder_distance;
static const auto &icm_data = primer::estimation::CurrentImuEstimate();

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
