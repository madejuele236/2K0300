#pragma once

#include "../../control/pid_controller.h"
#include "../../estimation/imu_estimator.h"
#include "../../inference/classifier_service.hpp"
#include "../../parameters/parameter_store.h"
#include "../../platform/camera/camera_service.hpp"
#include "../../platform/device_platform.h"
#include "../../runtime/runtime_state.hpp"
#include "../../transport/stream_service.hpp"

// TU-local compatibility bindings for the token-identical primer algorithms.
// State remains owned by, and is reached only through, each owner's public API.
static encoder &encoder_L = primer::platform::LeftEncoder();
static encoder &encoder_R = primer::platform::RightEncoder();
static Direction_PID &Image = primer::control::ImageController();
static float &Image_out = primer::control::MutableImageOutput();
static const icm_param_t &icm_data = primer::estimation::CurrentImuEstimate();
static const FlashInformation &Flash = primer::parameters::CurrentParameters();
static LQ_NCNN &classifier = primer::inference::Classifier();
static TransmissionStreamServer &camera_server =
    primer::transport::CameraStreamServer();
static lq_camera_ex &cam = primer::platform::Camera();

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

struct LegacyDistanceRawBinding {
    operator int16() const { return primer::platform::DistanceRaw(); }
};

struct LegacySpeedBinding {
    operator float() const { return primer::control::CurrentSpeed(); }
};

struct LegacyMasterSpeedBinding {
    operator float() const { return primer::control::MasterSpeed(); }
};

static constexpr LegacyEscBinding esc_pwm{};
[[maybe_unused]] static LegacyRunFlagBinding run_flag{};
static constexpr LegacyDistanceRawBinding dl1x_distance_raw{};
static constexpr LegacySpeedBinding Now_Speed{};
static constexpr LegacyMasterSpeedBinding Master_Speed{};
