#include "port/presentation_ports.hpp"

#include "estimation/imu_estimator.h"
#include "parameters/parameter_store.h"
#include "platform/device_platform.h"
#include "runtime/runtime_state.hpp"
#include "vision/vision_queries.hpp"

namespace primer::port::presentation {
namespace {

uint8 ReadDigitalKey(unsigned index)
{
    return primer::platform::DigitalKey(index).get_level();
}

uint16 ReadAnalogKey(unsigned index)
{
    return primer::platform::AnalogKey(index).convert();
}

void ClearDisplay()
{
    primer::platform::Display().clear();
}

void ShowGrayImage(uint16 x, uint16 y, const uint8 *image, uint16 width,
                   uint16 height, uint16 display_width, uint16 display_height,
                   uint8 threshold)
{
    primer::platform::Display().show_gray_image(
        x, y, image, width, height, display_width, display_height, threshold);
}

void ShowString(uint16 x, uint16 y, const char *value)
{
    primer::platform::Display().show_string(x, y, value);
}

}  // namespace

uint8 DigitalKey::get_level() const
{
    return ReadDigitalKey(index);
}

uint16 AnalogKey::convert() const
{
    return ReadAnalogKey(index);
}

void Display::clear() const
{
    ClearDisplay();
}

void Display::show_gray_image(uint16 x, uint16 y, const uint8 *image,
                              uint16 width, uint16 height,
                              uint16 display_width, uint16 display_height,
                              uint8 threshold) const
{
    ShowGrayImage(x, y, image, width, height, display_width, display_height,
                  threshold);
}

void Display::show_string(uint16 x, uint16 y, const char *text) const
{
    ShowString(x, y, text);
}

void SetEscDuty(int duty)
{
    primer::platform::SetEscDuty(duty);
}

int8_t RunFlag()
{
    return primer::runtime::RunFlag();
}

void SetRunFlag(int8_t value)
{
    primer::runtime::SetRunFlag(value);
}

void SaveParameters()
{
    Param_SaveAll();
}

ParameterLiveView ObserveParameters()
{
    auto &parameters = primer::parameters::MutableParameters();
    return {parameters.debug_rgb_r_min, parameters.debug_rgb_rb_diff,
            parameters.debug_rgb_rg_diff};
}

ImuLiveView ObserveImu()
{
    const auto &imu = primer::estimation::CurrentImuEstimate();
    return {imu.gyro_y, imu.gyro_z, imu.yaw};
}

const volatile int16 &ObserveDistanceRaw()
{
    return primer::platform::DistanceRawSignal();
}

TelemetryLiveView ObserveTelemetry()
{
    const auto telemetry = primer::runtime::ObserveRuntimeTelemetry();
    return {telemetry.left_encoder_total, telemetry.right_encoder_total,
            telemetry.encoder_distance};
}

int DistanceToRow(float distance)
{
    return real_distance_to_row(distance);
}

}  // namespace primer::port::presentation
