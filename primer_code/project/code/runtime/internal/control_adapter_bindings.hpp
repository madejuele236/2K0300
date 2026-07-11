#pragma once

#include "control/pid_controller.h"
#include "platform/device_platform.h"

namespace {
static auto &encoder_L = primer::platform::LeftEncoder();
static auto &encoder_R = primer::platform::RightEncoder();
}  // namespace
