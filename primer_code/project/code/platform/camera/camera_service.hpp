#pragma once

#include "platform/camera/lq_camera_ex.hpp"

// Runtime-owned camera instance.  The composition root defines it in the same
// translation unit and relative order as the original image.cpp globals.
extern lq_camera_ex cam;
