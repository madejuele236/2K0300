#pragma once

#include <cstdint>

namespace primer::runtime {
struct RuntimeTelemetry {
    const int32_t &left_encoder_total;
    const int32_t &right_encoder_total;
    const int16_t &encoder_distance;
};

struct RuntimeCounters {
    int32_t &left_encoder_total;
    int32_t &right_encoder_total;
    int16_t &encoder_distance;
};

int8_t RunFlag();
void SetRunFlag(int8_t value);
int32_t &CycleCounter();
float &SpeedFusionWeight();
int32_t LeftEncoderTotal();
int32_t RightEncoderTotal();
int16_t EncoderDistance();
RuntimeTelemetry ObserveRuntimeTelemetry();
RuntimeCounters AccessRuntimeCounters();
}  // namespace primer::runtime
