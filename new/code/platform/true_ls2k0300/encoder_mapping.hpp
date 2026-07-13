#pragma once

#include <cstdint>

namespace ls2k::platform::true_ls2k0300 {

// Board-level polarity contract established by the LS2K0300 wheel/encoder
// wiring: logical forward keeps the raw left count and reverses the raw right
// count. The MotorDevice owns converting logical drive polarity to hardware.
inline constexpr int kLeftEncoderDirectionSign = 1;
inline constexpr int kRightEncoderDirectionSign = -1;

struct LogicalEncoderCounts final {
    std::int32_t left{0};
    std::int32_t right{0};
};

[[nodiscard]] constexpr LogicalEncoderCounts NormalizeEncoderCounts(
    std::int32_t raw_left,
    std::int32_t raw_right) noexcept {
    return {
        raw_left * kLeftEncoderDirectionSign,
        raw_right * kRightEncoderDirectionSign,
    };
}

}  // namespace ls2k::platform::true_ls2k0300
