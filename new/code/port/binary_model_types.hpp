#ifndef LS2K_PORT_BINARY_MODEL_TYPES_HPP
#define LS2K_PORT_BINARY_MODEL_TYPES_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace ls2k::port {

inline constexpr int kBinaryModelScale = 16;
inline constexpr int kBinaryIlluminationColumns = 20;
inline constexpr int kBinaryIlluminationRows = 15;
inline constexpr int kBinaryResidualLumaScale = 10;
inline constexpr int kBinaryDefaultIlluminationWeight = 5;
inline constexpr int kBinaryMaximumIlluminationWeight = 9;
inline constexpr std::size_t kBinaryIlluminationSampleCount =
    static_cast<std::size_t>(kBinaryIlluminationColumns *
                             kBinaryIlluminationRows);

enum class BinaryModelSource : std::uint8_t {
    kNone = 0,
    kCurrent,
    kCached,
};

struct BinaryModelState {
    bool valid = false;
    int residual_threshold = 0;
    int illumination_weight = kBinaryDefaultIlluminationWeight;
    BinaryModelSource source = BinaryModelSource::kNone;
    std::uint8_t stale_frames = 0U;
    std::array<std::uint8_t, kBinaryIlluminationSampleCount> illumination{};
};

inline const char* ToString(BinaryModelSource source) {
    switch (source) {
        case BinaryModelSource::kNone:
            return "none";
        case BinaryModelSource::kCurrent:
            return "current";
        case BinaryModelSource::kCached:
            return "cached";
    }
    return "none";
}

}  // namespace ls2k::port

#endif  // LS2K_PORT_BINARY_MODEL_TYPES_HPP
