#ifndef LS2K_PORT_OTSU_THRESHOLD_TYPES_HPP
#define LS2K_PORT_OTSU_THRESHOLD_TYPES_HPP

#include <cstdint>

namespace ls2k::port {

enum class OtsuThresholdSource : std::uint8_t {
    kNone = 0,
    kCurrent,
    kCached,
};

struct OtsuThresholdState {
    bool valid = false;
    int threshold = 0;
    OtsuThresholdSource source = OtsuThresholdSource::kNone;
    std::uint8_t stale_frames = 0U;
};

inline const char* ToString(OtsuThresholdSource source) {
    switch (source) {
        case OtsuThresholdSource::kNone:
            return "none";
        case OtsuThresholdSource::kCurrent:
            return "current";
        case OtsuThresholdSource::kCached:
            return "cached";
    }
    return "none";
}

}  // namespace ls2k::port

#endif  // LS2K_PORT_OTSU_THRESHOLD_TYPES_HPP
