#include "vision/ml/v9_descriptor.hpp"

#include <algorithm>
#include <array>
#include <cstdint>

namespace ls2k::vision::ml {
namespace {

constexpr std::int32_t kBasis[8][8] = {
    {11585,11585,11585,11585,11585,11585,11585,11585},
    {16069,13623,9102,3196,-3196,-9102,-13623,-16069},
    {15137,6270,-6270,-15137,-15137,-6270,6270,15137},
    {13623,-3196,-16069,-9102,9102,16069,3196,-13623},
    {11585,-11585,-11585,11585,11585,-11585,-11585,11585},
    {9102,-16069,3196,13623,-13623,-3196,16069,-9102},
    {6270,-15137,15137,-6270,-6270,15137,-15137,6270},
    {3196,-9102,13623,-16069,16069,-13623,9102,-3196}};

std::int64_t OddMedian(std::array<std::int64_t, 63> values) {
    std::nth_element(values.begin(), values.begin() + 31, values.end());
    return values[31];
}

}  // namespace

port::V9Descriptor BuildV9Descriptor(const port::MlGrayRoi32& roi) {
    port::V9Descriptor out{};
    if (!roi.valid) return out;
    std::int64_t patches[8][8]{};
    for (int py = 0; py < 8; ++py) {
        for (int px = 0; px < 8; ++px) {
            for (int y = 0; y < 4; ++y) {
                for (int x = 0; x < 4; ++x) {
                    const std::uint8_t gray = roi.gray[static_cast<std::size_t>(
                        (py * 4 + y) * port::kMlRoiSide + px * 4 + x)];
                    patches[py][px] +=
                        (static_cast<std::int64_t>(gray) * 65536 + 127) / 255;
                }
            }
        }
    }
    std::int64_t temp[8][8]{};
    for (int i = 0; i < 8; ++i)
        for (int k = 0; k < 8; ++k)
            for (int j = 0; j < 8; ++j)
                temp[i][k] += static_cast<std::int64_t>(kBasis[i][j]) * patches[j][k];
    std::array<std::int64_t, 63> coeff{};
    int cursor = 0;
    for (int i = 0; i < 8; ++i) {
        for (int l = 0; l < 8; ++l) {
            std::int64_t value = 0;
            for (int k = 0; k < 8; ++k)
                value += temp[i][k] * static_cast<std::int64_t>(kBasis[l][k]);
            if (i != 0 || l != 0) coeff[static_cast<std::size_t>(cursor++)] = value;
        }
    }
    std::array<std::int64_t, 63> absolute{};
    for (std::size_t i = 0; i < coeff.size(); ++i) absolute[i] = coeff[i] < 0 ? -coeff[i] : coeff[i];
    const std::int64_t abs_median = OddMedian(absolute);
    for (int i = 0; i < 63; ++i) {
        if (coeff[static_cast<std::size_t>(i)] > 0)
            out.bytes[static_cast<std::size_t>(i / 8)] |= static_cast<std::uint8_t>(1U << (i % 8));
        if (absolute[static_cast<std::size_t>(i)] > abs_median) {
            const int bit = 63 + i;
            out.bytes[static_cast<std::size_t>(bit / 8)] |= static_cast<std::uint8_t>(1U << (bit % 8));
        }
    }
    out.valid = true;
    return out;
}

}  // namespace ls2k::vision::ml
