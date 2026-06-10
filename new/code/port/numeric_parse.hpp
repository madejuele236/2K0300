#pragma once

#include <charconv>
#include <optional>
#include <string_view>

namespace ls2k::port {

inline std::optional<int> ParseIntStrict(std::string_view text) {
    if (text.empty()) {
        return std::nullopt;
    }
    int value = 0;
    const char* begin = text.data();
    const char* end = begin + text.size();
    const std::from_chars_result result = std::from_chars(begin, end, value);
    if (result.ec != std::errc{} || result.ptr != end) {
        return std::nullopt;
    }
    return value;
}

inline std::optional<int> ParsePositiveIntStrict(std::string_view text) {
    const std::optional<int> value = ParseIntStrict(text);
    if (!value.has_value() || *value <= 0) {
        return std::nullopt;
    }
    return value;
}

}  // namespace ls2k::port
