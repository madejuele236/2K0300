#pragma once

#include <cstdint>

namespace primer::port {

// Runtime implements this command; vision depends only on the port contract.
void SetRunMode(int8_t value);

}  // namespace primer::port
