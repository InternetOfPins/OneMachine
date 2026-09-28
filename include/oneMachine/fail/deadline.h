// One deadline value for Detect (elapsed) and Gate (interval): time is an argument, never read here.
// Wrap-safe for intervals below 2^31 ms and a poll within 2^31 ms of the deadline. Unarmed is never due.
#pragma once
#include <stdint.h>

namespace fail {

  struct Deadline {
    uint32_t at    = 0;
    bool     armed = false;

    constexpr void arm(uint32_t now, uint32_t ms) { at = now + ms; armed = true; }
    constexpr void disarm()                       { armed = false; }
    constexpr bool due(uint32_t now) const        { return armed && int32_t(now - at) >= 0; }
  };

}
