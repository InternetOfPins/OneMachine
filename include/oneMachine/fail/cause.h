// OneBus's failure cause, seen as failCompose's outcome. OneBus does not know failCompose: the table lives here.
#pragma once
#include <oneBus/busAPI.h>
#include "outcome.h"

namespace fail {

  // Nack -> Absent, Timeout -> Timeout, BusError and ArbLost -> Fault (a single-master bus: ArbLost is a fault, not a retry),
  // Overflow -> Overflow, Unknown -> Unknown. The raw cause rides in `detail`. None is Ok.
  constexpr Outcome fromCause(oneBus::TwiCause c) {
    using C = oneBus::TwiCause;
    return c == C::None     ? Outcome::Ok()
         : c == C::Nack     ? Outcome::Fail(Kind::Absent,   uint8_t(c))
         : c == C::Timeout  ? Outcome::Fail(Kind::Timeout,  uint8_t(c))
         : c == C::BusError ? Outcome::Fail(Kind::Fault,    uint8_t(c))
         : c == C::ArbLost  ? Outcome::Fail(Kind::Fault,    uint8_t(c))
         : c == C::Overflow ? Outcome::Fail(Kind::Overflow, uint8_t(c))
         :                    Outcome::Fail(Kind::Unknown,  uint8_t(c));
  }

  // The bus's fault, not the address's (oneBus::isBusFault): it gates every device below that bus.
  constexpr bool busLevel(oneBus::TwiCause c) { return oneBus::isBusFault(c); }

}
