// The SPI access policy of the device edge (fail/devedge.h): a Mode names it with `template<typename Impl, typename W> using Access = fail::SpiAccess<Impl, W>;`.
// An SPI device has no ACK and the bus has no cause to report: a failed check is the device's. The reprobe reads the driver's ID command twice, at the
// driver's clock and mode, and wants an Ids match both times (the rule the scan uses); anything else is Absent, the byte read in the detail
// (0x00 and 0xFF are what a device held in reset or an empty slot give).
#pragma once
#include <stdint.h>
#include <oneMachine/discover/spi.h>
#include "outcome.h"

namespace fail {

  template<typename Impl, typename W>
  struct SpiAccess {
    static Outcome reprobe(RowId row) {
      W::route(W::reg.rows[row].parent);
      const uint8_t slot = uint8_t(W::reg.rows[row].busId);
      const uint8_t a = discover::spiIdByte<typename W::Bus, Impl>(slot), b = discover::spiIdByte<typename W::Bus, Impl>(slot);
      if (Impl::Ids::has(a) && a == b) return Outcome::Ok();
      return Outcome::Fail(Kind::Absent, Impl::Ids::has(a) ? b : a);
    }
  };

}
