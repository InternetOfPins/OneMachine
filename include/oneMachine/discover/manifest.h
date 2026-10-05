#pragma once
// oneMachine/discover/manifest.h -- what a driver declares about its part, beside the driver: the bus it speaks, its addresses and ids, its pins and
// who drives each one. A wiring is checked against it (the strap rule below; python/onemachine/wiring.py reads it through a host program), and the
// description's wiring lines carry it (OneMachine Redrawn, experiment 6).
//   Drive::Mcu      the MCU drives the line (a chip select, a reset, SCL with one master): safe on a boot strap pin while the MCU holds it at the
//                   level the strap needs, as a chip select idles high
//   Drive::Device   the part drives it (an IRQ, MISO): it can hold the line at reset, when the MCU is not running yet
//   Drive::Shared   both (open-drain SDA): a part can hold it low across a reset of the MCU
// The bus lines' roles (I2C, SPI) are here too for now; they belong with the buses (OneBus).
#include <stdint.h>

namespace discover {

  enum class Drive : uint8_t { Mcu, Device, Shared };
  constexpr const char* driveName(Drive d) { return d == Drive::Mcu ? "mcu" : d == Drive::Device ? "device" : "shared"; }

  // a line on a pin is safe at reset when the MCU drives it, or when the pin is not one the chip samples at reset
  template<class Board> constexpr bool safeAtReset(uint8_t gpio, Drive d) { return d == Drive::Mcu || !Board::strap(gpio); }

  struct I2cLines { static constexpr Drive sda = Drive::Shared, scl = Drive::Mcu; };
  struct SpiLines { static constexpr Drive sck = Drive::Mcu, miso = Drive::Device, mosi = Drive::Mcu, cs = Drive::Mcu; };

}
