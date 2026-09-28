// A real MPU6050 driver: found by its own identity (WHO_AM_I), woken and ranged at found(), polled for
// acceleration, rotation and temperature. Values are scaled to integers the capability's own `decimals` places:
// acceleration in mg (+-2 g), rotation in 0.1 deg/s (+-250 dps), temperature in 0.1 C.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/discover/registry.h>

namespace mpu {

  using discover::RowId;

  struct AccX { using Value = int16_t; static constexpr uint8_t id = 1, decimals = 3; static constexpr const char* name = "ax"; };
  struct AccY { using Value = int16_t; static constexpr uint8_t id = 2, decimals = 3; static constexpr const char* name = "ay"; };
  struct AccZ { using Value = int16_t; static constexpr uint8_t id = 3, decimals = 3; static constexpr const char* name = "az"; };
  struct GyrX { using Value = int16_t; static constexpr uint8_t id = 4, decimals = 1; static constexpr const char* name = "gx"; };
  struct GyrY { using Value = int16_t; static constexpr uint8_t id = 5, decimals = 1; static constexpr const char* name = "gy"; };
  struct GyrZ { using Value = int16_t; static constexpr uint8_t id = 6, decimals = 1; static constexpr const char* name = "gz"; };
  struct Temp { using Value = int16_t; static constexpr uint8_t id = 7, decimals = 1; static constexpr const char* name = "temp"; };

  template<typename W>
  struct Mpu6050 : discover::DriverBase<Mpu6050<W>, W> {
    using B = discover::DriverBase<Mpu6050<W>, W>;
    using Produces = hapi::Chain<AccX, AccY, AccZ, GyrX, GyrY, GyrZ, Temp>;
    static constexpr uint8_t addrLo = 0x68, addrHi = 0x69, id = 0x68, idReg = 0x75;   // AD0 low / high; WHO_AM_I

    static void writeReg(RowId row, uint8_t reg, uint8_t v) {
      using Twi = typename W::Twi;
      Twi::begin_write(B::addrOf(row)); Twi::write_byte(reg); Twi::write_byte(v); Twi::end_write();
    }
    // wake (it powers up asleep), internal clock, +-250 dps, +-2 g
    static void init(RowId row) { writeReg(row, 0x6B, 0x00); writeReg(row, 0x1B, 0x00); writeReg(row, 0x1C, 0x00); }

    static void read(RowId row) {
      uint8_t b[14] = {};
      B::readRegs(B::addrOf(row), 0x3B, b, 14);
      auto w = [&](uint8_t i) { return int16_t(uint16_t(uint16_t(b[i]) << 8) | b[i + 1]); };
      B::template emit<AccX>(row, int16_t((int32_t(w(0)) * 125) >> 11));
      B::template emit<AccY>(row, int16_t((int32_t(w(2)) * 125) >> 11));
      B::template emit<AccZ>(row, int16_t((int32_t(w(4)) * 125) >> 11));
      B::template emit<Temp>(row, int16_t(((int32_t(w(6)) * 1927) >> 16) + 365));
      B::template emit<GyrX>(row, int16_t((int32_t(w(8))  * 625) >> 13));
      B::template emit<GyrY>(row, int16_t((int32_t(w(10)) * 625) >> 13));
      B::template emit<GyrZ>(row, int16_t((int32_t(w(12)) * 625) >> 13));
    }
  };

}
