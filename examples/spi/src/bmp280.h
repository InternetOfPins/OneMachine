// A BMP280/BME280 driver over I2C (temperature and pressure; a BME280's humidity is left out): found by its chip id at
// 0x76/0x77 (0x58 BMP280, 0x60 BME280: one entry each, the same driver), calibration read at found(), then normal mode
// (temperature x2, pressure x16, filter x4, 500 ms standby). Integer compensation from the datasheet (section 8.2).
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>

namespace bmp {

  using discover::RowId;

  struct Temp  { using Value = int32_t; static constexpr uint8_t id = 31, decimals = 2; static constexpr const char* name = "temp"; };
  struct Press { using Value = int32_t; static constexpr uint8_t id = 32, decimals = 2; static constexpr const char* name = "hPa"; };   // Pa, read as hPa

  template<typename W>
  struct Bmp280 : discover::DriverBase<Bmp280<W>, W> {
    using B = discover::DriverBase<Bmp280<W>, W>;
    using Produces = hapi::Chain<Temp, Press>;
    static constexpr uint8_t addrLo = 0x76, addrHi = 0x77, idReg = 0xD0, id = 0x58;

    struct DeviceState { uint16_t T1, P1; int16_t T2, T3, P2, P3, P4, P5, P6, P7, P8, P9; };

    static void writeReg(RowId row, uint8_t reg, uint8_t v) {
      using Twi = typename W::Twi;
      Twi::begin_write(B::addrOf(row)); Twi::write_byte(reg); Twi::write_byte(v); Twi::end_write();
    }

    static void init(RowId row) {
      uint8_t c[24] = {};
      B::readRegs(B::addrOf(row), 0x88, c, 24);
      auto u = [&](uint8_t i) { return uint16_t(c[i] | (uint16_t(c[i + 1]) << 8)); };
      auto s = [&](uint8_t i) { return int16_t(u(i)); };
      auto& d = B::dev(row);
      d.T1 = u(0); d.T2 = s(2); d.T3 = s(4);
      d.P1 = u(6); d.P2 = s(8); d.P3 = s(10); d.P4 = s(12); d.P5 = s(14); d.P6 = s(16); d.P7 = s(18); d.P8 = s(20); d.P9 = s(22);
      writeReg(row, 0xF5, 0x90);   // standby 500 ms, filter x4 (config before ctrl_meas: written in sleep mode)
      writeReg(row, 0xF4, 0x57);   // temperature x2, pressure x16, normal mode
    }

    static void read(RowId row) {
      uint8_t b[6] = {};
      B::readRegs(B::addrOf(row), 0xF7, b, 6);
      const int32_t adcP = int32_t((uint32_t(b[0]) << 12) | (uint32_t(b[1]) << 4) | (b[2] >> 4));
      const int32_t adcT = int32_t((uint32_t(b[3]) << 12) | (uint32_t(b[4]) << 4) | (b[5] >> 4));
      const auto& d = B::dev(row);

      const int32_t v1 = ((((adcT >> 3) - (int32_t(d.T1) << 1))) * int32_t(d.T2)) >> 11;
      const int32_t v2 = (((((adcT >> 4) - int32_t(d.T1)) * ((adcT >> 4) - int32_t(d.T1))) >> 12) * int32_t(d.T3)) >> 14;
      const int32_t tFine = v1 + v2;
      B::template emit<Temp>(row, (tFine * 5 + 128) >> 8);   // 0.01 C

      int64_t p1 = int64_t(tFine) - 128000;
      int64_t p2 = p1 * p1 * int64_t(d.P6);
      p2 = p2 + ((p1 * int64_t(d.P5)) << 17);
      p2 = p2 + (int64_t(d.P4) << 35);
      p1 = ((p1 * p1 * int64_t(d.P3)) >> 8) + ((p1 * int64_t(d.P2)) << 12);
      p1 = (((int64_t(1) << 47) + p1) * int64_t(d.P1)) >> 33;
      if (p1 == 0) return;   // no calibration: a part that did not answer at found()
      int64_t p = 1048576 - adcP;
      p = (((p << 31) - p2) * 3125) / p1;
      p1 = (int64_t(d.P9) * (p >> 13) * (p >> 13)) >> 25;
      p2 = (int64_t(d.P8) * p) >> 19;
      p = ((p + p1 + p2) >> 8) + (int64_t(d.P7) << 4);
      B::template emit<Press>(row, int32_t(p >> 8));   // Pa (Q24.8 >> 8)
    }
  };

  // the BME280 answers 0x60 at the same register: a second entry, the same driver
  template<typename W> using Entries = hapi::Chain<discover::Use<discover::Own, Bmp280<W>>,
                                                   discover::Use<discover::IdProbe<0xD0, 0x60, 0x76, 0x77>, Bmp280<W>>>;

}
