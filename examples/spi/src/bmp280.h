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

    // the calibration is read until two reads agree: a corrupted read here would skew every later sample
    static bool readCal(RowId row, uint8_t* c) {
      uint8_t c2[24] = {};
      B::readRegs(B::addrOf(row), 0x88, c, 24);
      B::readRegs(B::addrOf(row), 0x88, c2, 24);
      for (uint8_t i = 0; i < 24; ++i) if (c[i] != c2[i]) return false;
      const uint16_t t1 = uint16_t(c[0] | (c[1] << 8)), p1 = uint16_t(c[6] | (c[7] << 8));
      return t1 != 0 && t1 != 0xFFFF && p1 != 0 && p1 != 0xFFFF;
    }

    static void init(RowId row) {
      writeReg(row, 0xE0, 0xB6);   // soft reset: a warm restart finds the part as the last firmware left it
      for (uint8_t i = 0; i < 200; ++i) {   // status bit 0 (im_update) is set while the NVM is copied in
        uint8_t st = 1;
        B::readRegs(B::addrOf(row), 0xF3, &st, 1);
        if (!(st & 1)) break;
      }
      uint8_t c[24] = {};
      bool ok = false;
      for (uint8_t i = 0; i < 5 && !(ok = readCal(row, c)); ++i) {}
      if (!ok) return;   // calibration stays zero: read() emits nothing
      auto u = [&](uint8_t i) { return uint16_t(c[i] | (uint16_t(c[i + 1]) << 8)); };
      auto s = [&](uint8_t i) { return int16_t(u(i)); };
      auto& d = B::dev(row);
      d.T1 = u(0); d.T2 = s(2); d.T3 = s(4);
      d.P1 = u(6); d.P2 = s(8); d.P3 = s(10); d.P4 = s(12); d.P5 = s(14); d.P6 = s(16); d.P7 = s(18); d.P8 = s(20); d.P9 = s(22);
      writeReg(row, 0xF5, 0x90);   // standby 500 ms, filter x4 (config before ctrl_meas: written in sleep mode)
      writeReg(row, 0xF4, 0x57);   // temperature x2, pressure x16, normal mode
    }

    static constexpr int64_t mul(int64_t v, unsigned n) { return v * (int64_t(1) << n); }

    static void read(RowId row) {
      uint8_t b[6] = {};
      B::readRegs(B::addrOf(row), 0xF7, b, 6);
      const int32_t adcP = int32_t((uint32_t(b[0]) << 12) | (uint32_t(b[1]) << 4) | (b[2] >> 4));
      const int32_t adcT = int32_t((uint32_t(b[3]) << 12) | (uint32_t(b[4]) << 4) | (b[5] >> 4));
      const auto& d = B::dev(row);
      if (d.T1 == 0) return;           // no calibration
      if (adcT == 0x80000) return;     // 0x80000: no conversion yet (reset value)

      const int32_t v1 = ((((adcT >> 3) - (int32_t(d.T1) << 1))) * int32_t(d.T2)) >> 11;
      const int32_t v2 = (((((adcT >> 4) - int32_t(d.T1)) * ((adcT >> 4) - int32_t(d.T1))) >> 12) * int32_t(d.T3)) >> 14;
      const int32_t tFine = v1 + v2;
      B::template emit<Temp>(row, (tFine * 5 + 128) >> 8);   // 0.01 C

      // the datasheet's shifts are multiplications here: left-shifting a negative value is undefined before C++20 (the same results)
      int64_t p1 = int64_t(tFine) - 128000;
      int64_t p2 = p1 * p1 * int64_t(d.P6);
      p2 = p2 + mul(p1 * int64_t(d.P5), 17);
      p2 = p2 + mul(int64_t(d.P4), 35);
      p1 = ((p1 * p1 * int64_t(d.P3)) >> 8) + mul(p1 * int64_t(d.P2), 12);
      p1 = (mul(1, 47) + p1) * int64_t(d.P1) >> 33;
      if (p1 == 0 || adcP == 0x80000) return;   // no calibration, or no pressure conversion yet
      int64_t p = 1048576 - adcP;
      p = ((mul(p, 31) - p2) * 3125) / p1;
      p1 = (int64_t(d.P9) * (p >> 13) * (p >> 13)) >> 25;
      p2 = (int64_t(d.P8) * p) >> 19;
      p = ((p + p1 + p2) >> 8) + mul(int64_t(d.P7), 4);
      B::template emit<Press>(row, int32_t(p >> 8));   // Pa (Q24.8 >> 8)
    }
  };

  // the BME280 answers 0x60 at the same register: a second entry, the same driver
  template<typename W> using Entries = hapi::Chain<discover::Use<discover::Own, Bmp280<W>>,
                                                   discover::Use<discover::IdProbe<0xD0, 0x60, 0x76, 0x77>, Bmp280<W>>>;

}
