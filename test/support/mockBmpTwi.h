// A simulated BMP280 on an I2C bus (0x76), behind a TwiAPI core like test/support/mockTwi.h: a 256-byte register file with the register pointer
// kept across STOP/START, the chip id at 0xD0, a soft reset at 0xE0 (0xB6) that puts ctrl_meas and config back to their reset values and keeps
// the calibration, and the calibration and raw samples of the datasheet's worked example (section 8.2): adc_T 519888, adc_P 415148 compensate
// to 25.08 C and 100653.27 Pa.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneBus/i2c.h>
#include <oneBus/twiMaster.h>

namespace mockbmp {

  struct State {
    inline static uint8_t  regs[256];
    inline static uint8_t  ptr = 0;
    inline static bool     present = true;
    inline static uint8_t  addr = 0x76;
    inline static uint8_t  idByte = 0x58;
    inline static uint8_t  log[16]; inline static uint8_t nlog = 0;     // the registers written, in order (not the pointer)
    inline static uint16_t resets = 0;

    enum Bus : uint8_t { Idle, Addressed, Data };
    inline static Bus  bus = Idle;
    inline static bool sel = false, rd = false, gotPtr = false;

    inline static uint32_t sampleP = 0, sampleT = 0;   // what a conversion gives: loaded into the data registers while the mode is not sleep
    static bool running() { return (regs[0xF4] & 3) != 0; }
    static void load() {
      const uint32_t adcP = sampleP, adcT = sampleT;
      regs[0xF7] = uint8_t(adcP >> 12); regs[0xF8] = uint8_t(adcP >> 4); regs[0xF9] = uint8_t((adcP & 15) << 4);
      regs[0xFA] = uint8_t(adcT >> 12); regs[0xFB] = uint8_t(adcT >> 4); regs[0xFC] = uint8_t((adcT & 15) << 4);
    }
    static void setSample(uint32_t adcP, uint32_t adcT) { sampleP = adcP; sampleT = adcT; if (running()) load(); }
    static void softReset() {
      ++resets;
      regs[0xF4] = 0x00; regs[0xF5] = 0x00; regs[0xF3] = 0x00;
      for (uint8_t r = 0xF7; r <= 0xFC; ++r) regs[r] = 0;
      regs[0xF7] = 0x80; regs[0xFA] = 0x80;                          // no conversion yet: 0x80000
    }
    static void reset() {
      for (auto& r : regs) r = 0;
      regs[0xD0] = idByte;
      // 0x88..0x9F: T1 T2 T3 P1..P9, little endian
      static constexpr uint8_t cal[24] = {0x70, 0x6B, 0x43, 0x67, 0x18, 0xFC, 0x7D, 0x8E, 0xBB, 0xD6, 0xD0, 0x0B, 0x27, 0x0B, 0x8C, 0x00, 0xF9, 0xFF, 0x8C, 0x3C, 0xF8, 0xC6, 0x70, 0x17};
      for (uint8_t i = 0; i < 24; ++i) regs[0x88 + i] = cal[i];
      softReset(); resets = 0;
      ptr = 0; present = true; nlog = 0; bus = Idle; sel = rd = gotPtr = false;
      setSample(415148, 519888);
    }
  };

  struct TwiCore {
    template<typename O>
    struct Part : O {
      using Base = O;
      static void twi_init(uint32_t) {}
      static void twi_start() { State::bus = State::Addressed; }
      static void twi_stop()  { State::bus = State::Idle; }
      static void twi_write(uint8_t b) {
        if (State::bus == State::Addressed) {
          State::sel = State::present && (b >> 1) == State::addr; State::rd = b & 1; State::gotPtr = false; State::bus = State::Data;
        } else if (State::bus == State::Data && !State::rd && State::sel) {
          if (!State::gotPtr) { State::ptr = b; State::gotPtr = true; return; }
          const uint8_t reg = State::ptr++;
          if (reg == 0xE0) { if (b == 0xB6) State::softReset(); return; }
          State::regs[reg] = b;
          if (reg == 0xF4 && State::running()) State::load();          // a conversion starts: its result is in the data registers
          if (State::nlog < sizeof(State::log)) State::log[State::nlog++] = reg;
        }
      }
      [[nodiscard]] static uint8_t twi_read(bool) {
        if (State::bus != State::Data || !State::sel) return 0xFF;
        return State::regs[State::ptr++];
      }
      static void begin() { Base::begin(); }
    };
  };

  using Twi = hapi::APIOf<oneBus::TwiAPI, oneBus::I2cMaster<100000>, TwiCore>;
  static_assert(oneBus::is_twi_master<Twi>::value, "mockbmp::Twi must satisfy TwiMaster");

}
