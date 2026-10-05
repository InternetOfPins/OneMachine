// Simulated BMP280s on an I2C bus (0x76 and 0x77), behind a TwiAPI core that reports acknowledgement and the cause of a failure, as a real
// bus does (test/support/mockAck.h): an address nobody answers is not acknowledged (cause Nack).
// Each chip is a 256-byte register file with the register pointer kept across STOP/START, the chip id at 0xD0, a soft reset at 0xE0 (0xB6)
// that puts ctrl_meas and config back to their reset values and keeps the calibration, and a conversion that starts when ctrl_meas leaves
// sleep. Chip 0x76 holds the datasheet's worked example (section 8.2): calibration, adc_T 519888 and adc_P 415148 compensate to 25.08 C and
// 100653.27 Pa. Chip 0x77 has a calibration of its own, so a part that replaces another is told by it.
//   present    false: the chip is unplugged (its address is not acknowledged); true again: it is back, as after power-up (reset values)
//   idByte     0x58 BMP280, 0x60 BME280
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneBus/i2c.h>
#include <oneBus/twiMaster.h>

namespace mockbmp {

  struct Chip {
    uint8_t  addr = 0;
    uint8_t  regs[256] = {};
    uint8_t  ptr = 0;
    bool     present = true;
    uint8_t  idByte = 0x58;
    uint8_t  log[32] = {}; uint8_t nlog = 0;      // the registers written, in order (not the pointer)
    uint16_t resets = 0;
    uint32_t sampleP = 0, sampleT = 0;            // what a conversion gives: loaded into the data registers while the mode is not sleep

    bool running() const { return (regs[0xF4] & 3) != 0; }
    void load() {
      regs[0xF7] = uint8_t(sampleP >> 12); regs[0xF8] = uint8_t(sampleP >> 4); regs[0xF9] = uint8_t((sampleP & 15) << 4);
      regs[0xFA] = uint8_t(sampleT >> 12); regs[0xFB] = uint8_t(sampleT >> 4); regs[0xFC] = uint8_t((sampleT & 15) << 4);
    }
    void setSample(uint32_t adcP, uint32_t adcT) { sampleP = adcP; sampleT = adcT; if (running()) load(); }
    void softReset() {
      ++resets;
      regs[0xF4] = 0x00; regs[0xF5] = 0x00; regs[0xF3] = 0x00;
      for (uint8_t r = 0xF7; r <= 0xFC; ++r) regs[r] = 0;
      regs[0xF7] = 0x80; regs[0xFA] = 0x80;       // no conversion yet: 0x80000
    }
    void unplug() { present = false; }
    void replug() { present = true; softReset(); }   // powered up again: reset values, calibration kept
    void reset(uint8_t a, bool second) {
      for (auto& r : regs) r = 0;
      addr = a; idByte = 0x58; present = true; nlog = 0; resets = 0; ptr = 0;
      regs[0xD0] = idByte;
      // 0x88..0x9F: T1 T2 T3 P1..P9, little endian
      static constexpr uint8_t cal[24] = {0x70, 0x6B, 0x43, 0x67, 0x18, 0xFC, 0x7D, 0x8E, 0xBB, 0xD6, 0xD0, 0x0B, 0x27, 0x0B, 0x8C, 0x00, 0xF9, 0xFF, 0x8C, 0x3C, 0xF8, 0xC6, 0x70, 0x17};
      for (uint8_t i = 0; i < 24; ++i) regs[0x88 + i] = cal[i];
      if (second) regs[0x8B] = 0x70;               // T2 differs (0x7043, not 0x6743): another part
      softReset(); resets = 0;
      setSample(415148, 519888);
    }
    void setId(uint8_t id) { idByte = id; regs[0xD0] = id; }
  };

  struct State {
    inline static Chip c76, c77;
    enum Bus : uint8_t { Idle, Addressed, Data };
    inline static Bus   bus = Idle;
    inline static Chip* sel = nullptr;
    inline static bool  rd = false, gotPtr = false;
    inline static oneBus::TwiCause cause = oneBus::TwiCause::None;
    inline static uint32_t writes = 0;            // data bytes written to any chip

    static Chip* find(uint8_t a) { Chip* c = a == 0x76 ? &c76 : a == 0x77 ? &c77 : nullptr; return c && c->present ? c : nullptr; }
    static void reset() {
      c76.reset(0x76, false); c77.reset(0x77, true);
      bus = Idle; sel = nullptr; rd = gotPtr = false; cause = oneBus::TwiCause::None; writes = 0;
    }
  };

  struct TwiCore {
    template<typename O>
    struct Part : O {
      using Base = O;
      static oneBus::TwiCause twi_cause() { return State::cause; }
      static void twi_init(uint32_t) {}
      static bool twi_start() { State::cause = oneBus::TwiCause::None; State::bus = State::Addressed; return true; }
      static void twi_stop()  { State::bus = State::Idle; }
      static bool twi_write(uint8_t b) {
        if (State::bus == State::Addressed) {
          State::sel = State::find(b >> 1); State::rd = b & 1; State::gotPtr = false; State::bus = State::Data;
          if (!State::sel) { State::cause = oneBus::TwiCause::Nack; return false; }
          return true;
        }
        if (State::bus == State::Data && !State::rd && State::sel) {
          Chip& c = *State::sel;
          if (!State::gotPtr) { c.ptr = b; State::gotPtr = true; return true; }
          ++State::writes;
          const uint8_t reg = c.ptr++;
          if (reg == 0xE0) { if (b == 0xB6) c.softReset(); return true; }
          c.regs[reg] = b;
          if (reg == 0xF4 && c.running()) c.load();          // a conversion starts: its result is in the data registers
          if (c.nlog < sizeof(c.log)) c.log[c.nlog++] = reg;
          return true;
        }
        return State::sel != nullptr;
      }
      [[nodiscard]] static uint8_t twi_read(bool) {
        if (State::bus != State::Data || !State::sel) return 0xFF;
        return State::sel->regs[State::sel->ptr++];
      }
      static void begin() { Base::begin(); }
    };
  };

  using Twi = hapi::APIOf<oneBus::TwiAPI, oneBus::I2cMaster<100000>, TwiCore>;
  static_assert(oneBus::is_twi_master<Twi>::value, "mockbmp::Twi must satisfy TwiMaster");

}
