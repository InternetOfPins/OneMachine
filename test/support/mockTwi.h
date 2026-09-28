// Simulated I2C topology behind a TwiAPI core. Same composition shape as the real chip aliases:
//   hapi::APIOf<oneBus::TwiAPI, oneBus::I2cMaster<F>, mock::TwiCore>
// Register-file devices (reg0 = ID); one of them is a 2-channel bridge whose reg1 selects the
// visible channel. A device is reachable if it is on the root bus or its channel is selected.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneBus/i2c.h>
#include <oneBus/twiMaster.h>

namespace mock {

  struct Dev {
    int8_t  ch;        // -1 = root bus, else bridge channel
    uint8_t addr;
    bool    bridge;
    uint8_t ptr;       // register pointer, kept across STOP/START like a real device
    uint8_t regs[4];
  };

  struct Bus {
    static constexpr uint8_t bridgeAddr = 0x70;
    static constexpr uint8_t idle       = 0xFF;   // what an absent address reads

    inline static Dev devs[] = {
      {-1, 0x70, true,  0, {0xC3, 0x00, 0, 0}},           // bridge, control reg = channel mask
      {-1, 0x40, false, 0, {0xB2, 0x00, 0xBB, 64}},       // SensorB: 18.7 C, 64 %
      { 0, 0x48, false, 0, {0xA1, 0x00, 0xD7, 0}},        // SensorA on ch0: 21.5 C
      { 1, 0x48, false, 0, {0xA1, 0x00, 0xFD, 0}},        // SensorA on ch1: 25.3 C
    };
    static constexpr uint8_t ndevs = sizeof(devs) / sizeof(devs[0]);

    inline static uint8_t  mask       = 0;
    inline static uint16_t contention = 0;    // transactions seen by two devices at once
    inline static uint16_t txns       = 0;

    static bool reachable(const Dev& d) { return d.ch < 0 || (mask >> d.ch) & 1; }

    // bus state machine
    enum State : uint8_t { Idle, Addressed, Data };
    inline static State   state  = Idle;
    inline static Dev*    sel    = nullptr;
    inline static bool    rd     = false;
    inline static bool    gotPtr = false;

    static Dev* find(uint8_t addr) {
      Dev* hit = nullptr;
      for (uint8_t i = 0; i < ndevs; ++i)
        if (devs[i].addr == addr && reachable(devs[i])) {
          if (hit) ++contention;
          else hit = &devs[i];
        }
      return hit;
    }

    static void poke(int8_t ch, uint8_t addr, uint8_t reg, uint8_t v) {
      for (uint8_t i = 0; i < ndevs; ++i)
        if (devs[i].ch == ch && devs[i].addr == addr) devs[i].regs[reg & 3] = v;
    }
  };

  struct TwiCore {
    template<typename O>
    struct Part : O {
      using Base = O;

      static void twi_init(uint32_t) {}
      static void twi_start() { Bus::state = Bus::Addressed; ++Bus::txns; }
      static void twi_stop()  { Bus::state = Bus::Idle; }

      static void twi_write(uint8_t b) {
        if (Bus::state == Bus::Addressed) {
          Bus::sel = Bus::find(b >> 1);
          Bus::rd = b & 1;
          Bus::gotPtr = false;
          Bus::state = Bus::Data;
        } else if (Bus::state == Bus::Data && !Bus::rd && Bus::sel) {
          if (!Bus::gotPtr) { Bus::sel->ptr = b; Bus::gotPtr = true; return; }
          uint8_t reg = Bus::sel->ptr++ & 3;
          if (Bus::sel->bridge && reg == 1) Bus::mask = b & 3;
        }
      }

      [[nodiscard]] static uint8_t twi_read(bool) {
        if (Bus::state != Bus::Data || !Bus::sel) return Bus::idle;
        uint8_t reg = Bus::sel->ptr++ & 3;
        return (Bus::sel->bridge && reg == 1) ? Bus::mask : Bus::sel->regs[reg];
      }

      static void begin() { Base::begin(); }
    };
  };

  using Twi = hapi::APIOf<oneBus::TwiAPI, oneBus::I2cMaster<100000>, TwiCore>;
  static_assert(oneBus::is_twi_master<Twi>::value, "mock::Twi must satisfy TwiMaster");

}
