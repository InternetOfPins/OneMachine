// An ACK-reporting bus over R1's register-file devices and R2's displays, plus devices that only answer.
// A core whose twi_start/twi_write return bool is what I2cMaster reads as "the address (or byte) was acknowledged";
// Report = false gives the same bus behind a void core (I2cMaster then takes every address as acknowledged).
// Layered over mock::TwiCore and mockdisp::DispCore, both untouched.
//   ghost   an address that answers with a fixed read value and keeps nothing (own = true), or replaces what a device
//           already there reads (own = false: a display that has no ID register, for instance)
//   counts  per address, what reached the core: SLA+W, SLA+R, data bytes written, address bytes not acknowledged
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneBus/i2c.h>
#include <oneBus/twiMaster.h>
#include "mockTwi.h"
#include "mockDisplay.h"

namespace mockack {

#ifdef __AVR__
  inline constexpr bool kCount = false;
#else
  inline constexpr bool kCount = true;
#endif

  struct Ghost { uint8_t addr; uint8_t readVal; bool own; };

  struct State {
    static constexpr uint8_t kGhosts = 8;
    static constexpr uint8_t kAddrs  = kCount ? 128 : 1;

    inline static Ghost    ghosts[kGhosts] = {};
    inline static uint8_t  nGhosts = 0;
    inline static uint8_t  timeouts = 0;                      // the next address bytes fail as a bus timeout (cause Timeout), whoever is addressed
    inline static oneBus::TwiCause cause = oneBus::TwiCause::None;

    inline static uint16_t slaW[kAddrs] = {}, slaR[kAddrs] = {}, dataW[kAddrs] = {}, nack[kAddrs] = {};

    static void add(uint8_t addr, uint8_t readVal, bool own) { if (nGhosts < kGhosts) ghosts[nGhosts++] = Ghost{addr, readVal, own}; }
    static const Ghost* ghost(uint8_t addr) {
      for (uint8_t i = 0; i < nGhosts; ++i) if (ghosts[i].addr == addr) return &ghosts[i];
      return nullptr;
    }
    static bool answers(uint8_t addr) { const Ghost* g = ghost(addr); return g && g->own; }

    static void clearCounts() {
      for (uint8_t i = 0; i < kAddrs; ++i) slaW[i] = slaR[i] = dataW[i] = nack[i] = 0;
    }
    static uint16_t addressBytes(uint8_t a) { return uint16_t(slaW[a] + slaR[a]); }
    static uint16_t writesTo(uint8_t a)     { return uint16_t(slaW[a] + dataW[a]); }

    // R1's topology (mux 0x70, SensorB 0x40, SensorA on both channels) and R2's display at 0x27 (0x3F is not fitted)
    static void reset() {
      using mock::Bus;
      Bus::devs[0] = {-1, 0x70, true,  0, {0xC3, 0x00, 0, 0}};
      Bus::devs[1] = {-1, 0x40, false, 0, {0xB2, 0x00, 0xBB, 64}};
      Bus::devs[2] = { 0, 0x48, false, 0, {0xA1, 0x00, 0xD7, 0}};
      Bus::devs[3] = { 1, 0x48, false, 0, {0xA1, 0x00, 0xFD, 0}};
      Bus::mask = 0; Bus::contention = 0; Bus::txns = 0; Bus::sel = nullptr; Bus::state = Bus::Idle;
      mockdisp::Screen::present = true;  mockdisp::Screen::reset();
      mockdisp::Screen2::present = false; mockdisp::Screen2::reset();
      nGhosts = 0; timeouts = 0; cause = oneBus::TwiCause::None;
      clearCounts();
    }
  };

  template<bool Report>
  struct CoreT {
    template<typename O>
    struct Part : O {
      inline static uint8_t addr = 0;
      inline static bool    addrPhase = false;

      static oneBus::TwiCause twi_cause() { return State::cause; }

      static auto twi_start() {
        State::cause = oneBus::TwiCause::None;
        addrPhase = true;
        O::twi_start();
        if constexpr (Report) return true;
      }
      static void twi_stop() { addrPhase = false; O::twi_stop(); }

      static auto twi_write(uint8_t b) {
        if (addrPhase) {
          addrPhase = false;
          addr = uint8_t(b >> 1);
          O::twi_write(b);
          bool ack = State::answers(addr) || mock::Bus::sel != nullptr || mockdisp::Screen::sel || mockdisp::Screen2::sel;
          if (State::timeouts) { --State::timeouts; ack = false; State::cause = oneBus::TwiCause::Timeout; }
          else if (!ack) State::cause = oneBus::TwiCause::Nack;
          if constexpr (kCount) {
            if (b & 1) ++State::slaR[addr]; else ++State::slaW[addr];
            if (!ack) ++State::nack[addr];
          }
          if constexpr (Report) return ack;
          else return;
        }
        if constexpr (kCount) ++State::dataW[addr];
        if (!State::answers(addr)) O::twi_write(b);      // a ghost takes the byte and keeps nothing
        if constexpr (Report) return true;
      }

      [[nodiscard]] static uint8_t twi_read(bool last) {
        if (const Ghost* g = State::ghost(addr)) return g->readVal;
        return O::twi_read(last);
      }
    };
  };

  using Twi     = hapi::APIOf<oneBus::TwiAPI, oneBus::I2cMaster<100000>, CoreT<true>,  mockdisp::DispCore, mock::TwiCore>;
  using TwiVoid = hapi::APIOf<oneBus::TwiAPI, oneBus::I2cMaster<100000>, CoreT<false>, mockdisp::DispCore, mock::TwiCore>;
  static_assert(oneBus::is_twi_master<Twi>::value && oneBus::is_twi_master<TwiVoid>::value, "mockack buses must satisfy TwiMaster");

}
