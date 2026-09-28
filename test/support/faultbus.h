// A reporting TWI core for failCompose F5, layered over discoverCompose's mock::TwiCore (mockTwi.h is not touched):
//   - an address nothing answers is NACKed (cause Nack, or Unknown when `causeless`: a core that cannot tell why);
//   - a stuck bus: the whole bus (root), or one mux channel behind an isolating mux; every transaction that needs it burns
//     `stuckCostMs` of mock time (A1: a clamped bus cost about 287 ms per operation on the F030) and reports Timeout;
//   - a read leg that fails without saying why (`stickOnRead` with `causeless`), while the write probe that follows still reports;
//   - counters of what reached the bus: transactions, address-only probes, and the time the stuck ones cost.
// The core keeps the last operation's cause, as the OneBus cores do. CoreT<Extra> lets a device layered under it (a display) answer.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneBus/i2c.h>
#include <oneBus/twiMaster.h>
#include "mockTwi.h"

namespace fbus {

  // What a device layered under this core adds: devices that answer without going through mock::Bus (`selected()` right
  // after the address byte) and that sit on the root bus (`onRoot(addr)`).
  struct NoExtra {
    static bool selected()          { return false; }
    static bool onRoot(uint8_t)     { return false; }
  };

  struct State {
    inline static bool     stuckRoot   = false;
    inline static bool     stuckCh[2]  = {false, false};
    inline static bool     causeless   = false;
    inline static bool     stickOnRead = false;   // the bus sticks at the next read's address byte: that read fails (Unknown when causeless), every start after it times out
    inline static uint32_t stuckCostMs = 287;

    inline static uint32_t starts = 0, txns = 0, probeTxns = 0, timeouts = 0, busyMs = 0, resets = 0;   // starts: every attempt to take the bus
    inline static bool     wroteData = false, isWrite = false, open = false, addrPhase = false;

    static void clearCounters() { starts = txns = probeTxns = timeouts = busyMs = resets = 0; }
    static void reset() {
      stuckRoot = false; stuckCh[0] = stuckCh[1] = false; causeless = false; stickOnRead = false; clearCounters();
      wroteData = isWrite = open = addrPhase = false;
    }
  };

  template<typename Extra = NoExtra>
  struct CoreT {
    template<typename O>
    struct Part : O {
      inline static oneBus::TwiCause _cause = oneBus::TwiCause::None;

      static bool timeout() {
        ++State::timeouts; State::busyMs += State::stuckCostMs; _cause = oneBus::TwiCause::Timeout; return false;
      }
      // a device behind the selected stuck channel, or an address probed while it is selected and nothing on the root answers
      static bool downstreamStuck(uint8_t addr) {
        for (uint8_t k = 0; k < 2; ++k) {
          if (!State::stuckCh[k] || !((mock::Bus::mask >> k) & 1)) continue;
          bool onRoot = Extra::onRoot(addr);
          for (uint8_t i = 0; i < mock::Bus::ndevs; ++i)
            if (mock::Bus::devs[i].addr == addr && mock::Bus::devs[i].ch < 0) onRoot = true;
          if (!onRoot) return true;
        }
        return false;
      }

      static void twi_init(uint32_t f) { O::twi_init(f); }

      static bool twi_start() {
        _cause = oneBus::TwiCause::None;
        ++State::starts;
        if (State::stuckRoot) return timeout();
        ++State::txns; State::open = true; State::addrPhase = true; State::wroteData = false; State::isWrite = false;
        O::twi_start();
        return true;
      }
      static void twi_stop() {
        if (State::open && State::isWrite && !State::wroteData) ++State::probeTxns;   // SLA+W and STOP: a presence probe
        State::open = false; O::twi_stop();
      }
      static bool twi_write(uint8_t b) {
        if (State::addrPhase) {                                                        // the address byte
          State::addrPhase = false;
          const uint8_t addr = uint8_t(b >> 1);
          State::isWrite = !(b & 1);
          if (State::stickOnRead && (b & 1)) {                                       // a read leg that finds the bus stuck: no cause, like Wire's requestFrom
            State::stickOnRead = false; State::stuckRoot = true;
            timeout(); _cause = State::causeless ? oneBus::TwiCause::Unknown : oneBus::TwiCause::Timeout; return false;
          }
          if (downstreamStuck(addr)) return timeout();
          O::twi_write(b);
          if (mock::Bus::sel == nullptr && !Extra::selected()) { _cause = State::causeless ? oneBus::TwiCause::Unknown : oneBus::TwiCause::Nack; return false; }
          return true;
        }
        State::wroteData = true;
        O::twi_write(b);
        return true;
      }
      [[nodiscard]] static uint8_t twi_read(bool last) { return O::twi_read(last); }
      static oneBus::TwiCause twi_cause() { return _cause; }
      static void begin() { O::begin(); }
    };
  };

  using Core = CoreT<>;
  using Twi  = hapi::APIOf<oneBus::TwiAPI, oneBus::I2cMaster<100000>, Core, mock::TwiCore>;
  static_assert(oneBus::is_twi_master<Twi>::value, "fbus::Twi must satisfy TwiMaster");

}
