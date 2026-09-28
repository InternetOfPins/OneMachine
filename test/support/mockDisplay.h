// Text displays on the mock bus, on the root bus. A second TwiCore layered over mock::TwiCore: the displays' addresses
// are intercepted, everything else is delegated, so mockTwi.h is untouched.
// Writes:  first byte = command. 0x00 select the ID register, 0x01 clear, 0x02 x y set cursor, 0x03 c... print chars.
// Reads:   the ID register.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneBus/i2c.h>
#include <oneBus/twiMaster.h>
#include "mockTwi.h"

namespace mockdisp {

  template<uint8_t A, uint8_t I>
  struct ScreenT {
    static constexpr uint8_t addr = A, id = I, cols = 16, rows = 2;

    inline static bool    present = true;
    inline static char    cell[rows][cols] = {};
    inline static uint8_t cx = 0, cy = 0;

    // what reached the display, in order: 'C' | 'S' x y | 'W' c
    inline static uint8_t log[96] = {};
    inline static uint8_t logN = 0, logLost = 0;

    // bus side
    inline static bool    sel = false;
    inline static uint8_t got = 0, cmd = 0, tx = 0;

    static void put(uint8_t b) { if (logN < sizeof log) log[logN++] = b; else ++logLost; }

    static void reset() {
      for (auto& r : cell) for (auto& c : r) c = ' ';
      cx = cy = 0; logN = logLost = 0; sel = false;
    }

    static bool take(uint8_t addrByte) {
      if (!present || (addrByte >> 1) != A) return false;
      sel = true; got = 0; cmd = 0;
      return true;
    }

    static void onWrite(uint8_t b) {
      if (got == 0) {
        cmd = b;
        if (b == 0x01) {
          for (auto& r : cell) for (auto& c : r) c = ' ';
          cx = cy = 0; put('C');
        }
      } else if (cmd == 0x02) {
        if (got == 1) tx = b;
        else if (got == 2) { cx = tx; cy = b; put('S'); put(tx); put(b); }
      } else if (cmd == 0x03) {
        if (cy < rows && cx < cols) cell[cy][cx] = char(b);
        if (cx < 0xFF) ++cx;
        put('W'); put(b);
      }
      if (got < 0xFF) ++got;
    }
  };

  using Screen  = ScreenT<0x27, 0xD4>;
  using Screen2 = ScreenT<0x3F, 0xD5>;

  struct DispCore {
    template<typename O>
    struct Part : O {
      static void twi_start() { Screen::sel = Screen2::sel = false; O::twi_start(); }
      static void twi_stop()  { Screen::sel = Screen2::sel = false; O::twi_stop(); }

      static void twi_write(uint8_t b) {
        if (Screen::sel)  { Screen::onWrite(b);  return; }
        if (Screen2::sel) { Screen2::onWrite(b); return; }
        if (mock::Bus::state == mock::Bus::Addressed && (Screen::take(b) || Screen2::take(b))) return;
        O::twi_write(b);
      }

      [[nodiscard]] static uint8_t twi_read(bool last) {
        if (Screen::sel)  return Screen::id;
        if (Screen2::sel) return Screen2::id;
        return O::twi_read(last);
      }
    };
  };

  using Twi = hapi::APIOf<oneBus::TwiAPI, oneBus::I2cMaster<100000>, DispCore, mock::TwiCore>;
  static_assert(oneBus::is_twi_master<Twi>::value, "mockdisp::Twi must satisfy TwiMaster");

}
