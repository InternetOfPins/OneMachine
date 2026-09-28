// The mock bus for Round 3: a display with a backlight whose address can move and which can be removed, layered over
// mock::TwiCore (R1) next to R2's 0x3F line display, reused as it is. mockTwi.h and mockDisplay.h are untouched.
// Display writes: first byte = command. 0x00 select the ID register, 0x01 clear, 0x02 x y cursor, 0x03 c... print,
// 0x04 b backlight. Reads: the ID register. `hits70` counts address bytes sent to the bridge, removed or not.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneBus/i2c.h>
#include <oneBus/twiMaster.h>
#include "mockTwi.h"
#include "mockDisplay.h"

namespace mock3 {

  struct Display {
    static constexpr uint8_t id = 0xD4, cols = 16, rows = 2;

    inline static uint8_t addr      = 0x27;
    inline static bool    present   = true;
    inline static bool    backlight = false;
    inline static char    cell[rows][cols] = {};
    inline static uint8_t cx = 0, cy = 0;

    // what reached the display, in order: 'C' | 'S' x y | 'W' c | 'B' b
    inline static uint8_t log[128] = {};
    inline static uint8_t logN = 0, logLost = 0;

    inline static bool    sel = false;
    inline static uint8_t got = 0, cmd = 0, tx = 0;

    inline static uint8_t hits70 = 0;

    static void put(uint8_t b) { if (logN < sizeof log) log[logN++] = b; else ++logLost; }
    static void blank() { for (auto& r : cell) for (auto& c : r) c = ' '; cx = cy = 0; }

    static void clearScreen() { blank(); logN = logLost = 0; sel = false; }
    static void powerCycle()  { blank(); backlight = false; }        // a device that comes back has lost its state
    static void reset()       { clearScreen(); backlight = false; }

    static bool take(uint8_t addrByte) {
      if (!present || (addrByte >> 1) != addr) return false;
      sel = true; got = 0; cmd = 0;
      return true;
    }

    static void onWrite(uint8_t b) {
      if (got == 0) {
        cmd = b;
        if (b == 0x01) { blank(); put('C'); }
      } else if (cmd == 0x02) {
        if (got == 1) tx = b;
        else if (got == 2) { cx = tx; cy = b; put('S'); put(tx); put(b); }
      } else if (cmd == 0x03) {
        if (cy < rows && cx < cols) cell[cy][cx] = char(b);
        if (cx < 0xFF) ++cx;
        put('W'); put(b);
      } else if (cmd == 0x04) {
        if (got == 1) { backlight = b != 0; put('B'); put(b); }
      }
      if (got < 0xFF) ++got;
    }
  };

  struct Core {
    template<typename O>
    struct Part : O {
      using Line = mockdisp::Screen2;

      static void twi_start() { Display::sel = Line::sel = false; O::twi_start(); }
      static void twi_stop()  { Display::sel = Line::sel = false; O::twi_stop(); }

      static void twi_write(uint8_t b) {
        if (Display::sel) { Display::onWrite(b); return; }
        if (Line::sel)    { Line::onWrite(b);    return; }
        if (mock::Bus::state == mock::Bus::Addressed) {
          if ((b >> 1) == 0x70 && Display::hits70 < 0xFF) ++Display::hits70;
          if (Display::take(b) || Line::take(b)) return;
        }
        O::twi_write(b);
      }

      [[nodiscard]] static uint8_t twi_read(bool last) {
        if (Display::sel) return Display::id;
        if (Line::sel)    return Line::id;
        return O::twi_read(last);
      }
    };
  };

  using Twi = hapi::APIOf<oneBus::TwiAPI, oneBus::I2cMaster<100000>, Core, mock::TwiCore>;
  static_assert(oneBus::is_twi_master<Twi>::value, "mock3::Twi must satisfy TwiMaster");

}
