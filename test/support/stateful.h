// Drivers that declare state (Round 3). SensorB, Mux (sensors.h) and LineDisplay (display.h) stay as they are:
// the drivers that declare nothing.
//   CalSensor     DeviceState: the calibration offset read from a config register at found(); shared by every consumer.
//   StateDisplay  DeviceState: the backlight; ClientState: a cursor per consumer.
// R3_NO_DEV / R3_NO_CLIENT drop the declarations (size builds).
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/binding.h>
#include "sensors.h"

#ifdef R3_NO_DEV
  static constexpr bool kDev = false;
#else
  static constexpr bool kDev = true;
#endif
#ifdef R3_NO_CLIENT
  static constexpr bool kClient = false;
#else
  static constexpr bool kClient = true;
#endif

struct Cursor { uint8_t col, row; };

struct CalDev  { struct DeviceState { int16_t offset; }; };
struct DispDev { struct DeviceState { bool backlight; }; };
struct DispCli { using ClientState = Cursor; };
struct NoDev {};
struct NoCal {};
struct NoCli {};

template<typename W>
struct CalSensor : discover::DriverBase<CalSensor<W>, W>, std::conditional_t<kDev, CalDev, NoCal> {
  using B = discover::DriverBase<CalSensor<W>, W>;
  using Produces = hapi::Chain<Temperature>;
  static constexpr uint8_t addrLo = 0x48, addrHi = 0x48, id = 0xA1;
  inline static uint8_t inits = 0;

  static void init(discover::RowId row) {
    if constexpr (kDev) {
      uint8_t c = 0;
      B::readRegs(B::addrOf(row), 3, &c, 1);
      B::dev(row).offset = int8_t(c);
    }
    ++inits;
  }

  static void read(discover::RowId row) {
    uint8_t b[2];
    B::readRegs(B::addrOf(row), 1, b, 2);
    int16_t t = int16_t(uint16_t((uint16_t(b[0]) << 8) | b[1]));
    if constexpr (kDev) t = int16_t(t + B::dev(row).offset);
    B::template emit<Temperature>(row, t);
  }
};

template<typename W>
struct StateDisplay : discover::DriverBase<StateDisplay<W>, W>,
                      std::conditional_t<kDev, DispDev, NoDev>, std::conditional_t<kClient, DispCli, NoCli> {
  using B   = discover::DriverBase<StateDisplay<W>, W>;
  using Ops = hapi::Chain<discover::Print, discover::Clear, discover::SetCursor>;
  static constexpr uint8_t addrLo = 0x27, addrHi = 0x47, id = 0xD4;
  inline static uint8_t inits = 0;

  // found(): the backlight goes on, on the device and in the row
  static void init(discover::RowId row) { light(row, true); ++inits; }

  static void light(discover::RowId row, bool on) {
    using Twi = typename W::Twi;
    begin(row); Twi::write_byte(0x04); Twi::write_byte(on ? 1 : 0); Twi::end_write();
    if constexpr (kDev) B::dev(row).backlight = on;
  }
  static bool lit(discover::RowId row) {
    if constexpr (kDev) return B::dev(row).backlight; else { (void)row; return true; }
  }

  // with a ClientState: the consumer's own cursor, sent before every print
  static void setCursor(discover::RowId, Cursor& c, uint8_t x, uint8_t y) { c.col = x; c.row = y; }
  static void clear(discover::RowId row, Cursor& c) { clear(row); c = Cursor{0, 0}; }
  static void print(discover::RowId row, Cursor& c, const char* s) {
    using Twi = typename W::Twi;
    wake(row);
    begin(row); Twi::write_byte(0x02); Twi::write_byte(c.col); Twi::write_byte(c.row); Twi::end_write();
    uint8_t n = 0;
    begin(row); Twi::write_byte(0x03);
    while (*s) { Twi::write_byte(uint8_t(*s++)); ++n; }
    Twi::end_write();
    c.col = uint8_t(c.col + n);
  }

  // without one: the device's own cursor
  static void clear(discover::RowId row) {
    using Twi = typename W::Twi;
    wake(row);
    begin(row); Twi::write_byte(0x01); Twi::end_write();
  }
  static void print(discover::RowId row, const char* s) {
    using Twi = typename W::Twi;
    wake(row);
    begin(row); Twi::write_byte(0x03);
    while (*s) Twi::write_byte(uint8_t(*s++));
    Twi::end_write();
  }

private:
  // a print wakes a display whose backlight is off
  static void wake(discover::RowId row) {
    if constexpr (kDev) { if (!B::dev(row).backlight) light(row, true); }
    else { (void)row; }
  }
  static void begin(discover::RowId row) {
    W::route(W::reg.rows[row].parent);
    W::Twi::begin_write(B::addrOf(row));
  }
};
