// Text display driver over the mock protocol (mockDisplay.h): the interface a direct connection talks to, and the
// provider of the Print/Clear/SetCursor operations for output classes. Operations route to the display's own bus first.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/discover/binding.h>

template<typename W>
struct TextDisplay : discover::DriverBase<TextDisplay<W>, W> {
  using B   = discover::DriverBase<TextDisplay<W>, W>;
  using Ops = hapi::Chain<discover::Print, discover::Clear, discover::SetCursor>;
  static constexpr uint8_t addrLo = 0x27, addrHi = 0x27, id = 0xD4;

  static void clear(discover::RowId row) {
    using Twi = typename W::Twi;
    begin(row); Twi::write_byte(0x01); Twi::end_write();
  }
  static void setCursor(discover::RowId row, uint8_t x, uint8_t y) {
    using Twi = typename W::Twi;
    begin(row); Twi::write_byte(0x02); Twi::write_byte(x); Twi::write_byte(y); Twi::end_write();
  }
  static void print(discover::RowId row, const char* s) {
    using Twi = typename W::Twi;
    begin(row); Twi::write_byte(0x03);
    while (*s) Twi::write_byte(uint8_t(*s++));
    Twi::end_write();
  }

private:
  static void begin(discover::RowId row) {
    W::route(W::reg.rows[row].parent);
    W::Twi::begin_write(B::addrOf(row));
  }
};

// A second, different driver type that also provides the TextOut class (print + clear) but not SetCursor:
// a class consumer can use either; a direct connection that needs the cursor can only bind TextDisplay.
template<typename W>
struct LineDisplay : discover::DriverBase<LineDisplay<W>, W> {
  using B   = discover::DriverBase<LineDisplay<W>, W>;
  using Ops = hapi::Chain<discover::Print, discover::Clear>;
  static constexpr uint8_t addrLo = 0x3F, addrHi = 0x3F, id = 0xD5;

  static void clear(discover::RowId row) {
    using Twi = typename W::Twi;
    begin(row); Twi::write_byte(0x01); Twi::end_write();
  }
  static void print(discover::RowId row, const char* s) {
    using Twi = typename W::Twi;
    begin(row); Twi::write_byte(0x03);
    while (*s) Twi::write_byte(uint8_t(*s++));
    Twi::end_write();
  }

private:
  static void begin(discover::RowId row) {
    W::route(W::reg.rows[row].parent);
    W::Twi::begin_write(B::addrOf(row));
  }
};
