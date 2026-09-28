// Mock sensors and the capability tags they produce, shared by the rounds. Drivers are DriverBase<itself,W>.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/discover/registry.h>

using discover::RowId;
using discover::DriverBase;
using hapi::Chain;

// ---- capabilities ----------------------------------------------------------------------------
struct Temperature { using Value = int16_t; static constexpr uint8_t id = 1; static constexpr uint8_t decimals = 1; static constexpr const char* name = "temperature"; };   // 0.1 C
struct Humidity    { using Value = uint8_t; static constexpr uint8_t id = 2; static constexpr uint8_t decimals = 0; static constexpr const char* name = "humidity"; };      // %

// ---- drivers ---------------------------------------------------------------------------------
template<typename W>
struct SensorA : DriverBase<SensorA<W>, W> {
  using B = DriverBase<SensorA<W>, W>;
  using Produces = Chain<Temperature>;
  static constexpr uint8_t addrLo = 0x48, addrHi = 0x48, id = 0xA1;

  static void read(RowId row) {
    uint8_t b[2];
    B::readRegs(B::addrOf(row), 1, b, 2);
    B::template emit<Temperature>(row, int16_t(uint16_t((uint16_t(b[0]) << 8) | b[1])));
#ifdef NEG_UNDECLARED_CAP
    B::template emit<Humidity>(row, 0);
#endif
  }
};

template<typename W>
struct SensorB : DriverBase<SensorB<W>, W> {
  using B = DriverBase<SensorB<W>, W>;
  using Produces = Chain<Temperature, Humidity>;
  static constexpr uint8_t addrLo = 0x40, addrHi = 0x40, id = 0xB2;

  static void read(RowId row) {
    uint8_t b[3];
    B::readRegs(B::addrOf(row), 1, b, 3);
    B::template emit<Temperature>(row, int16_t(uint16_t((uint16_t(b[0]) << 8) | b[1])));
    B::template emit<Humidity>(row, b[2]);
  }
};

template<typename W>
struct Mux : DriverBase<Mux<W>, W> {
  static constexpr bool    isBridge = true;
  static constexpr uint8_t channels = 2;
  static constexpr uint8_t addrLo = 0x70, addrHi = 0x77, id = 0xC3;

  static void select(uint8_t addr, uint8_t ch) { ctrl(addr, uint8_t(1u << ch)); }
  static void clear(uint8_t addr)               { ctrl(addr, 0); }

private:
  static void ctrl(uint8_t addr, uint8_t mask) {
    using Twi = typename W::Twi;
    Twi::begin_write(addr); Twi::write_byte(1); Twi::write_byte(mask); Twi::end_write();
  }
};
