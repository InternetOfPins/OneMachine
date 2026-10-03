// spi -- discovery on two buses of one board: an RC522 RFID reader found on an SPI bus whose chip selects are declared
// statically (one slot each), and a BMP280/BME280 found on I2C. Each bus is its own World: discover once, pump on its
// own period. Wemos D1 mini (ESP8266), Serial 115200. Wiring:
//   RC522    SCK D5, MISO D6, MOSI D7, SDA (its CS) D8, RST D0 (held high here) or 3V3, 3V3, GND
//   BMP280   SDA D2, SCL D1, 3V3, GND (CSB high or open: I2C mode)
// Slot 1 (D4) is declared with nothing on it: the scan reports it empty.
//
// Line format: <ms> <name>[<row>]=<value>   a card's UID in hex when one arrives, 0 when it leaves (after 3 polls
// without it, or when the reader stops answering). miss[1]=<n>: polls in a row that found no card while one is held.
//              rfid[1] stale | gone | alive, init #<n> | reinit, init #<n>    the reader's row, and how often it was initialised
//
// The reader is under failure handling (fail::DevEdge): a reader that stops answering goes Stale, is probed, and is initialised again when
// it answers; one that was reset without the sketch knowing (its configuration gone) is initialised again at once. Faults, from the serial monitor:
//   v   RST low for 3 s: the reader vanishes
//   p   RST low for 1 ms: a silent reset, the reader still answers and has lost its configuration
#include <Arduino.h>
#undef bit   // Arduino's bit(b) macro; fail:: has its own bit(Kind)
#include <chips/esp8266/esp8266Twi.h>
#include <chips/esp8266/esp8266Spi.h>
#include <chips/esp8266/esp8266Gpio.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include <oneMachine/discover/spi.h>
#include <oneMachine/fail/world.h>
#include "rc522.h"
#include "bmp280.h"

#ifndef BUILD_REV
  #define BUILD_REV "unknown"   // set by ../version.py: each repo's git commit
#endif

using discover::RowId;
using hapi::Chain;
namespace esp = hw::esp8266;

using Twi = esp::Esp8266TwiMaster<4, 5, 100000>;                                      // SDA D2, SCL D1
using Spi = hapi::APIOf<oneBus::SpiAPI, oneBus::SpiSlots<esp::OutPin<15>, esp::OutPin<2>>,   // slot 0 D8, slot 1 D4
                        oneBus::SpiMaster<4000000>, esp::Esp8266SpiCore>;

struct Printer {
  using Accepts = Chain<rc522::Card, bmp::Temp, bmp::Press>;
  template<typename Cap>
  struct Body {
    template<typename T> struct Part : T {
      void on(const discover::Sample<Cap>& s) {
        Serial.print(millis()); Serial.print(' '); Serial.print(Cap::name); Serial.print('[');
        Serial.print(s.row); Serial.print(F("]="));
        if constexpr (Cap::decimals == 0) { Serial.println(uint32_t(s.value), HEX); return; }
        const int32_t v = int32_t(s.value); int32_t p = 1; for (uint8_t i = 0; i < Cap::decimals; ++i) p *= 10;
        if (v < 0) Serial.print('-');
        const int32_t a = v < 0 ? -v : v;
        Serial.print(a / p); Serial.print('.');
        const int32_t frac = a % p;
        for (int32_t q = p / 10; q > frac && q > 1; q /= 10) Serial.print('0');
        Serial.println(frac);
      }
    };
  };
};

struct RfidApp;
struct AirApp;

struct RfidMode {
  static constexpr bool lifecycle = true, returnPath = false, idempotent = true;
  template<typename E> using DevStack = fail::Controller<E, fail::TickPart<fail::Retry<2>>, fail::Recover, fail::DetectError,
    fail::HoldOp<fail::Coalesce>, fail::Gate<50>, fail::TickPart<fail::Reprobe<500, 120>>, fail::LazyStatus>;
  template<typename Impl, typename W> using Access = fail::SpiAccess<Impl, W>;
};
using Rfid = rc522::Rc522<RfidApp, RfidMode, 1>;
struct RfidApp : discover::World<RfidApp, Spi, Chain<Printer>, Chain<Rfid>, 3, discover::SpiScan,
                                 Chain<discover::SpiSlotIds<Spi::slots>>> {
  static constexpr bool lifecycle = true;
  static void release(RowId) {}
  static void unbindAll() {}
};
using RfidTicker = fail::Ticks<RfidApp, RfidApp::DriverList>;
constexpr uint8_t rstPin = 16;   // D0: the RC522's RST
struct AirApp  : discover::World<AirApp, Twi, Chain<Printer>, bmp::Entries<AirApp>, 3, discover::I2cScan> {};

template<typename A> static void table(const __FlashStringHelper* bus) {
  Serial.print(bus); Serial.print(F(": ")); Serial.print(A::reg.count - 1); Serial.println(F(" device(s)"));
  for (RowId r = 1; r < A::reg.count; ++r) {
    Serial.print(F("  row ")); Serial.print(r); Serial.print(F(" at 0x")); Serial.println(A::reg.rows[r].busId, HEX);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\nOneMachine SPI + I2C discovery"));
  Serial.println(F("build " BUILD_REV " " __DATE__ " " __TIME__));
  esp::OutPin<rstPin>::begin(); esp::OutPin<rstPin>::on();   // RC522 RST high; a chip select on this pin would reset it
  Twi::begin();
  Spi::begin();
  RfidApp::discover();
  const bool rcAfterSpi = RfidApp::reg.count > 1 && Rfid::configured(1);   // before the I2C scan, for the line below
  AirApp::discover();
  table<RfidApp>(F("SPI slots"));
  table<AirApp>(F("I2C"));
  if (RfidApp::reg.count > 1) {
    Serial.print(F("RC522 version 0x")); Serial.print(Rfid::rd(1, rc522::VersionReg), HEX);
    Serial.println();
    // where the RC522's configuration stands: written at init (on which try), still there after the SPI scan, still
    // there now (after the I2C scan); then one register written and read back, to tell lost writes from a reset since
    const uint16_t tries = RfidApp::devState<Rfid>(1).initTries;
    Serial.print(F("RC522 init: "));
    if (tries) { Serial.print(F("configured on write ")); Serial.print(tries); } else Serial.print(F("never configured"));
    Serial.print(F(", after SPI scan ")); Serial.print(rcAfterSpi ? F("on") : F("off"));
    Serial.print(F(", now ")); Serial.print(Rfid::configured(1) ? F("on") : F("off"));
    Serial.print(F(" (CommandReg 0x")); Serial.print(Rfid::rd(1, rc522::CommandReg), HEX);
    Serial.print(F(" TPrescalerReg 0x")); Serial.print(Rfid::rd(1, rc522::TPrescalerReg), HEX);
    Rfid::wr(1, rc522::TReloadRegL, 0x5A);
    Serial.print(F("), write test: TReloadRegL 0x5A reads 0x")); Serial.println(Rfid::rd(1, rc522::TReloadRegL), HEX);
    Rfid::wr(1, rc522::TReloadRegL, 0xE8);
  }
}

// the reader's row: a status change, and each initialisation
static void logRfid() {
  static discover::Status last = discover::Status::Gone;   // nothing reported yet
  static uint16_t lastInits = 0;
  if (RfidApp::reg.count < 2) return;
  const discover::Status st = RfidApp::reg.status(1);
  const uint16_t inits = RfidApp::devState<Rfid>(1).inits;
  if (st != last) {
    Serial.print(millis()); Serial.print(F(" rfid[1] "));
    if (st == discover::Status::Alive) { Serial.print(F("alive, init #")); Serial.println(inits); }
    else Serial.println(st == discover::Status::Stale ? F("stale") : F("gone"));
    last = st; lastInits = inits;
  } else if (inits != lastInits) {
    lastInits = inits;
    Serial.print(millis()); Serial.print(F(" rfid[1] reinit, init #")); Serial.println(inits);
  }
}

// the fault script: a serial key, no schedule
static void faults(uint32_t now) {
  static uint32_t vanishEnd = 0;
  static bool vanished = false;
  if (Serial.available()) {
    const int c = Serial.read();
    if (c == 'v' && !vanished) { Serial.print(now); Serial.println(F(" fault: RST low 3 s")); esp::OutPin<rstPin>::off(); vanished = true; vanishEnd = now + 3000; }
    else if (c == 'p' && !vanished) {
      Serial.print(now); Serial.println(F(" fault: RST pulse"));
      esp::OutPin<rstPin>::off(); delayMicroseconds(1000); esp::OutPin<rstPin>::on();
    }
  }
  if (vanished && int32_t(now - vanishEnd) >= 0) { esp::OutPin<rstPin>::on(); vanished = false; Serial.print(now); Serial.println(F(" fault: RST high")); }
}

void loop() {
  static uint32_t nextCard = 0, nextAir = 0;
  const uint32_t now = millis();
  faults(now);
  if (int32_t(now - nextCard) >= 0) {
    nextCard = now + 100;  RfidApp::pump();
    static uint8_t lastMiss = 0;   // a card held still that misses polls: the streak, printed when it grows
    const uint8_t miss = RfidApp::reg.count > 1 ? RfidApp::devState<Rfid>(1).missStreak : 0;
    if (miss != lastMiss) { lastMiss = miss; if (miss) { Serial.print(millis()); Serial.print(F(" miss[1]=")); Serial.println(miss); } }
  }
  RfidTicker::run(now);
  logRfid();
  if (int32_t(now - nextAir)  >= 0) { nextAir  = now + 1000; AirApp::pump(); }
}
