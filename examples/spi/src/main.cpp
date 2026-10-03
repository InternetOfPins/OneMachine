// spi -- discovery on two buses of one board: an RC522 RFID reader found on an SPI bus whose chip selects are declared
// statically (one slot each), and a BMP280/BME280 found on I2C. Each bus is its own World: discover once, pump on its
// own period. Wemos D1 mini (ESP8266), Serial 115200. Wiring:
//   RC522    SCK D5, MISO D6, MOSI D7, SDA (its CS) D8, RST D3 or 3V3, 3V3, GND
//   BMP280   SDA D2, SCL D1, 3V3, GND (CSB high or open: I2C mode)
// Slot 1 (D0) is declared with nothing on it: the scan reports it empty.
//
// Line format: <ms> <name>[<row>]=<value>   a card's UID in hex when one arrives, 0 when it leaves.
#include <Arduino.h>
#include <chips/esp8266/esp8266Twi.h>
#include <chips/esp8266/esp8266Spi.h>
#include <chips/esp8266/esp8266Gpio.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include <oneMachine/discover/spi.h>
#include "rc522.h"
#include "bmp280.h"

#ifndef BUILD_REV
  #define BUILD_REV "unknown"   // set by ../version.py: each repo's git commit
#endif

using discover::RowId;
using hapi::Chain;
namespace esp = hw::esp8266;

using Twi = esp::Esp8266TwiMaster<4, 5, 100000>;                                      // SDA D2, SCL D1
using Spi = hapi::APIOf<oneBus::SpiAPI, oneBus::SpiSlots<esp::OutPin<15>, esp::OutPin<16>>,   // slot 0 D8, slot 1 D0
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
using Rfid = rc522::Rc522<RfidApp>;
struct RfidApp : discover::World<RfidApp, Spi, Chain<Printer>, Chain<Rfid>, 3, discover::SpiScan,
                                 Chain<discover::SpiSlotIds<Spi::slots>>> {};
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
  Twi::begin();
  Spi::begin();
  RfidApp::discover();
  AirApp::discover();
  table<RfidApp>(F("SPI slots"));
  table<AirApp>(F("I2C"));
  if (RfidApp::reg.count > 1) {
    Serial.print(F("RC522 version 0x")); Serial.print(Rfid::rd(1, rc522::VersionReg), HEX);
    if (Rfid::configured(1)) Serial.println(F(", configured, antenna on"));
    else {   // what the chip shows instead: PowerDown (0x10) still set in CommandReg means it never woke from the reset
      Serial.print(F(", NOT configured (antenna off): CommandReg 0x")); Serial.print(Rfid::rd(1, rc522::CommandReg), HEX);
      Serial.print(F(" TPrescalerReg 0x")); Serial.println(Rfid::rd(1, rc522::TPrescalerReg), HEX);
    }
  }
}

void loop() {
  static uint32_t nextCard = 0, nextAir = 0;
  const uint32_t now = millis();
  if (int32_t(now - nextCard) >= 0) { nextCard = now + 100;  RfidApp::pump(); }
  if (int32_t(now - nextAir)  >= 0) { nextAir  = now + 1000; AirApp::pump(); }
}
