// mpu6050 -- discover adds real data: the same MPU6050, now woken and configured at found(), polled once a second,
// its seven capabilities (3-axis acceleration, 3-axis rotation, temperature) printed as they arrive. Nano
// (ATmega328P), a GY-521 (MPU6050) on A4 (SDA) / A5 (SCL), Serial 115200.
//
// Line format: <ms> <name>[<row>]=<value>   one line per capability, scaled to the decimals it declares.
#include <Arduino.h>
#include <chips/avr/avrTwi.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include "mpu6050.h"

using discover::RowId;
using hapi::Chain;
using namespace mpu;

using Twi = hw::avr::mega::Twi<100000UL, F_CPU>;
struct App;
using Mpu = Mpu6050<App>;

// Prints every sample as it arrives, scaled to its capability's own decimal places.
struct Printer {
  using Accepts = Chain<AccX, AccY, AccZ, GyrX, GyrY, GyrZ, Temp>;
  template<typename Cap>
  struct Body {
    template<typename T> struct Part : T {
      void on(const discover::Sample<Cap>& s) {
        Serial.print(millis()); Serial.print(' '); Serial.print(Cap::name); Serial.print('[');
        Serial.print(s.row); Serial.print(F("]="));
        const int32_t v = s.value; int32_t p = 1; for (uint8_t i = 0; i < Cap::decimals; ++i) p *= 10;
        Serial.print(v / p); Serial.print('.');
        int32_t frac = v % p; if (frac < 0) frac = -frac;
        Serial.println(frac);
      }
    };
  };
};

using Entries = Chain<discover::Use<discover::Own, Mpu>>;

struct App : discover::World<App, Twi, Chain<Printer>, Entries, 4, discover::I2cScan> {};

void setup() {
  Serial.begin(115200);
  Twi::begin();
  App::discover();
  Serial.println(App::reg.count > 1 ? F("MPU6050 found") : F("MPU6050 not found -- check the wiring"));
}

void loop() {
  static uint32_t next = 0;
  const uint32_t now = millis();
  if (now >= next) { next = now + 1000; App::pump(); }
}
