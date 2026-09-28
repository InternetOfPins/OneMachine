// recover -- mpu6050 adds a failure edge: a transient I2C fault (a bumped wire, a glitch) is retried, the device is
// re-probed if it stops answering, and re-initialised if it comes back. Nano (ATmega328P), a GY-521 (MPU6050) on
// A4 (SDA) / A5 (SCL), Serial 115200. Unplug SDA or SCL for a few seconds and reconnect it: a STATUS line reports
// the row going Stale and back to Alive, and the samples resume with no further attention.
//
// Line format: <ms> <name>[<row>]=<value>            a sample
//              STATUS <ms> row <row> <from>-><to>     a row's status changed
#include <Arduino.h>
#undef bit   // Arduino's bit(b) macro; fail:: has its own bit(Kind)
#include <chips/avr/avrTwi.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include <oneMachine/fail/busedge.h>
#include <oneMachine/fail/world.h>
#include "mpu6050.h"

using discover::RowId;
using discover::Status;
using hapi::Chain;
using namespace mpu;

using Twi = hw::avr::mega::Twi<100000UL, F_CPU>;
struct App;

struct Mode {
  using Twi = ::Twi;
  static constexpr bool checked = true, returnPath = false, idempotent = true, lifecycle = true;
  template<typename E> using BusStack = fail::Controller<E, fail::TickPart<fail::Retry<0>>, fail::Recover,
    fail::DetectError, fail::HoldOp<fail::Coalesce>, fail::Backoff<100, 400>, fail::Status>;
  template<typename E> using DevStack = fail::Controller<E, fail::TickPart<fail::Retry<2>>, fail::Recover,
    fail::DetectError, fail::HoldOp<fail::Coalesce>, fail::Gate<50>, fail::TickPart<fail::Reprobe<500, 120>>, fail::LazyStatus>;
};
using Mpu = Mpu6050<App, Mode, 1>;

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
using Drivers = discover::DriversIn<Entries>;

struct App : discover::World<App, Twi, Chain<Printer>, Entries, 4, discover::I2cScan>,
             fail::BusEdge<App, Drivers, 1, Mode> {
  static constexpr bool lifecycle = true;
  static void release(RowId) {}
  static void unbindAll() {}
  static void busReset() { Twi::begin(); }
};
using Ticker = fail::Ticks<App, Drivers>;

static Status lastStatus[4] = {Status::Alive, Status::Alive, Status::Alive, Status::Alive};
static void logStatusChanges(uint32_t now) {
  for (RowId r = 0; r < App::reg.count && r < 4; ++r) {
    const Status s = App::reg.status(r);
    if (s != lastStatus[r]) {
      Serial.print(F("STATUS ")); Serial.print(now); Serial.print(F(" row ")); Serial.print(r);
      Serial.print(' '); Serial.print(uint8_t(lastStatus[r])); Serial.print(F("->")); Serial.println(uint8_t(s));
      lastStatus[r] = s;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Twi::begin();
  App::discover();
  Serial.println(App::reg.count > 1 ? F("MPU6050 found") : F("MPU6050 not found -- check the wiring"));
}

void loop() {
  static uint32_t next = 0;
  const uint32_t now = millis();
  if (now >= next) { next = now + 100; App::pump(); }
  App::tickBuses(now);
  Ticker::run(now);
  logStatusChanges(now);
}
