// health -- recover adds a health monitor: a row that flaps too often, or costs too much bus time retrying, is
// quarantined (left alone, tried again on a growing schedule) and then disconnected (isolate() cuts its own
// supply) instead of retrying it forever -- and a canary catches a device that silently loses its own state
// (a brownout on its own supply, still answering but asleep) without ever being reported as a fault. Nano
// (ATmega328P), a GY-521 (MPU6050) on A4 (SDA) / A5 (SCL), its VCC on pin 8 (MPU_VCC_PIN) instead of straight to
// 5V, Serial 115200. See README.md for what's reliably demonstrable by hand on this wiring and what isn't.
//
// Line format: <ms> <name>[<row>]=<value>                        a sample
//              STATUS <ms> row <row> <from>-><to>                 a row's status changed
//              HLTH <ms> row <row> flap=F cost=C quarantined=Q disconnected=D   the monitor's view of the row, on a change
#include <Arduino.h>
#undef bit   // Arduino's bit(b) macro; fail:: has its own bit(Kind)
#include <chips/avr/avrTwi.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include <oneMachine/fail/busedge.h>
#include <oneMachine/fail/world.h>
#include <oneMachine/fail/health.h>
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
  using Health = fail::HealthT<App, Drivers, 4>;
};
using Ticker = fail::Ticks<App, Drivers>;

template<typename Dr> static RowId rowOfDriver() {
  for (RowId r = 0; r < App::reg.count; ++r) if (!App::reg.rows[r].isBus && App::reg.rows[r].drv == discover::instOf<Dr>()) return r;
  return discover::noRow;
}
static uint8_t lastHealthState = 0xFF;
static void logHealthChanges(uint32_t now) {
  const RowId r = rowOfDriver<Mpu>();
  if (r == discover::noRow) return;
  const fail::HealthRow& h = App::Health::status(r);
  const uint8_t state = uint8_t((h.quarantined << 1) | h.disconnected);
  if (state != lastHealthState) {
    Serial.print(F("HLTH ")); Serial.print(now); Serial.print(F(" row ")); Serial.print(r);
    Serial.print(F(" flap=")); Serial.print(h.flapEwma); Serial.print(F(" cost=")); Serial.print(h.costEwma);
    Serial.print(F(" quarantined=")); Serial.print(h.quarantined); Serial.print(F(" disconnected=")); Serial.println(h.disconnected);
    lastHealthState = state;
  }
}

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
  pinMode(MPU_VCC_PIN, OUTPUT); digitalWrite(MPU_VCC_PIN, HIGH); delay(250);   // give the module time to start
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
  App::Health::onEdge();
  App::Health::onTick(now);
  logStatusChanges(now);
  logHealthChanges(now);
}
