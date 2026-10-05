// spi -- discovery on two buses of one board: an RC522 RFID reader found on an SPI bus whose chip selects are declared
// statically (one slot each), and a BMP280/BME280 found on I2C. Each bus is its own World: discover once, pump on its
// own period. Wemos D1 mini (ESP8266), Serial 115200. Wiring:
//   RC522    SCK D5, MISO D6, MOSI D7, SDA (its CS) D8, RST D4 (held high here) or 3V3, IRQ D0, 3V3, GND
//   BMP280   SDA D2, SCL D1, 3V3, GND (CSB high or open: I2C mode)
// Slot 1 (D3) is declared with nothing on it: the scan reports it empty.
// D0 (GPIO16) is the RC522's IRQ input. A poll starts its command and returns; the loop finishes it (Rfid::service) when the
// line is asserted, or after 40 ms, so nothing in the loop waits for the reader. The ESP8266 has no interrupt on GPIO16, so the
// line is sampled (irq::Sampled); where the pin has one, irq::IsrFlag sets a flag in the ISR instead. The chip drives the line
// (push-pull, active low); ComIrqReg then tells RxIRq (a card answered) from TimerIRq (none did), and a line that disagrees with
// the register is a device failure. Not on a boot strapping pin (D3, D4, D8): the RC522 keeps its state across a reset of the
// board and a pending request would hold the line low at the next boot.
//
// Line format: <ms> <name>[<row>]=<value>   a card's UID in hex when one arrives, 0 when it leaves (after 3 polls
// without it, or when the reader stops answering). miss[1]=<n>: polls in a row that found no card while one is held.
//              rfid[1] stale | gone | alive, init #<n> | reinit, init #<n>    the reader's row, and how often it was initialised
//              STATUS <ms> row 1 <from>-><to>      its status changed (0 Alive, 1 Stale, 2 Gone)
//              HLTH <ms> row 1 flap=F cost=C quarantined=Q probation=P disconnected=D   the health monitor's view, on a change
//
// The reader is under failure handling (fail::DevEdge): a reader that stops answering goes Stale, is probed, and is initialised again when
// it answers; one that was reset without the sketch knowing (its configuration gone) is initialised again at once. A health monitor watches the
// row: one that keeps flapping is quarantined (not polled) for a growing time. Faults, from the serial monitor:
//   v   RST low for 3 s: the reader vanishes
//   p   RST low for 1 ms: a silent reset, the reader still answers and has lost its configuration
//   IRQ <ms> rx=R tmo=T spurious=S missed=M none=N 0x<ComIrqReg>:<count> ...   every 5 s, the IRQ line against ComIrqReg
//   LOOP <ms> n=<iterations> max=<us> pump=<us> svc=<us>   every 5 s, the loop's iterations and the longest iteration, pump() and service()
#include <Arduino.h>
#undef bit   // Arduino's bit(b) macro; fail:: has its own bit(Kind)
#include <chips/esp8266/esp8266Twi.h>
#include <chips/esp8266/esp8266Spi.h>
#include <chips/esp8266/esp8266Gpio.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include <oneMachine/discover/spi.h>
#include <oneMachine/fail/busedge.h>
#include <oneMachine/fail/world.h>
#include <oneMachine/fail/health.h>
#include "rc522.h"
#include "irq_esp8266.h"
#include "bmp280.h"

#ifndef BUILD_REV
  #define BUILD_REV "unknown"   // set by ../version.py: each repo's git commit
#endif

using discover::RowId;
using hapi::Chain;
namespace esp = hw::esp8266;

using Twi = esp::Esp8266TwiMaster<4, 5, 100000>;                                      // SDA D2, SCL D1
using Spi = hapi::APIOf<oneBus::SpiAPI, oneBus::SpiSlots<esp::OutPin<15>, esp::OutPin<0>>,   // slot 0 D8, slot 1 D3
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

// Rig diagnostics: the IRQ line against ComIrqReg after every command. rx/tmo: the line fell and the register agrees (a card
// answered / the timer ran out); spurious: it fell but the register shows nothing; missed: the register shows a request and the
// line never fell; none: neither.
struct IrqCounters {
  inline static uint32_t rx = 0, tmo = 0, spurious = 0, missed = 0, none = 0;
  inline static uint8_t vals[8] = {}; inline static uint32_t cnt[8] = {};
  static void seen(uint8_t irq, bool line, const fail::Outcome&) {
    const uint8_t hit = irq & 0x21;
    if (line) { if (!hit) ++spurious; else if (irq & 0x20) ++rx; else ++tmo; }
    else if (hit) ++missed; else ++none;
    for (uint8_t i = 0; i < 8; ++i) {
      if (cnt[i] && vals[i] == irq) { ++cnt[i]; return; }
      if (!cnt[i]) { vals[i] = irq; cnt[i] = 1; return; }
    }
  }
  static void report(uint32_t now) {
    Serial.print(F("IRQ ")); Serial.print(now);
    Serial.print(F(" rx=")); Serial.print(rx); Serial.print(F(" tmo=")); Serial.print(tmo);
    Serial.print(F(" spurious=")); Serial.print(spurious); Serial.print(F(" missed=")); Serial.print(missed);
    Serial.print(F(" none=")); Serial.print(none);
    for (uint8_t i = 0; i < 8 && cnt[i]; ++i) { Serial.print(F(" 0x")); Serial.print(vals[i], HEX); Serial.print(':'); Serial.print(cnt[i]); }
    Serial.println();
  }
};

struct RfidApp;
struct AirApp;

struct RfidMode {
  static constexpr bool lifecycle = true, returnPath = false, idempotent = true;
  template<typename E> using DevStack = fail::Controller<E, fail::TickPart<fail::Retry<2>>, fail::Recover, fail::DetectError,
    fail::HoldOp<fail::Coalesce>, fail::Gate<50>, fail::TickPart<fail::Reprobe<500, 120>>, fail::LazyStatus>;
  template<typename Impl, typename W> using Access = fail::SpiAccess<Impl, W>;
  using Irq = rc522::Interrupt<irq::Sampled<16>, IrqCounters>;   // D0
};
using Rfid = rc522::Rc522<RfidApp, RfidMode, 1>;
using RfidDrivers = discover::DriversIn<Chain<Rfid>>;
struct RfidApp : discover::World<RfidApp, Spi, Chain<Printer>, Chain<Rfid>, 3, discover::SpiScan,
                                 Chain<discover::SpiSlotIds<Spi::slots>>>,
                 fail::DeviceOwnStale<RfidApp, RfidDrivers> {
  static constexpr bool lifecycle = true;
  static void release(RowId) {}
  static void unbindAll() {}
  using Health = fail::HealthT<RfidApp, RfidDrivers, 3, rc522::HealthCfg>;
};
using RfidTicker = fail::Ticks<RfidApp, RfidDrivers>;
constexpr uint8_t rstPin = 2;    // D4: the RC522's RST
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
  Rfid::Irq::begin();
  Serial.print(F("reset: ")); Serial.print(ESP.getResetReason()); Serial.print(F(", IRQ line ")); Serial.println(Rfid::Irq::line() ? F("low") : F("high"));
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
    Serial.print(F("RC522 IRQ: ComIEnReg 0x")); Serial.print(Rfid::rd(1, rc522::ComIEnReg), HEX);
    Serial.print(F(" DivIEnReg 0x")); Serial.print(Rfid::rd(1, rc522::DivIEnReg), HEX);
    Serial.print(F(" ComIrqReg 0x")); Serial.print(Rfid::rd(1, rc522::ComIrqReg), HEX);
    Serial.print(F(", IRQ line ")); Serial.println(Rfid::Irq::line() ? F("low") : F("high"));
  }
}

// the health monitor's view of the reader's row, when its quarantine state changes
static void logHealth() {
  static uint8_t last = 0xFF;
  if (RfidApp::reg.count < 2) return;
  const fail::HealthRow& h = RfidApp::Health::status(1);
  const uint8_t state = uint8_t((h.quarantined << 2) | (h.probation << 1) | h.disconnected);
  if (state == last) return;
  last = state;
  Serial.print(F("HLTH ")); Serial.print(millis()); Serial.print(F(" row 1 flap=")); Serial.print(h.flapEwma);
  Serial.print(F(" cost=")); Serial.print(h.costEwma); Serial.print(F(" quarantined=")); Serial.print(h.quarantined);
  Serial.print(F(" probation=")); Serial.print(h.probation); Serial.print(F(" disconnected=")); Serial.println(h.disconnected);
}

// the reader's row: a status change, and each initialisation
static void logRfid() {
  static discover::Status last = discover::Status::Gone;   // nothing reported yet
  static uint16_t lastInits = 0;
  if (RfidApp::reg.count < 2) return;
  const discover::Status st = RfidApp::reg.status(1);
  const uint16_t inits = RfidApp::devState<Rfid>(1).inits;
  if (st != last) {
    if (last != discover::Status::Gone) { Serial.print(F("STATUS ")); Serial.print(millis()); Serial.print(F(" row 1 ")); Serial.print(uint8_t(last)); Serial.print(F("->")); Serial.println(uint8_t(st)); }
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

// loop timing, rig diagnostics: iterations since the last report and the longest iteration, pump() and service() (microseconds)
struct LoopStats {
  inline static uint32_t n = 0, last = 0, maxLoop = 0, maxPump = 0, maxSvc = 0;
  static void lap(uint32_t us) { if (last && us - last > maxLoop) maxLoop = us - last; last = us; ++n; }
  static void note(uint32_t& m, uint32_t from) { const uint32_t d = micros() - from; if (d > m) m = d; }
  static void report(uint32_t now) {
    Serial.print(F("LOOP ")); Serial.print(now); Serial.print(F(" n=")); Serial.print(n); Serial.print(F(" max=")); Serial.print(maxLoop);
    Serial.print(F(" pump=")); Serial.print(maxPump); Serial.print(F(" svc=")); Serial.println(maxSvc);
    n = 0; maxLoop = maxPump = maxSvc = 0;
  }
};

void loop() {
  static uint32_t nextCard = 0, nextAir = 0;
  LoopStats::lap(micros());
  const uint32_t now = millis();
  faults(now);
  if (RfidApp::reg.count > 1) { const uint32_t t = micros(); Rfid::service(1, now); LoopStats::note(LoopStats::maxSvc, t); }
  if (int32_t(now - nextCard) >= 0) {
    nextCard = now + 100;
    { const uint32_t t = micros(); RfidApp::pump(); LoopStats::note(LoopStats::maxPump, t); }
    static uint8_t lastMiss = 0;   // a card held still that misses polls: the streak, printed when it grows
    const uint8_t miss = RfidApp::reg.count > 1 ? RfidApp::devState<Rfid>(1).missStreak : 0;
    if (miss != lastMiss) { lastMiss = miss; if (miss) { Serial.print(millis()); Serial.print(F(" miss[1]=")); Serial.println(miss); } }
  }
  RfidTicker::run(now);
  RfidApp::Health::onEdge();
  RfidApp::Health::onTick(now);
  logRfid();
  logHealth();
  if (int32_t(now - nextAir)  >= 0) { nextAir  = now + 1000; AirApp::pump(); }
  static uint32_t nextIrq = 5000;
  if (int32_t(now - nextIrq) >= 0) { nextIrq = now + 5000; IrqCounters::report(now); LoopStats::report(now); }
}
