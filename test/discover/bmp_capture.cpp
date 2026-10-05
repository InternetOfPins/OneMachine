// Capture and restore on the BMP280 machine (one Reconcile part per register: what is wanted of it, compared with what it reads back), under a failure edge (fail::DevEdge over the I2C bus edge), against a simulated chip (mockBmpTwi.h).
//   - found: the defaults are the init and are captured; the slot (the row, the machine's statics) is kept while the part is gone
//   - a set is captured; the part is reset without the host knowing (soft reset): the canary sees a register that no longer holds the capture,
//     Recover calls reinit(), the part is validated (same chip id and calibration) and the capture is replayed
//   - the part is unplugged: Stale, probed; a set while it is gone is captured too; plugged in again (reset values), the last intent comes back
//   - another part (its own calibration) or another chip id: not validated, the slot is dropped and the defaults are the init
// Native only. Time is simulated: the poll every 100 ms, the edges ticked every 10 ms.
#include <stdint.h>
#include <cstdio>
#include <hapi/hapi.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include <oneMachine/fail/busedge.h>
#include <oneMachine/fail/world.h>
#include "../support/mockBmpTwi.h"
#include "../../examples/spi/src/bmp280_machine.h"

using discover::RowId;
using discover::Status;
using hapi::Chain;

struct Mode {
  static constexpr bool checked = true, returnPath = false, idempotent = true, lifecycle = true;
  template<typename E> using BusStack = fail::Controller<E, fail::TickPart<fail::Retry<0>>, fail::Recover, fail::DetectError,
    fail::HoldOp<fail::Coalesce>, fail::Backoff<100, 400>, fail::Status>;
  template<typename E> using DevStack = fail::Controller<E, fail::TickPart<fail::Retry<2>>, fail::Recover, fail::DetectError,
    fail::HoldOp<fail::Coalesce>, fail::Gate<50>, fail::TickPart<fail::Reprobe<500, 120>>, fail::LazyStatus>;
};

struct App;
using M = bmpm::Machine<App, bmpm::Addr<0x76>, Mode>;
using Drivers = Chain<M::Driver>;
struct App : discover::World<App, mockbmp::Twi, Chain<>, M::Entries, 3, discover::I2cScan>, fail::BusEdge<App, Drivers, 1, Mode> {
  static constexpr bool lifecycle = true;
  static void release(RowId) {}
  static void unbindAll() {}
  static void busReset() { mockbmp::Twi::begin(); }
};
using Ticker = fail::Ticks<App, Drivers>;

static int failures = 0;
#define CHECK(...) do { if (!(__VA_ARGS__)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #__VA_ARGS__); } } while (0)

static uint32_t now = 0;
static bool leftAlive = false;
static void advance(uint32_t ms) {
  for (const uint32_t end = now + ms; now < end; now += 10) {
    if (now % 100 == 0) App::pump();
    App::tickBuses(now);
    Ticker::run(now);
    if (App::reg.status(1) != Status::Alive) leftAlive = true;
  }
}
static bool runUntil(Status s, uint32_t limitMs) {
  for (uint32_t t = 0; t < limitMs; t += 10) { if (App::reg.status(1) == s) return true; advance(10); }
  return App::reg.status(1) == s;
}
static void start() {
  mockbmp::State::reset();
  M::Dev::known = false; M::Dev::restored = M::Dev::defaulted = 0; M::Dev::temp = M::Dev::press = 0;
  M::Driver::resetFail();
  now = 0; leftAlive = false;
  App::discover();
}
static uint8_t cap(unsigned i) { uint8_t v = 0; M::visitReg(uint8_t(i), [&](auto& r) { v = r.desired(); }); return v; }   // what the register wants (Reconcile)

int main() {
  using mockbmp::State;
  auto& c = State::c76;

  // ---- found: the defaults are the init, and are captured ----------------------------------------------------------------------
  start();
  CHECK(App::reg.count == 2 && App::reg.status(1) == Status::Alive);
  CHECK(M::Dev::defaulted == 1 && M::Dev::restored == 0);
  CHECK(c.regs[0xF5] == 0x90 && c.regs[0xF4] == 0x57);
  CHECK(cap(0) == 0x90 && cap(1) == 0x57);
  CHECK(M::Dev::known);
  advance(500);
  CHECK(M::Dev::temp == 2508 && M::Dev::press == 100653);
  CHECK(M::Driver::failStatus(1).fails == 0 && M::Driver::failStatus(1).recovers == 0 && !leftAlive);

  // ---- a set is captured, and written to the real register ---------------------------------------------------------------------
  M::resolve<3, 1>().set(0x27);
  CHECK(c.regs[0xF4] == 0x27 && cap(1) == 0x27);
  advance(500);
  CHECK(M::Driver::failStatus(1).fails == 0 && c.regs[0xF4] == 0x27);                // the canary reads back what was set: nothing to report

  // ---- the part is reset without the host knowing: the capture comes back ------------------------------------------------------
  c.softReset();                                                                      // ctrl_meas and config at their reset values, calibration kept
  CHECK(c.regs[0xF4] == 0x00 && c.regs[0xF5] == 0x00);
  advance(300);
  CHECK(c.regs[0xF4] == 0x27 && c.regs[0xF5] == 0x90);                                // replayed, not the defaults
  CHECK(M::Dev::restored == 1 && M::Dev::defaulted == 1);
  CHECK(M::Driver::failStatus(1).recovers == 1 && uint8_t(M::Driver::failStatus(1).lastKind) == uint8_t(fail::Kind::Corrupt));
  CHECK(App::reg.status(1) == Status::Alive && !leftAlive && App::reg.count == 2);   // the row never left Alive
  advance(500);
  CHECK(M::Dev::restored == 1);                                                       // and it does not repeat: the canary is satisfied

  // ---- unplugged: Stale and probed; a set while it is gone is captured; plugged in again, the last intent comes back ----------
  c.unplug();
  CHECK(runUntil(Status::Stale, 3000));
  CHECK(App::reg.count == 2);                                                         // the slot is kept on loss
  M::resolve<3, 1>().set(0x2B);                                                       // the write is not acknowledged; the intent is captured
  CHECK(cap(1) == 0x2B && c.regs[0xF4] == 0x27);
  advance(1000);
  CHECK(App::reg.status(1) == Status::Stale);                                         // still gone: probed, not given up
  c.replug();                                                                         // powered up again: reset values
  CHECK(c.regs[0xF4] == 0x00);
  CHECK(runUntil(Status::Alive, 3000));
  advance(200);
  CHECK(c.regs[0xF4] == 0x2B && c.regs[0xF5] == 0x90);                                // the last intent, not the value before the loss
  CHECK(M::Dev::restored == 2 && M::Dev::defaulted == 1);
  advance(500);
  CHECK(M::Dev::temp == 2508);                                                        // and it measures again

  // ---- another part in its place (its own calibration): not validated, the slot is dropped, the defaults are the init -------------
  const uint32_t before = M::Dev::ident;
  c.unplug();
  CHECK(runUntil(Status::Stale, 3000));
  c.regs[0x8B] = 0x70;                                                                // a different part: T2 differs
  c.replug();
  CHECK(runUntil(Status::Alive, 3000));
  advance(200);
  CHECK(c.regs[0xF4] == 0x57 && c.regs[0xF5] == 0x90);                                // the defaults, not 0x2B
  CHECK(cap(1) == 0x57);                                                              // the old intent is gone; the defaults are wanted
  CHECK(M::Dev::defaulted == 2 && M::Dev::restored == 2 && M::Dev::ident != before);
  CHECK(M::Dev::cal.T2 == 0x7043);                                                    // its calibration, read again

  // ---- another chip id (a BME280 in place of the BMP280): validation fails, the defaults are the init -----------------------
  M::resolve<3, 1>().set(0x27);
  c.setId(0x60);
  CHECK(M::bring() == M::How::Defaults);
  CHECK(c.regs[0xF4] == 0x57 && M::Dev::defaulted == 3);
  c.setId(0x58);
  CHECK(M::bring() == M::How::Defaults);                                              // the hash is of the part now known (the BME280's): a third change
  CHECK(M::bring() == M::How::Restored);                                              // the same part: validated, replayed

  if (failures) { std::printf("FAILED: %d\n", failures); return 1; }
  std::printf("OK: BMP280 capture and restore native\n");
  return 0;
}
