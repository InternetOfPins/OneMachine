// A failure edge's reprobe accepts every chip id the driver's entries declare, not only Impl::id. A driver that serves a family (a BMP280 at 0x58
// and a BME280 at 0x60 are one driver, two identify entries) has a row for each; a BME280 row that goes Stale must reprobe back to Alive.
//   - a row found by the 0x60 entry: unplugged, Stale; plugged in again, Alive
//   - a row found by the driver's own id (0x58): the same
//   - another id at that address (0x61, declared by no entry): the reprobe does not accept it, the row stays Stale
// Native only. Time is simulated: the poll every 100 ms, the edges ticked every 10 ms.
#include <stdint.h>
#include <cstdio>
#include <hapi/hapi.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include <oneMachine/fail/devedge.h>
#include <oneMachine/fail/busedge.h>
#include <oneMachine/fail/world.h>
#include "../support/mockBmpTwi.h"

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

template<typename W>
struct Sensor : discover::DriverBase<Sensor<W>, W>, fail::DevEdge<Sensor<W>, W, Mode, 1> {
  using B = discover::DriverBase<Sensor, W>;
  using Edge = fail::DevEdge<Sensor, W, Mode, 1>;
  static constexpr bool polled = true;
  static constexpr uint8_t addrLo = 0x76, addrHi = 0x76, idReg = 0xD0, id = 0x58;
  inline static int inits = 0, reinits = 0;
  static void init(RowId) { ++inits; }
  static void reinit(RowId) { ++reinits; }
  static void read(RowId row) { Edge::serve(row, fail::Cause::Fresh); }
  static fail::Outcome attempt(RowId row) { uint8_t v = 0; return Edge::checkedRead(row, 0xD0, &v, 1); }
};

struct App;
using S = Sensor<App>;
// the one driver, two entries: its own id (0x58) and a BME280's (0x60)
using Entries = Chain<discover::Use<discover::Own, S>, discover::Use<discover::IdProbe<0xD0, 0x60, 0x76, 0x76>, S>>;
struct App : discover::World<App, mockbmp::Twi, Chain<>, Entries, 3, discover::I2cScan>, fail::BusEdge<App, Chain<S>, 1, Mode> {
  static constexpr bool lifecycle = true;
  static void release(RowId) {}
  static void unbindAll() {}
  static void busReset() { mockbmp::Twi::begin(); }
};
using Ticker = fail::Ticks<App, Chain<S>>;

static int failures = 0;
#define CHECK(...) do { if (!(__VA_ARGS__)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #__VA_ARGS__); } } while (0)

static uint32_t now = 0;
static void advance(uint32_t ms) {
  for (const uint32_t end = now + ms; now < end; now += 10) {
    if (now % 100 == 0) App::pump();
    App::tickBuses(now);
    Ticker::run(now);
  }
}
static bool runUntil(Status s, uint32_t limitMs) {
  for (uint32_t t = 0; t < limitMs; t += 10) { if (App::reg.status(1) == s) return true; advance(10); }
  return App::reg.status(1) == s;
}
static void start(uint8_t id) {
  mockbmp::State::reset();
  mockbmp::State::c76.setId(id);
  S::resetFail(); S::inits = S::reinits = 0; now = 0;
  App::discover();
}

int main() {
  using mockbmp::State;
  CHECK(discover::DeclaredIds<App, S>::has(0x58) && discover::DeclaredIds<App, S>::has(0x60) && !discover::DeclaredIds<App, S>::has(0x61));

  // ---- a BME280 row (found by the 0x60 entry) revives ---------------------------------------------------------------------------
  start(0x60);
  CHECK(App::reg.count == 2 && App::reg.status(1) == Status::Alive);
  State::c76.unplug();
  CHECK(runUntil(Status::Stale, 3000));
  advance(1500);                                                       // probed, still gone
  CHECK(App::reg.status(1) == Status::Stale);
  State::c76.replug();
  CHECK(runUntil(Status::Alive, 3000));                                // the reprobe accepts 0x60
  CHECK(S::reinits >= 1);

  // ---- a row of the driver's own id revives, as before -----------------------------------------------------------------------------
  start(0x58);
  State::c76.unplug();
  CHECK(runUntil(Status::Stale, 3000));
  State::c76.replug();
  CHECK(runUntil(Status::Alive, 3000));

  // ---- an id no entry declares is not accepted: the row stays Stale ---------------------------------------------------------------
  start(0x60);
  State::c76.unplug();
  CHECK(runUntil(Status::Stale, 3000));
  State::c76.setId(0x61);
  State::c76.replug();
  advance(3000);
  CHECK(App::reg.status(1) == Status::Stale);

  if (failures) { std::printf("FAILED: %d\n", failures); return 1; }
  std::printf("OK: reprobe accepts the declared ids native\n");
  return 0;
}
