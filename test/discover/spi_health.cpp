// The RC522 row watched by fail::HealthT (an SPI world: fail::DeviceOwnStale gives the monitor its ownStale), against the register model.
//   - single, spaced faults (RST held low 3 s, an RST pulse) are not quarantined
//   - a reader that comes and goes about once a second: quarantined on the flap rate alone
//   - a part that answers its ID and keeps nothing written (VCC pulled, living off its signal pins) flaps and costs: the row is quarantined,
//     is not touched outside a probe window, stays quarantined while its probes find it bad, and the blocks grow
//   - once the part is well again the row stays quiet: quarantine clears and the card is read again
//   - the I2C-free ownStale: a device row's own fault counts, a bus row's never does
// Native only. Time is simulated: the poll every 100 ms, the failure edge and the monitor ticked every 10 ms.
#include <stdint.h>
#include <cstdio>
#include <hapi/hapi.h>
#include <oneMachine/discover/spi.h>
#include <oneMachine/fail/busedge.h>
#include <oneMachine/fail/world.h>
#include <oneMachine/fail/health.h>
#include "../support/mockSpi.h"
#include "../../examples/spi/src/rc522.h"

using discover::RowId;
using discover::Sample;
using discover::Status;
using hapi::Chain;

struct CardLog {
  using Accepts = Chain<rc522::Card>;
  inline static uint32_t last = 0;
  inline static uint8_t n = 0;
  template<typename Cap> struct Body {
    template<typename T> struct Part : T {
      using T::T;
      void on(const Sample<Cap>& s) { last = s.value; ++n; }
    };
  };
};

struct Mode {
  static constexpr bool lifecycle = true, returnPath = false, idempotent = true;
  template<typename E> using DevStack = fail::Controller<E, fail::TickPart<fail::Retry<2>>, fail::Recover, fail::DetectError,
    fail::HoldOp<fail::Coalesce>, fail::Gate<50>, fail::TickPart<fail::Reprobe<500, 120>>, fail::LazyStatus>;
  template<typename Impl, typename W> using Access = fail::SpiAccess<Impl, W>;
};

struct App;
using Rfid = rc522::Rc522<App, Mode, 1>;
using Drivers = discover::DriversIn<Chain<Rfid>>;
struct App : discover::World<App, mspi::Bus, Chain<CardLog>, Chain<Rfid>, 3, discover::SpiScan, Chain<discover::SpiSlotIds<4>>>,
             fail::DeviceOwnStale<App, Drivers> {
  static constexpr bool lifecycle = true;
  static void release(RowId) {}
  static void unbindAll() {}
  using Health = fail::HealthT<App, Drivers, 3, rc522::HealthCfg>;
};
using Ticker = fail::Ticks<App, Drivers>;

static int failures = 0;
#define CHECK(c) do { if (!(c)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

static uint32_t now = 0;
static bool everQuarantined = false;

static void start() {
  mspi::State::reset();
  mspi::State::kind[0] = mspi::Kind::Rc522;
  mspi::State::card = mspi::Card{true, {0xDE, 0xAD, 0xBE, 0xEF}, false, false, false};
  CardLog::n = 0; now = 0; everQuarantined = false;
  Rfid::resetFail();
  App::Health::reset();
  App::discover();
}
static void advance(uint32_t ms) {
  for (const uint32_t end = now + ms; now < end; now += 10) {
    if (now % 100 == 0) App::pump();
    Ticker::run(now);
    App::Health::onEdge();
    App::Health::onTick(now);
    if (App::Health::status(1).quarantined) everQuarantined = true;
  }
}
static const fail::HealthRow& hr() { return App::Health::status(1); }

int main() {
  using mspi::State;

  // ---- single, spaced faults: as on the board (5 x vanish, 5 x pulse, 6 s apart) -------------------------------
  start();
  advance(1000);
  for (uint8_t i = 0; i < 5; ++i) { State::rc.hold(true); advance(3000); State::rc.hold(false); advance(3000); }
  for (uint8_t i = 0; i < 5; ++i) { State::rc.hold(true); State::rc.hold(false); advance(6000); }
  CHECK(!everQuarantined);
  CHECK(App::reg.status(1) == Status::Alive && Rfid::configured(1));
  CHECK(hr().flapCount == 5);                                            // the five vanishes; a pulse never leaves Alive

  // ---- ownStale: a device row's own fault, never a bus row's ----------------------------------------------------
  start();
  CHECK(!App::ownStale(discover::rootRow));
  State::rc.hold(true);
  advance(1000);
  CHECK(App::reg.status(1) == Status::Stale && App::ownStale(1) && !App::ownStale(discover::rootRow));
  State::rc.hold(false);
  advance(2000);
  CHECK(App::reg.status(1) == Status::Alive && !App::ownStale(1));

  // ---- a reader that comes and goes about once a second (RST toggled): only the flap rate sees it ---------------------
  start();
  advance(1000);
  for (uint8_t i = 0; i < 40; ++i) { State::rc.hold(true); advance(600); State::rc.hold(false); advance(500); }
  CHECK(everQuarantined);
  State::rc.hold(false);
  { uint32_t t = now; while (now - t < 120000 && (hr().quarantined || App::reg.status(1) != Status::Alive)) advance(100); }
  CHECK(!hr().quarantined && App::reg.status(1) == Status::Alive);

  // ---- a part that keeps nothing written ----------------------------------------------------------------------------
  start();
  advance(1000);
  CHECK(CardLog::n == 1);
  State::rc.noStore = true;
  State::rc.regs[0x14] = 0; State::rc.regs[0x2B] = 0;
  uint32_t t0 = now;
  while (now - t0 < 30000 && !hr().quarantined) advance(10);
  CHECK(hr().quarantined && now - t0 <= 20000);                          // quarantined within seconds
  
  // the hard block: nothing on the slot
  advance(100);                                                          // let the poll in flight finish
  const uint32_t b0 = State::bytes[0];
  advance(1500);
  CHECK(hr().quarantined && !hr().probation && State::bytes[0] == b0);

  // while the part stays bad, each probe finds it still bad: it stays quarantined and each block is longer than the last (Fibonacci x 2 s);
  // outside a probe window nothing reaches the slot
  uint16_t maxFib = hr().fib.cur;
  bool     leftEarly = false, touched = false;
  uint32_t seen = State::bytes[0], windows = 0;
  bool     wasOpen = false;
  for (uint32_t t = 0; t < 120000; t += 10) {
    advance(10);
    if (!hr().quarantined) leftEarly = true;
    if (hr().probeWindowOpen) { seen = State::bytes[0]; if (!wasOpen) ++windows; }
    else if (wasOpen) seen = State::bytes[0];                            // the step in which the window closes: the row's own tick ran just before it
    else if (State::bytes[0] != seen) touched = true;                    // a poll or a tick reached the slot outside a probe window
    wasOpen = hr().probeWindowOpen;
    if (hr().fib.cur > maxFib) maxFib = hr().fib.cur;
  }
  CHECK(!touched);
  CHECK(!leftEarly);                                                     // the probes found it still bad: no release on averages that decayed in the block
  CHECK(windows >= 3);                                                   // it was probed, repeatedly
  CHECK(maxFib >= 5);                                                    // and each failed probe lengthened the block

  // the part is well again: a probe finds it quiet, quarantine clears, the card is read again
  State::rc.noStore = false;
  t0 = now;
  while (now - t0 < 240000 && (hr().quarantined || App::reg.status(1) != Status::Alive)) advance(100);
  CHECK(!hr().quarantined && App::reg.status(1) == Status::Alive);
  advance(2000);
  CHECK(CardLog::last == 0xDEADBEEFu && Rfid::configured(1));

  std::printf(failures ? "FAILED (%d)\n" : "OK: RC522 health monitor native\n", failures);
  return failures != 0;
}
