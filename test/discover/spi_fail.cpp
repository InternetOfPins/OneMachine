// The RC522 driver under failure handling (fail::DevEdge over fail::SpiAccess), against the register model with RST driven (test/support/mockSpi.h).
//   - a clean run: nothing retried, recovered or initialised again
//   - RST held low (the reader vanishes): Stale, a held card reported gone, probed until it answers, Alive with the configuration written again,
//     and the card read again
//   - a short RST pulse (a silent reset: the ID still answers, the configuration is gone): initialised again at once, the row never leaves Alive,
//     the card is not reported gone
//   - the SPI reprobe: an ID read twice, Ids on both
// -DMACHINE runs them with the driver configured by the RC522 machine (examples/spi/src/rc522_machine.h): the same outcomes.
// Native only. Time is simulated: the poll every 100 ms, the failure edge ticked every 10 ms.
#include <stdint.h>
#include <cstdio>
#include <hapi/hapi.h>
#include <oneMachine/discover/spi.h>
#include <oneMachine/fail/world.h>
#include "../support/mockSpi.h"
#include "../../examples/spi/src/rc522.h"
#ifdef MACHINE
  #include "../../examples/spi/src/rc522_machine.h"   // -DMACHINE: the same scenarios, the driver configured by the RC522 machine
#endif

using discover::RowId;
using discover::Sample;
using discover::Status;
using hapi::Chain;

struct CardLog {
  using Accepts = Chain<rc522::Card>;
  inline static uint32_t log[16];
  inline static uint8_t n = 0;
  template<typename Cap> struct Body {
    template<typename T> struct Part : T {
      using T::T;
      void on(const Sample<Cap>& s) { if (n < 16) log[n++] = s.value; }
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
#ifdef MACHINE
using Rfid = rc522m::Machine<App, rc522m::Slot<0>, Mode>::Driver;
#else
using Rfid = rc522::Rc522<App, Mode, 1>;
#endif
struct App : discover::World<App, mspi::Bus, Chain<CardLog>, Chain<Rfid>, 3, discover::SpiScan, Chain<discover::SpiSlotIds<4>>> {
  static constexpr bool lifecycle = true;
  static void release(RowId) {}
  static void unbindAll() {}
};
using Ticker = fail::Ticks<App, App::DriverList>;

static int failures = 0;
#define CHECK(c) do { if (!(c)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

static uint32_t now = 0;
static bool leftAlive = false;   // did the row leave Alive since the last start()

// the rig: one RC522 on slot 0, a card on it, discovered, and one poll to read the card
static void start() {
  mspi::State::reset();
  mspi::State::kind[0] = mspi::Kind::Rc522;
  mspi::State::card = mspi::Card{true, {0xDE, 0xAD, 0xBE, 0xEF}, false, false, false};
  CardLog::n = 0; leftAlive = false; now = 0;
  Rfid::resetFail();
  App::discover();
}
static void advance(uint32_t ms) {
  for (const uint32_t end = now + ms; now < end; now += 10) {
    if (now % 100 == 0) App::pump();
    Ticker::run(now);
    if (App::reg.status(1) != Status::Alive) leftAlive = true;
  }
}
static bool runUntil(Status s, uint32_t limitMs) {
  for (uint32_t t = 0; t < limitMs; t += 10) { if (App::reg.status(1) == s) return true; advance(10); }
  return App::reg.status(1) == s;
}
static auto& st() { return App::devState<Rfid>(1); }
static bool configured() { return Rfid::configured(1); }

int main() {
  using mspi::State;

  // ---- clean -----------------------------------------------------------------------------------------
  start();
  CHECK(App::reg.count == 2);
  CHECK(st().inits == 1);
  advance(2000);
  CHECK(CardLog::n == 1 && CardLog::log[0] == 0xDEADBEEFu);
  CHECK(st().inits == 1 && !leftAlive);
  const fail::FailStatus fs0 = Rfid::failStatus(1);
  CHECK(fs0.fails == 0 && fs0.recovers == 0 && fs0.retries == 0);

  // ---- the reader vanishes (RST low) --------------------------------------------------------------------
  start();
  advance(500);
  CHECK(CardLog::n == 1 && st().uid == 0xDEADBEEFu);
  State::rc.hold(true);
  CHECK(runUntil(Status::Stale, 2000));
  CHECK(CardLog::n == 2 && CardLog::log[1] == 0);                        // the held card is reported gone
  CHECK(st().uid == 0 && st().inits == 1);
  advance(1500);                                                         // still held: probed, still Stale, not given up (M = 120)
  CHECK(App::reg.status(1) == Status::Stale && CardLog::n == 2);
  const uint32_t released = now;
  State::rc.hold(false);
  CHECK(!configured());                                                  // it came up at its defaults
  CHECK(runUntil(Status::Alive, 2000));
  CHECK(now - released <= 1000);                                         // one probe interval and a little
  CHECK(st().inits == 2 && configured());                                // the configuration was written again
  advance(500);
  CHECK(CardLog::n == 3 && CardLog::log[2] == 0xDEADBEEFu);              // and the card is read again
  CHECK(State::rc.regs[0x14] & 0x03);                                    // antenna on

  // repeated: one initialisation per fault, nothing else
  for (uint8_t i = 0; i < 5; ++i) {
    State::rc.hold(true);   CHECK(runUntil(Status::Stale, 2000));
    advance(300);
    State::rc.hold(false);  CHECK(runUntil(Status::Alive, 2000));
    advance(500);
  }
  CHECK(st().inits == 7 && configured() && App::reg.status(1) == Status::Alive);
  CHECK(CardLog::n == 3 + 10 && CardLog::log[CardLog::n - 1] == 0xDEADBEEFu);   // a gone and a read again per fault

  // ---- a silent reset (RST pulse) ---------------------------------------------------------------------------
  start();
  advance(500);
  CHECK(st().inits == 1 && configured());
  State::rc.hold(true); State::rc.hold(false);
  CHECK(!configured() && State::rc.regs[0x37] == 0x92);                  // the ID still answers, the configuration is gone
  advance(300);
  CHECK(st().inits == 2 && configured());                                // initialised again
  CHECK(!leftAlive);                                                     // the row never left Alive
  CHECK(CardLog::n == 1 && st().uid == 0xDEADBEEFu);                     // the card was not reported gone
  CHECK(Rfid::failStatus(1).recovers == 1);
  for (uint8_t i = 0; i < 5; ++i) { State::rc.hold(true); State::rc.hold(false); advance(300); }
  CHECK(st().inits == 7 && configured() && !leftAlive && CardLog::n == 1);

  // ---- a part that answers its ID and keeps nothing written (VCC pulled, living off the signal pins) ---------------------
  start();
  advance(500);
  State::rc.noStore = true;
  State::rc.regs[0x14] = 0; State::rc.regs[0x2B] = 0;                    // and what it held is gone
  CHECK(runUntil(Status::Stale, 3000));                                  // Corrupt a few times, then Absent: Stale
  CHECK(st().inits <= 1u + Rfid::corruptMax);                            // not an initialisation per poll
  const uint16_t storm = st().inits;
  advance(3000);
  CHECK(App::reg.status(1) == Status::Stale || st().inits <= storm + 8); // probed at the reprobe interval, however it flaps
  State::rc.noStore = false;
  CHECK(runUntil(Status::Alive, 4000));
  advance(1000);
  CHECK(configured() && App::reg.status(1) == Status::Alive);

  // ---- a SoftReset that lands late while running is the same silent reset ----------------------------------------
  start();
  advance(500);
  Rfid::wr(1, rc522::CommandReg, rc522::SoftReset);
  advance(500);
  CHECK(st().inits == 2 && configured() && !leftAlive);

  // ---- the SPI reprobe -------------------------------------------------------------------------------------
  start();
  using Probe = fail::SpiAccess<Rfid, App>;
  CHECK(Probe::reprobe(1).isOk());
  State::rc.hold(true);
  { const fail::Outcome o = Probe::reprobe(1); CHECK(o.failed() && o.kind() == fail::Kind::Absent && o.detail == 0x00); }
  State::rc.hold(false);
  CHECK(Probe::reprobe(1).isOk());
  State::rc.regs[0x37] = 0x90;                                           // somebody else's id
  { const fail::Outcome o = Probe::reprobe(1); CHECK(o.failed() && o.kind() == fail::Kind::Absent && o.detail == 0x90); }

  std::printf(failures ? "FAILED (%d)\n" : "OK: RC522 failure edge native\n", failures);
  return failures != 0;
}
