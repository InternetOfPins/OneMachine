// The RC522 with its interrupt part (rc522::Interrupt) against the register model with a modelled IRQ line (test/support/mockSpi.h):
//   - a poll starts its command and returns: nothing waits for the chip, and nothing is delivered until service() finishes it
//   - service() finishes a command when the line is asserted, or after the timeout; one step per command (WUPA, a second WUPA, anticollision)
//   - what the bits mean: RxIRq a card answered, TimerIRq alone nobody did
//   - the line is released after every command (ComIEnReg back to IRqInv only, the request cleared), and push-pull is set at init
//   - the line against the register is a device failure through the failure edge: a line that never fell (Fault, detail 2), one that fell with no
//     request (Fault, detail 1), a command that never ended (Timeout)
// Native only. Time is simulated.
#include <stdint.h>
#include <cstdio>
#include <hapi/hapi.h>
#include <oneMachine/discover/spi.h>
#include <oneMachine/fail/world.h>
#include "../support/mockSpi.h"
#include "../../examples/spi/src/rc522.h"

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

struct MockLine {
  static void begin() {}
  static void arm() {}
  static bool ready() { return mspi::State::rc.line(); }
};
struct Seen {
  inline static uint32_t n = 0; inline static uint8_t lastIrq = 0; inline static bool lastLine = false; inline static fail::Outcome last;
  static void seen(uint8_t irq, bool line, const fail::Outcome& o) { ++n; lastIrq = irq; lastLine = line; last = o; }
};

struct Mode {
  static constexpr bool lifecycle = true, returnPath = false, idempotent = true;
  template<typename E> using DevStack = fail::Controller<E, fail::TickPart<fail::Retry<2>>, fail::Recover, fail::DetectError,
    fail::HoldOp<fail::Coalesce>, fail::Gate<50>, fail::TickPart<fail::Reprobe<500, 120>>, fail::LazyStatus>;
  template<typename Impl, typename W> using Access = fail::SpiAccess<Impl, W>;
  using Irq = rc522::Interrupt<MockLine, Seen>;
};

struct App;
using Rfid = rc522::Rc522<App, Mode, 1>;
struct App : discover::World<App, mspi::Bus, Chain<CardLog>, Chain<Rfid>, 3, discover::SpiScan, Chain<discover::SpiSlotIds<4>>> {
  static constexpr bool lifecycle = true;
  static void release(RowId) {}
  static void unbindAll() {}
};
using Ticker = fail::Ticks<App, App::DriverList>;

static int failures = 0;
#define CHECK(c) do { if (!(c)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

static uint32_t now = 0;
static auto& st() { return App::devState<Rfid>(1); }
static bool inFlight() { return st().phase != Rfid::Rest; }

static void start(bool card) {
  mspi::State::reset();
  mspi::State::kind[0] = mspi::Kind::Rc522;
  mspi::State::card = mspi::Card{card, {0xDE, 0xAD, 0xBE, 0xEF}, false, false, false};
  CardLog::n = 0; Seen::n = 0; now = 0;
  Rfid::resetFail();
  App::discover();
}
// service the poll until it is at rest; the number of steps it took (a step finishes one command)
static int run(uint32_t limitMs = 200) {
  int steps = 0;
  for (uint32_t t = 0; inFlight() && t < limitMs; ++t, ++now) {
    const uint8_t before = st().phase;
    const uint32_t seen = Seen::n;
    Rfid::service(1, now);
    if (Seen::n != seen || st().phase != before) ++steps;
  }
  return steps;
}
static uint32_t bytes() { return mspi::State::bytes[0]; }

int main() {
  using mspi::State;
  auto& rc = State::rc;

  // ---- init: push-pull set, nothing enabled between commands ---------------------------------------------
  start(true);
  CHECK(App::reg.count == 2);
  CHECK(rc.regs[0x03] == rc522::Interrupt<MockLine>::pushPull);
  CHECK(rc.regs[0x02] == rc522::Interrupt<MockLine>::idle && !rc.line());

  // ---- a card: the poll returns at once, service() finishes it in two steps (WUPA, anticollision) ------------------
  const uint32_t b0 = bytes();
  App::pump();
  CHECK(inFlight());                                  // started, not waited for
  CHECK(CardLog::n == 0);
  CHECK(bytes() - b0 < 80);                           // a canary and a command: no polling loop
  CHECK(rc.regs[0x02] == rc522::Interrupt<MockLine>::enable);   // enabled while the command runs
  CHECK(rc.line());                                   // the mock ends it at StartSend
  CHECK(run() == 2);
  CHECK(!inFlight());
  CHECK(CardLog::n == 1 && CardLog::log[0] == 0xDEADBEEFu);
  CHECK(Seen::n == 2 && Seen::lastLine && (Seen::lastIrq & 0x20) && Seen::last.isOk());
  CHECK(rc.regs[0x02] == rc522::Interrupt<MockLine>::idle && rc.regs[0x04] == 0 && !rc.line());   // released and cleared
  CHECK(Rfid::failStatus(1).fails == 0);

  // ---- no card: two silent wake-ups (the second is the retry), the timer ended both ---------------------------------------
  start(false);
  App::pump();
  CHECK(inFlight());
  CHECK(run() == 2);
  CHECK(CardLog::n == 0 && !inFlight());
  CHECK(Seen::n == 2 && Seen::lastLine && !(Seen::lastIrq & 0x20) && (Seen::lastIrq & 0x01) && Seen::last.isOk());
  CHECK(Rfid::failStatus(1).fails == 0 && App::reg.status(1) == Status::Alive);
  CHECK(rc.regs[0x02] == rc522::Interrupt<MockLine>::idle && !rc.line());

  // ---- the card leaves: reported gone after 3 polls, the same as without the interrupt part ---------------------
  start(true);
  App::pump(); run();
  CHECK(CardLog::n == 1);
  State::card.present = false;
  for (int i = 0; i < 3; ++i) { App::pump(); run(); }
  CHECK(CardLog::n == 2 && CardLog::log[1] == 0 && st().uid == 0);

  // ---- a poll still in flight is not started again -------------------------------------------------------------
  start(true);
  App::pump();
  const uint32_t b1 = bytes();
  App::pump();                                        // the canary only
  CHECK(bytes() - b1 < 20 && inFlight());
  run();
  CHECK(CardLog::n == 1);

  // ---- a command that never ends: nothing before the timeout, then a Timeout through the edge ------------------
  start(true);
  rc.stall = true;
  App::pump();
  const uint32_t t0 = now;
  Rfid::service(1, t0);                               // arms the deadline
  Rfid::service(1, t0 + rc522::Interrupt<MockLine>::timeoutMs - 1);
  CHECK(inFlight() && Seen::n == 0);
  Rfid::service(1, t0 + rc522::Interrupt<MockLine>::timeoutMs);
  CHECK(!inFlight() && Seen::n == 1 && Seen::last.failed() && Seen::last.kind() == fail::Kind::Timeout);
  CHECK(Rfid::failStatus(1).fails >= 1 && uint8_t(Rfid::failStatus(1).lastKind) == uint8_t(fail::Kind::Timeout));
  CHECK(st().inits == 2);                             // Recover initialised the chip again
  CHECK(CardLog::n == 0);

  // ---- the line never fell though the register shows the answer (not connected): Fault, detail 2 ----------------
  start(true);
  rc.lineFault = 1;
  App::pump();
  CHECK(inFlight() && !rc.line());
  Rfid::service(1, now);                              // the line is not asserted: waits for the timeout
  CHECK(inFlight());
  Rfid::service(1, now + rc522::Interrupt<MockLine>::timeoutMs);
  CHECK(!inFlight() && Seen::last.failed() && Seen::last.kind() == fail::Kind::Fault && Seen::last.detail == rc522::LineCheck::missed);
  CHECK(Rfid::failStatus(1).fails >= 1 && uint8_t(Rfid::failStatus(1).lastKind) == uint8_t(fail::Kind::Fault));
  CHECK(CardLog::n == 0);                             // the poll ended: no card is claimed on a line that disagrees

  // ---- the line fell and the register shows no request (stuck low): Fault, detail 1 ------------------------
  start(true);
  rc.stall = true; rc.lineFault = 2;
  App::pump();
  CHECK(inFlight());
  Rfid::service(1, now);                              // asserted: finished at once, no timeout wait
  CHECK(!inFlight() && Seen::last.failed() && Seen::last.kind() == fail::Kind::Fault && Seen::last.detail == rc522::LineCheck::spurious);

  if (failures) { std::printf("FAILED: %d\n", failures); return 1; }
  std::printf("OK: RC522 interrupt part native\n");
  return 0;
}
