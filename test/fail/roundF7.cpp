// failCompose F7: a health monitor (health.h) over discoverCompose's registry, root bus, two plain devices (no mux --
// R4b's OLED shape is exercised on the board, not duplicated here). One driver type wraps SensorA/SensorB's
// addresses with a device edge and the optional required/mayIsolate/isolate declarations a build flag selects.
//   native: ./roundF7 [-DF7_REQUIRED | -DF7_NO_ISOLATE]      AVR: main() runs a scripted fault and stores a checksum
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include "../support/sensors.h"
#include "../support/faultbus.h"
#include <oneMachine/fail/busedge.h>
#include <oneMachine/fail/devedge.h>
#include <oneMachine/fail/health.h>

using discover::RowId;
using discover::Status;
using hapi::Chain;

#if !defined(__AVR__) && !defined(F7_PARITY)
#include <cstdio>
static int checks = 0, failures = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #cond); } } while (0)
#endif

using Twi = hapi::APIOf<oneBus::TwiAPI, oneBus::I2cMaster<100000>, fbus::CoreT<>, mock::TwiCore>;

struct App;
// A plain device edge over a mock sensor address: no calibration, no canary -- just enough to flap and to cost bus time.
template<typename W, uint8_t AddrLo, uint8_t AddrHi, uint8_t Id, bool Required>
struct HDev : discover::DriverBase<HDev<W, AddrLo, AddrHi, Id, Required>, W>, fail::DevEdge<HDev<W, AddrLo, AddrHi, Id, Required>, W, struct HMode, 1> {
  using Self = HDev;
  using B    = discover::DriverBase<Self, W>;
  using Edge = fail::DevEdge<Self, W, struct HMode, 1>;
  using Produces = Chain<Temperature>;
  static constexpr uint8_t addrLo = AddrLo, addrHi = AddrHi, id = Id;
  static constexpr bool required = Required;
#ifndef F7_NO_ISOLATE
  static constexpr bool mayIsolate = true;
#endif
  inline static uint8_t isolated = 0;
  inline static bool    forceCorrupt = false;      // a device that answers but reports its own reading corrupted: costs bus time, never goes Stale
#ifndef F7_NO_ISOLATE_FN
  static void isolate(RowId) { ++isolated; }          // F7_NO_ISOLATE_FN: mayIsolate declared, nothing to cut
#endif

  static void reinit(RowId) {}
  static void read(RowId row) { Edge::serve(row, fail::Cause::Fresh); }
  static fail::Outcome attempt(RowId row) {
    if (forceCorrupt) return fail::Outcome::Fail(fail::Kind::Corrupt, 0);
    uint8_t b[2] = {0, 0};
    const fail::Outcome o = Edge::checkedRead(row, 1, b, 2);
    if (!o.isOk()) return o;
    B::template emit<Temperature>(row, int16_t(uint16_t((uint16_t(b[0]) << 8) | b[1])));
    return o;
  }
};

using A = HDev<App, 0x40, 0x40, 0xB2, false>;              // the steady device (SensorB's address): must stay unaffected
#ifdef F7_REQUIRED
using B = HDev<App, 0x48, 0x48, 0xA1, true>;                // the flapper, required: never isolated, only escalated
#else
using B = HDev<App, 0x48, 0x48, 0xA1, false>;               // the flapper
#endif

struct HMode {
  using Twi = ::Twi;
  static constexpr bool checked = true, returnPath = false, idempotent = true, lifecycle = true;
  template<typename E> using BusStack = fail::Controller<E, fail::TickPart<fail::Retry<0>>, fail::Recover, fail::DetectError, fail::HoldOp<fail::Coalesce>, fail::Backoff<50, 200>, fail::Status>;
  template<typename E> using DevStack = fail::Controller<E, fail::TickPart<fail::Retry<2>>, fail::Recover, fail::DetectError, fail::HoldOp<fail::Coalesce>, fail::Gate<20>, fail::TickPart<fail::Reprobe<50, 30>>, fail::LazyStatus>;
};

struct Log {
  using Accepts = Chain<Temperature>;
  inline static uint16_t n[4] = {};
  template<typename Cap> struct Body {
    template<typename T> struct Part : T {
      using T::T;
      void on(const discover::Sample<Cap>& s) { if (s.row < 4) ++n[s.row]; }
    };
  };
};

using Drivers = Chain<A, B>;
using BusBase = fail::BusEdge<App, Drivers, 1, HMode>;

struct App : discover::World<App, Twi, Chain<Log>, Chain<A, B>, 4, discover::I2cScan>, BusBase {
  using WorldB = discover::World<App, Twi, Chain<Log>, Chain<A, B>, 4, discover::I2cScan>;
  static constexpr bool lifecycle = true;
  static void release(RowId) {}
  static void unbindAll() {}
  static void busReset() { fbus::State::resets++; }
#ifndef F7_NO_HEALTH
  using Health = fail::HealthT<App, Drivers, 4>;
#endif
#ifdef F7_ESCALATE_HOOK
  inline static uint8_t escalateCalls = 0;
  inline static RowId lastEscalateRow = discover::noRow;
  inline static fail::EscalateReason lastEscalateReason = fail::EscalateReason(0);
  static void onEscalate(RowId row, fail::EscalateReason reason) { ++escalateCalls; lastEscalateRow = row; lastEscalateReason = reason; }
#endif
};

static void tickAll(uint32_t now) {
  App::tickBuses(now);
  for (RowId r = 0; r < App::reg.count; ++r) {
    if (App::reg.rows[r].isBus) continue;
    if (App::reg.rows[r].drv == discover::instOf<A>()) A::tickRow(r, now);
    else if (App::reg.rows[r].drv == discover::instOf<B>()) B::tickRow(r, now);
  }
#ifndef F7_NO_HEALTH
  App::Health::onEdge();
  App::Health::onTick(now);
#endif
}
static uint32_t T = 0;
static void step() { if (T % 10 == 0) App::pump(); tickAll(T); ++T; }
static void run(uint32_t until) { while (T < until) step(); }

static RowId rowOfA() { for (RowId r = 0; r < App::reg.count; ++r) if (!App::reg.rows[r].isBus && App::reg.rows[r].busId == A::addrLo) return r; return discover::noRow; }
static RowId rowOfB() { for (RowId r = 0; r < App::reg.count; ++r) if (!App::reg.rows[r].isBus && App::reg.rows[r].busId == B::addrLo) return r; return discover::noRow; }

static void fresh() {
  fbus::State::reset();
  mock::Bus::devs[0] = {-1, 0x40, false, 0, {0xB2, 0x00, 0, 0}};
  mock::Bus::devs[1] = {-1, 0x48, false, 0, {0xA1, 0x00, 0, 0}};
  mock::Bus::mask = 0; mock::Bus::contention = 0; mock::Bus::txns = 0; mock::Bus::sel = nullptr; mock::Bus::state = mock::Bus::Idle;
  A::isolated = B::isolated = 0;
  Log::n[0] = Log::n[1] = Log::n[2] = Log::n[3] = 0;
  App::discover();
#ifndef F7_NO_HEALTH
  App::Health::reset();
#endif
  T = 0;
}

#if !defined(__AVR__) && !defined(F7_PARITY)
// ---- 1. healthy: no flaps, no cost, no policy fires -------------------------------------------------------------
static void healthy() {
  fresh();
  run(2000);
  const RowId a = rowOfA(), b = rowOfB();
  CHECK(App::Health::status(a).flapCount == 0 && App::Health::status(b).flapCount == 0);
  CHECK(App::Health::status(a).flapEwma == 0 && App::Health::status(b).flapEwma == 0);
  CHECK(App::Health::status(a).costEwma == 0 && App::Health::status(b).costEwma == 0);
  CHECK(!App::Health::status(a).quarantined && !App::Health::status(b).quarantined);
  CHECK(Log::n[a] > 0 && Log::n[b] > 0);
}

#if !defined(F7_REQUIRED) && !defined(F7_NO_ISOLATE)
// ---- 2. a flapping device: the pattern is counted, and Quarantine stops polling it, in growing durations --------
static void flapping() {
  fresh();
  const RowId b = rowOfB();
  for (int i = 0; i < 7 && !App::Health::status(b).quarantined; ++i) {
    mock::Bus::devs[1].addr = 0x4F; run(T + 150);
    mock::Bus::devs[1].addr = 0x48; run(T + 150);
  }
  CHECK(App::Health::status(b).flapCount >= 7 && App::Health::status(b).quarantined);
  const uint16_t before = Log::n[b];
  run(T + 500);                                                                // well inside the quarantine: not polled at all
  CHECK(Log::n[b] == before);
  // let it clear (it is healthy at 0x48 already), then flap it into a second quarantine: the duration must grow
  bool cleared = false;
  for (int i = 0; i < 60 && !cleared; ++i) { run(T + 200); if (!App::Health::status(b).quarantined) cleared = true; }
  CHECK(cleared);
  for (int i = 0; i < 7 && !App::Health::status(b).quarantined; ++i) {
    mock::Bus::devs[1].addr = 0x4F; run(T + 150);
    mock::Bus::devs[1].addr = 0x48; run(T + 150);
  }
  CHECK(App::Health::status(b).quarantined && App::Health::status(b).fib.cur > 2);   // the second episode's step exceeds the first's (2)
}

// ---- 3. a device that drags the bus (Corrupt every poll, never Stale): Disconnect fires on cost alone -----------
static void dragsTheBus() {
  fresh();
  const RowId a = rowOfA(), b = rowOfB();
  B::forceCorrupt = true;
  uint16_t peakCost = 0;
  for (int i = 0; i < 20 && !App::Health::status(b).quarantined; ++i) {
    run(T + 500);
    if (App::Health::status(b).costEwma > peakCost) peakCost = App::Health::status(b).costEwma;
  }
  CHECK(peakCost >= 96 && App::Health::status(b).flapCount == 0);                // cost alone crossed enterD; never went Stale
#ifdef F7_NO_ISOLATE_FN
  CHECK(App::Health::status(b).quarantined && !App::Health::status(b).disconnected && B::isolated == 0);   // quarantined, but nothing was cut: not "disconnected"
#else
  CHECK(App::Health::status(b).quarantined && App::Health::status(b).disconnected && B::isolated >= 1);
#endif
  const uint16_t na = Log::n[a];
  run(T + 1000);
  CHECK(Log::n[a] > na);                                                        // A's own polling is unaffected by B's disconnect
  B::forceCorrupt = false;
}

// ---- 4. a required flapping device: never quarantined or disconnected; the condition escalates instead ----------
#elif defined(F7_REQUIRED)
static void requiredFlaps() {
  fresh();
  const RowId b = rowOfB();
  for (int i = 0; i < 8; ++i) {
    mock::Bus::devs[1].addr = 0x4F; run(T + 150);
    mock::Bus::devs[1].addr = 0x48; run(T + 150);
  }
  CHECK(App::Health::status(b).flapCount >= 7);
  CHECK(!App::Health::status(b).quarantined && B::isolated == 0);               // never isolated: required
  CHECK(App::Health::status(b).escalations > 0);                                // the condition surfaced instead
  const uint16_t before = Log::n[b];
  mock::Bus::devs[1].addr = 0x48; run(T + 300);
  CHECK(Log::n[b] > before);                                                    // polling continued throughout
#ifdef F7_ESCALATE_HOOK
  CHECK(App::escalateCalls > 0 && App::lastEscalateRow == b && App::lastEscalateReason == fail::EscalateReason::Flap);
#endif
}

// ---- 4b. mayIsolate not declared: the same pattern is watched and reported, but Quarantine never runs -------------
#else
static void reportOnly() {
  fresh();
  const RowId b = rowOfB();
  for (int i = 0; i < 8; ++i) {
    mock::Bus::devs[1].addr = 0x4F; run(T + 150);
    mock::Bus::devs[1].addr = 0x48; run(T + 150);
  }
  CHECK(App::Health::status(b).flapCount >= 7);                                 // watched and counted...
  CHECK(!App::Health::status(b).quarantined && B::isolated == 0);               // ...but never acted on: no mayIsolate declared
  const uint16_t before = Log::n[b];
  mock::Bus::devs[1].addr = 0x48; run(T + 300);
  CHECK(Log::n[b] > before);
}
#endif

#if !defined(F7_REQUIRED) && !defined(F7_NO_ISOLATE)
// ---- 5. a single outage is not flapping: one episode, Report only, no quarantine ----------------------------------
static void singleOutage() {
  fresh();
  const RowId b = rowOfB();
  mock::Bus::devs[1].addr = 0x4F; run(T + 300);
  mock::Bus::devs[1].addr = 0x48; run(T + 2500);
  CHECK(App::Health::status(b).flapCount == 1);
  CHECK(!App::Health::status(b).quarantined);
  CHECK(App::Health::status(b).flapEwma < 20);                                  // long decayed back down: one episode is not a pattern
}

// ---- 6. a bus fault is not a device flap: both rows follow the bus down, neither device's own flap count moves ---
static void busFaultNotFlap() {
  fresh();
  const RowId a = rowOfA(), b = rowOfB();
  fbus::State::stuckRoot = true;
  run(T + 300);
  CHECK(App::reg.status(a) == Status::Stale && App::reg.status(b) == Status::Stale);
  CHECK(App::Health::status(a).flapCount == 0 && App::Health::status(b).flapCount == 0);
  CHECK(App::Health::status(0).flapCount >= 1);                                 // the bus row's own flap, not the devices'
  fbus::State::stuckRoot = false;
  run(T + 1000);
  CHECK(App::reg.status(a) == Status::Alive && App::reg.status(b) == Status::Alive);
  CHECK(!App::Health::status(a).quarantined && !App::Health::status(b).quarantined);
}

// ---- 7. hysteresis: quarantine does not exit on a probe that only reaches between exit and enter -------------------
static void hysteresis() {
  fresh();
  const RowId b = rowOfB();
  for (int i = 0; i < 7 && !App::Health::status(b).quarantined; ++i) {
    mock::Bus::devs[1].addr = 0x4F; run(T + 150);
    mock::Bus::devs[1].addr = 0x48; run(T + 150);
  }
  CHECK(App::Health::status(b).quarantined);
  // the device answers again (0x48): probation exits once flapEwma has actually decayed below exitQ. A broken exit that used
  // enterQ instead would let it out the moment flapEwma first dips under enterQ, still well above exitQ -- caught by requiring
  // the value observed at the exit itself to be below exitQ, not merely below enterQ.
  mock::Bus::devs[1].addr = 0x48;
  bool exitedEventually = false;
  uint16_t atExit = 0xFFFF;
  for (int i = 0; i < 60; ++i) {
    run(T + 200);
    if (!App::Health::status(b).quarantined) { exitedEventually = true; atExit = App::Health::status(b).flapEwma; break; }
  }
  CHECK(exitedEventually);                                                       // probation does let a genuinely healthy device rejoin
  CHECK(atExit < 32);                                                            // exitQ, not just under enterQ (96): real hysteresis, not a coin flip

  // deterministic check of the hysteresis band itself: a row parked mid-band (between exitQ=32 and enterQ=96) during probation
  // must NOT be let out. Poking the public HealthRow directly (the same idiom mock state is poked elsewhere).
  fresh();
  for (int i = 0; i < 7 && !App::Health::status(b).quarantined; ++i) {
    mock::Bus::devs[1].addr = 0x4F; run(T + 150);
    mock::Bus::devs[1].addr = 0x48; run(T + 150);
  }
  CHECK(App::Health::status(b).quarantined && !App::Health::status(b).probation);
  const uint32_t hardBlockEnds = App::Health::status(b).until;
  while (T < hardBlockEnds + 20) step();                                             // past the hard block: now in open-ended probation
  CHECK(App::Health::status(b).quarantined && App::Health::status(b).probation);
  const uint32_t until = T + 12000;                                             // several probe windows' worth (probeN=4, tickMs=500)
  while (T < until) { App::Health::rows[b].flapEwma = 60; App::Health::rows[b].costEwma = 0; step(); }
  CHECK(App::Health::status(b).quarantined);                                    // 60 is inside the band: mid-band must stay quarantined
}

// ---- 8. the probe is judged after it ran: a device that is still bad on its probes is not let out by the averages that decayed -----
static void probeIsJudged() {
  fresh();
  const RowId b = rowOfB();
  for (int i = 0; i < 7 && !App::Health::status(b).quarantined; ++i) {
    mock::Bus::devs[1].addr = 0x4F; run(T + 150);
    mock::Bus::devs[1].addr = 0x48; run(T + 150);
  }
  CHECK(App::Health::status(b).quarantined);
  mock::Bus::devs[1].addr = 0x4F;                                               // gone, and it stays gone through the hard block and the probes
  bool leftEarly = false;
  const uint32_t until = T + 30000;
  while (T < until) { step(); if (!App::Health::status(b).quarantined) leftEarly = true; }
  CHECK(!leftEarly);                                                            // the averages decayed long ago; the probes found it still gone
  CHECK(App::Health::status(b).quarantined && App::Health::status(b).fib.cur >= 3);   // and each failed probe made the next block longer
  // it comes back: a probe finds it answering and quiet, and it rejoins
  mock::Bus::devs[1].addr = 0x48;
  bool cleared = false;
  for (int i = 0; i < 200 && !cleared; ++i) { run(T + 500); cleared = !App::Health::status(b).quarantined; }
  CHECK(cleared && App::reg.status(b) == Status::Alive);
}

// ---- 9. a long quarantine does not cool the back-off: the quiet inside the block is not the device being quiet ---------------------
static void quietInsideBlockDoesNotCool() {
  fresh();
  const RowId b = rowOfB();
  for (int i = 0; i < 7 && !App::Health::status(b).quarantined; ++i) {
    mock::Bus::devs[1].addr = 0x4F; run(T + 150);
    mock::Bus::devs[1].addr = 0x48; run(T + 150);
  }
  CHECK(App::Health::status(b).quarantined && App::Health::status(b).quietPeriods == 0);
  App::Health::rows[b].flapEwma = 0;                                            // as if it had decayed: below exitQ for the whole block
  const uint32_t blockEnd = App::Health::status(b).until;
  while (T < blockEnd) step();                                                   // a whole hard block, not polled
  CHECK(App::Health::status(b).quietPeriods == 0);                              // none of it counted as quiet
}
#endif

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  healthy();
#if defined(F7_REQUIRED)
  requiredFlaps();
#elif defined(F7_NO_ISOLATE)
  reportOnly();
#else
  flapping();
  dragsTheBus();
  singleOutage();
  busFaultNotFlap();
  hysteresis();
  probeIsJudged();
  quietInsideBlockDoesNotCool();
#endif
  std::printf("checks %d\n", checks);
  std::printf(failures ? "FAILED (%d)\n" : "OK: failCompose F7 native\n", failures);
  return failures != 0;
}
#else
// ---- the scripted flap, folded into a checksum (mock time, mock bus, discoverCompose R3's convention): AVR stores it in
// g_sum (read back over simavr/avr-gdb); -DF7_PARITY runs the identical sequence natively and prints it, for the parity check.
#ifdef __AVR__
extern "C" void __cxa_pure_virtual() { for (;;) {} }
volatile uint16_t g_sum;
__attribute__((noinline)) void done() { for (;;) {} }
#else
#include <cstdio>
#endif
static uint16_t checksum() {
  const RowId a = rowOfA(), b = rowOfB();
  uint16_t h = 5381;
  h = uint16_t(h * 31u + Log::n[a]); h = uint16_t(h * 31u + Log::n[b]);
#ifndef F7_NO_HEALTH
  h = uint16_t(h * 31u + App::Health::status(a).flapCount); h = uint16_t(h * 31u + App::Health::status(b).flapCount);
  h = uint16_t(h * 31u + App::Health::status(b).quarantined); h = uint16_t(h * 31u + App::Health::status(b).disconnected);
  h = uint16_t(h * 31u + App::Health::status(b).fib.cur);
#endif
  return h;
}
static void faultScript() {
  fresh();
  for (int i = 0; i < 8; ++i) {
    mock::Bus::devs[1].addr = 0x4F; run(T + 150);
    mock::Bus::devs[1].addr = 0x48; run(T + 150);
  }
}
int main() {
  faultScript();
#ifdef __AVR__
  g_sum = checksum();
  done();
#else
  std::printf("checksum 0x%04X\n", checksum());
#endif
}
#endif
