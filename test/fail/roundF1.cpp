// failCompose F1, integration: one device edge (SensorA) with a failure controller, inside discoverCompose's world.
//   FAIL_STEP 0 bare, real bus (NoFaults)          -> must be discoverCompose R1's image
//             1 bare, scripted faults              -> the scaffold alone
//             2 + Detect(error) + Status
//             3 + Recover
//             4 + Retry (Gate, Hold)
//             5 + Detect(deadline)                 -> full
//   native: ./roundF1 test | scenario     AVR: main() runs the scenario and stores a checksum
#include <stdint.h>
#include <hapi/hapi.h>
#define DISCOVER_TEST_RAW_STATUS
#include <oneMachine/fail/world.h>
#include <oneMachine/discover/identify.h>
#include "../support/mockTwi.h"

#ifndef FAIL_STEP
#define FAIL_STEP 5
#endif

using discover::RowId;
using discover::DriverBase;
using discover::Sample;
using hapi::Chain;

// ---- capabilities, consumers, the two drivers that stay as they are (as in discoverCompose R1) ---------------
struct Temperature { using Value = int16_t; static constexpr uint8_t id = 1; };   // 0.1 C
struct Humidity    { using Value = uint8_t; static constexpr uint8_t id = 2; };   // %

struct TempLogger {
  using Accepts = Chain<Temperature>;
  struct Entry { RowId row; int16_t v; };
  inline static Entry   log[16];
  inline static uint8_t n = 0;

  template<typename Cap> struct Body {
    template<typename T> struct Part : T {
      using T::T;
      void on(const Sample<Cap>& s) { if (n < 16) log[n++] = Entry{s.row, s.value}; }
    };
  };
};

struct ValuePrinter {
  using Accepts = Chain<Temperature, Humidity>;
  struct Entry { uint8_t cap; RowId row; int32_t v; };
  inline static Entry   log[16];
  inline static uint8_t n = 0;

  template<typename Cap> struct Body {
    template<typename T> struct Part : T {
      using T::T;
      void on(const Sample<Cap>& s) { if (n < 16) log[n++] = Entry{Cap::id, s.row, int32_t(s.value)}; }
    };
  };
};

using Consumers = Chain<TempLogger, ValuePrinter>;

template<typename W>
struct SensorB : DriverBase<SensorB<W>, W> {
  using B = DriverBase<SensorB<W>, W>;
  using Produces = Chain<Temperature, Humidity>;
  static constexpr uint8_t addrLo = 0x40, addrHi = 0x40, id = 0xB2;

  static void read(RowId row) {
    uint8_t b[3];
    B::readRegs(B::addrOf(row), 1, b, 3);
    B::template emit<Temperature>(row, int16_t(uint16_t((uint16_t(b[0]) << 8) | b[1])));
    B::template emit<Humidity>(row, b[2]);
  }
};

template<typename W>
struct Mux : DriverBase<Mux<W>, W> {
  static constexpr bool    isBridge = true;
  static constexpr uint8_t channels = 2;
  static constexpr uint8_t addrLo = 0x70, addrHi = 0x77, id = 0xC3;

  static void select(uint8_t addr, uint8_t ch) { ctrl(addr, uint8_t(1u << ch)); }
  static void clear(uint8_t addr)               { ctrl(addr, 0); }

private:
  static void ctrl(uint8_t addr, uint8_t mask) {
    using Twi = typename W::Twi;
    Twi::begin_write(addr); Twi::write_byte(1); Twi::write_byte(mask); Twi::end_write();
  }
};

// ---- the edge: what the failure stack needs from SensorA ---------------------------------------------------------
struct BusReset {
  inline static uint16_t count = 0;
  static void run() { ++count; mock::Bus::state = mock::Bus::Idle; mock::Bus::sel = nullptr; mock::Bus::gotPtr = false; }
};

template<typename D> struct AEnv {
  static constexpr uint8_t retryMask = fail::KindSet<fail::Kind::Timeout, fail::Kind::Fault>::mask;
  static constexpr bool    rawStatus = true;     // F1's app has no lifecycle
  static void busReset()                              { BusReset::run(); }
  static void reissue(fail::RowId r)                  { D::reissue(r); }
  // F1's app has no lifecycle (discoverCompose R1's world), so this is the test-only escape, not World::setStatus
  static void setRowState(fail::RowId r, uint8_t s)   { discover::RawStatus::set(D::World::reg, r, discover::Status(s)); }
  static fail::Outcome reprobe(fail::RowId)           { return fail::Outcome::Ok(); }
};

struct BareMode    { using Faults = fail::NoFaults;  template<typename D> using Stack = fail::Bare; };
struct BareInjMode { using Faults = fail::Injected;  template<typename D> using Stack = fail::Bare; };
template<typename... L> struct CtlMode {
  using Faults = fail::Injected;
  template<typename D> using Stack = fail::Controller<AEnv<D>, L...>;
};

#if   FAIL_STEP == 0
using Mode = BareMode;
#elif FAIL_STEP == 1
using Mode = BareInjMode;
#elif FAIL_STEP == 2
using Mode = CtlMode<fail::DetectError, fail::Status>;
#elif FAIL_STEP == 3
using Mode = CtlMode<fail::Recover, fail::DetectError, fail::Status>;
#elif FAIL_STEP == 4
using Mode = CtlMode<fail::TickPart<fail::Retry<3>>, fail::Recover, fail::DetectError,
                     fail::HoldOp<fail::RejectNewest>, fail::Gate<50>, fail::Status>;
#else
using Mode = CtlMode<fail::TickPart<fail::Retry<3>>, fail::Recover, fail::TickPart<fail::DetectDeadline<200>>, fail::DetectError,
                     fail::HoldOp<fail::RejectNewest>, fail::Gate<50>, fail::Status>;
#endif

template<typename W, typename M, uint8_t N>
struct SensorA : DriverBase<SensorA<W, M, N>, W> {
  using B = DriverBase<SensorA<W, M, N>, W>;
  using World    = W;
  using Produces = Chain<Temperature>;
  static constexpr uint8_t addrLo = 0x48, addrHi = 0x48, id = 0xA1;

  using Faults = typename M::Faults;
  using Stack  = typename M::template Stack<SensorA>;      // one per registry row, none at all when bare
  using Tab    = fail::Table<Stack, N>;

  static void read(RowId row)    { serve(row, fail::Cause::Fresh); }
  static void reissue(RowId row) { serve(row, fail::Cause::Reissue); }

  static void tickRow(RowId row, uint32_t now) {
    if constexpr (fail::has_tick<Stack>::value) { auto& s = Tab::at(row); s.bind(row); s.tick(now); }
  }
  [[nodiscard]] static fail::FailStatus failStatus(RowId row) noexcept {
    if constexpr (!__is_empty(Stack)) return row < N ? Tab::at(row).status() : fail::FailStatus{};
    else return fail::FailStatus{};
  }
  static void resetFail() { if constexpr (!__is_empty(Stack)) for (RowId r = 0; r < N; ++r) Tab::at(r) = Stack{}; }

private:
  static void serve(RowId row, fail::Cause c) { Tab::at(row).serve(row, c, [row]() -> fail::Outcome { return attempt(row); }); }

  static fail::Outcome attempt(RowId row) {
    uint8_t b[2];
    B::readRegs(B::addrOf(row), 1, b, 2);
    fail::Outcome o = Faults::after(row, b);
    if (o.isOk()) B::template emit<Temperature>(row, int16_t(uint16_t((uint16_t(b[0]) << 8) | b[1])));
    return o;
  }
};

// ---- application ---------------------------------------------------------------------------------------------------
template<uint8_t N> struct AppN;
template<uint8_t N> using DriversOf = Chain<SensorA<AppN<N>, Mode, N>, SensorB<AppN<N>>, Mux<AppN<N>>>;
template<uint8_t N> struct AppN : discover::World<AppN<N>, mock::Twi, Consumers, DriversOf<N>, N, discover::I2cScan> {};

using App     = AppN<8>;
using SensorAF = SensorA<App, Mode, 8>;
using Ticker  = fail::Ticks<App, DriversOf<8>>;

static_assert(discover::DriverSet<DriversOf<8>>::distinct, "distinct");
static_assert(uint8_t(fail::RowState::Alive) == uint8_t(discover::Status::Alive) && uint8_t(fail::RowState::Stale) == uint8_t(discover::Status::Stale) &&
              uint8_t(fail::RowState::Gone) == uint8_t(discover::Status::Gone), "the edge writes RowState into the registry row as discover::Status");
#ifdef __AVR__
static_assert(sizeof(App::reg.rows[0]) == 5, "discoverCompose's row is untouched: busId + ptr + parent + flags");
#endif
#if FAIL_STEP <= 3
static_assert(!fail::has_tick<SensorAF::Stack>::value && __is_empty(Ticker), "no ticking component chosen: the tick fold is empty");
#endif

// ---- discoverCompose R1's checksum, unchanged, and the failure state on top -----------------------------------------
static uint16_t checksum() {
  uint16_t h = 0;
  for (RowId r = 0; r < App::reg.count; ++r) {
    const auto& row = App::reg.rows[r];
    h = uint16_t(h * 31u + row.busId + row.parent * 7u + row.isBus);
  }
  for (uint8_t i = 0; i < TempLogger::n; ++i)
    h = uint16_t(h * 31u + TempLogger::log[i].row + uint16_t(TempLogger::log[i].v));
  for (uint8_t i = 0; i < ValuePrinter::n; ++i)
    h = uint16_t(h * 31u + ValuePrinter::log[i].cap + ValuePrinter::log[i].row + uint16_t(ValuePrinter::log[i].v));
  return h;
}

#if FAIL_STEP >= 1
using fail::Step;

// mock time is the loop counter; the data path (pump) runs before the expirers (tick), R-6.
//   [0,200)   row 5: two hangs, then the device answers        -> two recoveries, two retries, a late sample
//   [200,600) row 5: four hangs                                  -> exhaustion, row Gone, then Alive again
//   [600,800) healthy
// the tick fold behind one out-of-line function, so its calls can be inspected in the image
__attribute__((noinline)) static void tickAll(uint32_t now) { Ticker::run(now); }

static void scenario() {
  static const Step phase1[] = {Step::hang(), Step::hang()};
  static const Step phase2[] = {Step::hang(), Step::hang(), Step::hang(), Step::hang()};
  App::discover();
  uint8_t every = 0;                                          // a poll every 10th tick, without a division
  for (uint16_t t = 0; t < 800; ++t) {
    if (t == 0)   fail::Injected::load(5, phase1, 2);
    if (t == 200) fail::Injected::load(5, phase2, 4);
    if (every == 0) App::pump();
    if (++every == 10) every = 0;
    tickAll(t);
  }
}

static uint16_t failChecksum() {
  uint16_t h = checksum();
  for (RowId r = 0; r < App::reg.count; ++r) h = uint16_t(h * 31u + unsigned(App::reg.rows[r].status()));
  for (RowId r = 5; r <= 6; ++r) {
    const fail::FailStatus s = SensorAF::failStatus(r);
    h = uint16_t(h * 31u + s.retries); h = uint16_t(h * 31u + s.recovers); h = uint16_t(h * 31u + s.drops);
    h = uint16_t(h * 31u + s.fails);   h = uint16_t(h * 31u + s.lastKind); h = uint16_t(h * 31u + s.lastDetail);
  }
  return uint16_t(h * 31u + fail::Injected::attempts + BusReset::count);
}
#endif

// ---- AVR entry --------------------------------------------------------------------------------------------------------
#ifdef __AVR__
extern "C" void __cxa_pure_virtual() { for (;;) {} }
volatile uint16_t g_sum;
__attribute__((noinline)) void done() { for (;;) {} }
int main() {
#if FAIL_STEP == 0
  App::discover();
  App::pump();
#ifdef BARE_TICK_CALL
  Ticker::run(0);
#endif
  g_sum = checksum();
#else
  scenario();
  g_sum = failChecksum();
#endif
  done();
}
#else

// ---- native ---------------------------------------------------------------------------------------------------------
#include <cstdio>
#include <cstring>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

#if FAIL_STEP == 5
static uint16_t perRow[8];
static void drain() {
  for (uint8_t i = 0; i < TempLogger::n; ++i) ++perRow[TempLogger::log[i].row];
  TempLogger::n = 0; ValuePrinter::n = 0;
}

// each injected fault on the real mock-bus edge (row 6, SensorA on channel 1, reads 253): the kind it becomes, whether a
// sample is delivered, and whether the edge recovers (R-1) and holds the operation for a retry (Timeout and Fault only)
static void injectionTests() {
  struct Case { const char* what; Step step; int kind; uint8_t detail; bool sample; bool recovered; bool held; };
  using fail::Kind;
  const Case cases[] = {
    {"nack",       Step::nack(),      int(Kind::Absent),   0x20, false, false, false},
    {"hang",       Step::hang(),      int(Kind::Timeout),  fail::Injected::maxPolls, false, true,  true},
    {"slow(3)",    Step::slow(3),     0,                   0,    true,  false, false},
    {"slow(9)",    Step::slow(9),     int(Kind::Timeout),  fail::Injected::maxPolls, false, true,  true},
    {"error",      Step::error(),     int(Kind::Fault),    0x08, false, true,  true},
    {"refuse(5)",  Step::refuse(5),   int(Kind::Refused),  5,    false, false, false},
    {"overflow",   Step::overflow(),  int(Kind::Overflow), 1,    false, false, false},
    {"bad data",   Step::badData(),   int(Kind::Corrupt),  0x40, false, false, false},
  };
  for (const Case& c : cases) {
    SensorAF::resetFail(); BusReset::count = 0; fail::Injected::clear();
    for (RowId r = 0; r < App::reg.count; ++r) discover::RawStatus::set(App::reg, r, discover::Status::Alive);   // the edge writes on a change: start each case from Alive
    for (uint8_t i = 0; i < 8; ++i) perRow[i] = 0;
    fail::Injected::load(6, &c.step, 1);
    App::pump(); drain();
    const fail::FailStatus s = SensorAF::failStatus(6);
    const bool ok = c.kind == 0;
    if (!(s.fails == (ok ? 0 : 1) && s.lastKind == c.kind && s.lastDetail == c.detail)) { ++failures; std::printf("FAIL %s: fails=%u kind=%u detail=0x%02X\n", c.what, s.fails, s.lastKind, s.lastDetail); }
    if (!((perRow[6] == 1) == c.sample)) { ++failures; std::printf("FAIL %s: sample delivered=%u\n", c.what, perRow[6]); }
    if (!(s.recovers == (c.recovered ? 1 : 0) && BusReset::count == (c.recovered ? 1 : 0))) { ++failures; std::printf("FAIL %s: recovers=%u resets=%u\n", c.what, s.recovers, BusReset::count); }
    if (!(bool(SensorAF::Tab::at(6).held) == c.held)) { ++failures; std::printf("FAIL %s: held=%d\n", c.what, int(SensorAF::Tab::at(6).held)); }
    if (!(uint8_t(App::reg.rows[6].status()) == (ok ? 0 : 1))) { ++failures; std::printf("FAIL %s: row status %u\n", c.what, unsigned(App::reg.rows[6].status())); }
    if (!(perRow[5] == 1 && perRow[4] == 1)) { ++failures; std::printf("FAIL %s: the other rows were disturbed\n", c.what); }
  }
  fail::Injected::clear();
}

static void detailed() {
  using discover::instOf;
  using fail::Injected;
  mock::Bus::mask = 2;
  App::discover();
  CHECK(App::reg.count == 7 && App::reg.rows[5].drv == instOf<SensorAF>() && App::reg.rows[6].drv == instOf<SensorAF>());
  CHECK(App::reg.rows[5].parent != App::reg.rows[6].parent);
  static const Step p1[] = {Step::hang(), Step::hang()};
  static const Step p2[] = {Step::hang(), Step::hang(), Step::hang(), Step::hang()};
  auto st = [](RowId r) { return SensorAF::failStatus(r); };
  auto rowState = [](RowId r) { return uint8_t(App::reg.rows[r].status()); };

  Injected::load(5, p1, 2);
  for (uint16_t t = 0; t < 200; ++t) {
    if (t % 10 == 0) App::pump();
    Ticker::run(t);
    drain();
    if (t == 0)   { CHECK(rowState(5) == 1 /*Stale*/ && st(5).fails == 1 && st(5).recovers == 1 && BusReset::count == 1); }
    if (t == 99)  { CHECK(rowState(5) == 1 && st(5).retries == 1 && st(5).drops == 9); }     // refused at 10..90; the first re-issue at 50
    if (t == 100) { CHECK(rowState(5) == 0 /*Alive*/ && st(5).retries == 2 && st(5).drops == 10); }
  }
  {
    const fail::FailStatus s = st(5);
    CHECK(s.fails == 2 && s.retries == 2 && s.recovers == 2 && s.drops == 10);
    CHECK(s.lastKind == uint8_t(fail::Kind::Timeout) && s.lastDetail == fail::Injected::maxPolls);
    CHECK(BusReset::count == 2 && Injected::attempts == 12);
    CHECK(perRow[5] == 10);                                   // t=100 late sample + 9 healthy polls (110..190)
    CHECK(perRow[6] == 20 && perRow[4] == 20);                // the identical sensor on the other channel and the different one: untouched
    const fail::FailStatus o = st(6);
    CHECK(o.fails == 0 && o.retries == 0 && o.recovers == 0 && o.drops == 0 && rowState(6) == 0);
  }

  Injected::load(5, p2, 4);
  for (uint16_t t = 200; t < 600; ++t) {
    if (t % 10 == 0) App::pump();
    Ticker::run(t);
    drain();
    if (t == 349) { CHECK(rowState(5) == 1 && st(5).retries == 4); }
    if (t == 350) { CHECK(rowState(5) == 2 /*Gone*/ && App::reg.status(5) == discover::Status::Gone && st(5).retries == 5 && st(5).drops == 26); }
    if (t == 360) { CHECK(rowState(5) == 0 && App::reg.status(5) == discover::Status::Alive); }
  }
  {
    const fail::FailStatus s = st(5);
    CHECK(s.fails == 6 && s.retries == 5 && s.recovers == 6 && s.drops == 26);      // 15 refused + 1 exhausted, on top of phase 1's 10
    CHECK(BusReset::count == 6 && Injected::attempts == 12 + 4 + 24);      // phase 1, the four hangs, and the healthy polls 360..590
    CHECK(perRow[5] == 10 + 24);                              // healthy again from t=360: 360..590
    CHECK(rowState(6) == 0 && st(6).fails == 0);
  }
  for (uint16_t t = 600; t < 800; ++t) { if (t % 10 == 0) App::pump(); Ticker::run(t); drain(); }
  CHECK(st(5).fails == 6 && st(5).drops == 26 && rowState(5) == 0);                   // healthy: nothing more is counted
  CHECK(Injected::attempts == 40 + 20 && BusReset::count == 6);
  injectionTests();
  CHECK(App::reg.count == 7);
  std::printf("per row on the edge: stack %zu B, registry row %zu B (untouched)\n", sizeof(SensorAF::Stack), sizeof(App::reg.rows[0]));
}
#endif

int main(int argc, char** argv) {
  const bool test = argc > 1 && !std::strcmp(argv[1], "test");
#if FAIL_STEP >= 1
  if (!test) { scenario(); std::printf("checksum 0x%04X\n", failChecksum()); return 0; }
#endif
  if (!test) {                                                // FAIL_STEP 0: exactly discoverCompose R1's sequence
    mock::Bus::mask = 2; App::discover(); TempLogger::n = 0; ValuePrinter::n = 0;
    App::pump();
    std::printf("checksum 0x%04X\n", checksum()); return 0;
  }
  // every step: with nothing failing, the edge behaves exactly as discoverCompose R1's (its checksum, 0x5A03)
  {
    App::discover(); TempLogger::n = 0; ValuePrinter::n = 0;
    App::pump();
    const uint16_t c = checksum();
    std::printf("no faults, one pump: checksum 0x%04X (discoverCompose R1: 0x5A03)\n", c);
    CHECK(c == 0x5A03);
#if FAIL_STEP >= 1
    fail::Injected::clear();
#endif
  }
#if FAIL_STEP == 5
  SensorAF::resetFail(); BusReset::count = 0; TempLogger::n = 0; ValuePrinter::n = 0;
  detailed();
#endif
  std::printf(failures ? "FAILED (%d)\n" : "OK: failCompose F1 integration, FAIL_STEP=%d\n", failures ? failures : FAIL_STEP);
  return failures != 0;
}
#endif
