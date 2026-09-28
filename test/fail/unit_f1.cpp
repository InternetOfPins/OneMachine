// failCompose F1, unit level: Deadline, and the controller stack on a scripted edge with mock time.
// No registry, no bus: the operation is a script of Outcomes, time is an argument. -DNEG_* builds the compile-fail cases.
#include <stdint.h>
#include <stdio.h>
#include <type_traits>
#include <oneMachine/fail/layers.h>

using namespace fail;

static int failures;
#define CHECK(c) do { if (!(c)) { ++failures; printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

// ---- a scripted edge ------------------------------------------------------------------------------------
template<uint8_t Mask> struct TEnv {
  static constexpr uint8_t retryMask = Mask;
  static constexpr bool    rawStatus = true;     // a unit-level edge: no registry, the row state is only observed
  inline static int     resets = 0;
  inline static uint8_t rowStateSeen = 0;
  static void busReset()                     { ++resets; }
  static void setRowState(RowId, uint8_t s)  { rowStateSeen = s; }
  static void reissue(RowId r);
};

template<uint8_t Mask> using Full = Controller<TEnv<Mask>,
  TickPart<Retry<3>>, Recover, TickPart<DetectDeadline<200>>, DetectError, HoldOp<RejectNewest>, Gate<50>, Status>;

template<uint8_t Mask> struct Rig {
  inline static Full<Mask> ctl;
  inline static Outcome    script[64];
  inline static int        slen = 0, spos = 0, nops = 0;
  inline static uint32_t   opTimes[64], now = 0;

  static void reset() { ctl = Full<Mask>{}; slen = spos = nops = 0; TEnv<Mask>::resets = 0; TEnv<Mask>::rowStateSeen = 0; }
  static void load(std::initializer_list<Outcome> l) { slen = 0; spos = 0; for (auto o : l) script[slen++] = o; }
  static Outcome op() { opTimes[nops++ % 64] = now; return spos < slen ? script[spos++] : Outcome::Ok(); }
  static void poll()  { ctl.serve(0, Cause::Fresh, [] { return op(); }); }
  static void reissueOp(RowId r) { ctl.serve(r, Cause::Reissue, [] { return op(); }); }
  // poll (if asked), then tick: R-6, the data path before the expirers
  static void step(uint32_t t, bool doPoll) { now = t; if (doPoll) poll(); ctl.tick(t); }
};
template<uint8_t Mask> void TEnv<Mask>::reissue(RowId r) { Rig<Mask>::reissueOp(r); }

static constexpr Kind kinds[] = {Kind::Absent, Kind::Timeout, Kind::Refused, Kind::Overflow, Kind::Corrupt, Kind::Fault, Kind::Unknown};
static bool recovers(Kind k) { return k == Kind::Timeout || k == Kind::Fault; }   // R-1, restated independently

// ---- Recover is per edge: an edge may declare kinds of its own, and its recovery may need the row --------------------------------------------
// (a device that lost its state answers, reports Corrupt, and is re-initialised; a bus is reset. Timeout and Fault stay recoverable everywhere.)
struct REnv {
  static constexpr uint8_t retryMask = KindSet<Kind::Corrupt, Kind::Timeout>::mask;
  static constexpr uint8_t recoverMask = KindSet<Kind::Corrupt>::mask;
  static constexpr bool    rawStatus = true;
  inline static int recovered = 0, resets = 0; inline static RowId lastRow = 0xFF;
  static void busReset()                    { ++resets; }
  static void recover(RowId r)              { ++recovered; lastRow = r; }
  static void setRowState(RowId, uint8_t)   {}
  static void reissue(RowId);
};
using RCtl = Controller<REnv, TickPart<Retry<3>>, Recover, DetectError, HoldOp<RejectNewest>, Gate<50>, Status>;
static RCtl rctl; static Outcome rscript[8]; static int rlen = 0, rpos = 0;
static Outcome rop() { return rpos < rlen ? rscript[rpos++] : Outcome::Ok(); }
void REnv::reissue(RowId r) { rctl.serve(r, Cause::Reissue, [] { return rop(); }); }

static void recoverPerEdgeTests() {
  // a kind the edge declared: the edge's own recovery runs, with the row, and the operation is retried and succeeds
  rctl = RCtl{}; REnv::recovered = REnv::resets = 0; REnv::lastRow = 0xFF; rlen = 2; rpos = 0; rscript[0] = Outcome::Fail(Kind::Corrupt, 0x40); rscript[1] = Outcome::Ok();
  rctl.serve(7, Cause::Fresh, [] { return rop(); });
  CHECK(REnv::recovered == 1 && REnv::lastRow == 7 && REnv::resets == 0);       // recover(row), not busReset()
  for (uint32_t t = 0; t < 300; t += 10) rctl.tick(t);
  { const FailStatus s = rctl.status(); CHECK(s.recovers == 1 && s.retries == 1 && s.fails == 1); }
  CHECK(REnv::recovered == 1 && rpos == 2);                                      // one recovery, one retry, done
  // Timeout stays recoverable on every edge (R-1), through the edge's recovery
  rctl = RCtl{}; REnv::recovered = 0; rlen = 1; rpos = 0; rscript[0] = Outcome::Fail(Kind::Timeout, 2);
  rctl.serve(3, Cause::Fresh, [] { return rop(); });
  CHECK(REnv::recovered == 1 && REnv::lastRow == 3);
  // a kind nobody declared is not recovered
  rctl = RCtl{}; REnv::recovered = 0; rlen = 1; rpos = 0; rscript[0] = Outcome::Fail(Kind::Refused, 1);
  rctl.serve(3, Cause::Fresh, [] { return rop(); });
  CHECK(REnv::recovered == 0 && rctl.status().recovers == 0);
  // an edge that declares nothing keeps R-1 and busReset() (the existing tests above); Corrupt is not recovered there
  Rig<0x7F>::reset(); Rig<0x7F>::load({Outcome::Fail(Kind::Corrupt, 1), Outcome::Ok()});
  Rig<0x7F>::poll();
  CHECK(Rig<0x7F>::ctl.status().recovers == 0 && TEnv<0x7F>::resets == 0);
}

// ---- Deadline ---------------------------------------------------------------------------------------------
static uint64_t rng = 0x9E3779B97F4A7C15ull;
static uint64_t rnd() { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return rng; }

static void deadlineTests() {
  // unarmed is never due, at any time, including now = 0
  { Deadline d; for (uint32_t n : {0u, 1u, 0x7FFFFFFFu, 0x80000000u, 0xFFFFFFFFu}) CHECK(!d.due(n));
    for (int i = 0; i < 1000; ++i) CHECK(!d.due(uint32_t(rnd()))); }
  // disarm makes an armed one never due again
  { Deadline d; d.arm(10, 5); CHECK(d.due(15)); d.disarm(); CHECK(!d.due(15)); CHECK(!d.due(1000)); }
  // arm(0, 0) is due at once; it is armed, not unarmed
  { Deadline d; d.arm(0, 0); CHECK(d.due(0)); }

  const uint64_t wrap = 1ull << 32;
  long bad = 0;
  auto dense = [&](uint64_t start, uint32_t ms) {            // polled every ms: due exactly at +ms, never before
    Deadline d; d.arm(uint32_t(start), ms);
    for (uint32_t e = 0; e <= ms + 2; ++e) { bool due = d.due(uint32_t(start + e)); if ((e < ms && due) || (e >= ms && !due)) ++bad; }
  };
  auto sparse = [&](uint64_t start, uint32_t ms, uint32_t late) {   // three polls: just before, on time, and long after
    Deadline d; d.arm(uint32_t(start), ms);
    if (ms && d.due(uint32_t(start + ms - 1))) ++bad;
    if (!d.due(uint32_t(start + ms))) ++bad;
    if (!d.due(uint32_t(start + ms + late))) ++bad;           // first poll long past the deadline, possibly across the wrap
  };
  for (uint32_t ms : {1u, 2u, 30u, 1000u})
    for (uint64_t s = wrap - 2ull * ms - 6; s <= wrap + 6; ++s) dense(s, ms);
  for (uint32_t ms : {4096u, 100000u, 1u << 20})
    for (uint64_t s = wrap - 2ull * ms - 6; s <= wrap + 6; ++s) sparse(s, ms, uint32_t(rnd() % 100000u));
  for (int i = 0; i < 100000; ++i) sparse(uint32_t(rnd()), 1 + uint32_t(rnd() % 100000u), uint32_t(rnd() % 1000000u));
  CHECK(bad == 0);
  if (bad) printf("  deadline wrap: %ld bad polls\n", bad);

  // the two documented wrap modes, spelled out
  { Deadline d; d.arm(0xFFFFFF00u, 1000);                    // armed within Ms before the wrap: not early, on time
    CHECK(!d.due(0xFFFFFF00u)); CHECK(!d.due(0xFFFFFF00u + 999u)); CHECK(d.due(0xFFFFFF00u + 1000u)); }
  { Deadline d; d.arm(0xFFFFFC00u, 1000);                    // first polled after the wrap: not missed
    CHECK(d.due(0x10u)); }
}

// ---- Status and counters per injected kind -------------------------------------------------------------------
static void kindTests() {
  using R = Rig<0>;                                          // an edge that retries nothing, to isolate one failure
  for (Kind k : kinds) {
    R::reset(); R::load({Outcome::Fail(k, uint8_t(0x40 + uint8_t(k)))});
    R::step(0, true);
    const FailStatus s = R::ctl.status();
    CHECK(s.fails == 1 && s.lastKind == uint8_t(k) && s.lastDetail == uint8_t(0x40 + uint8_t(k)));
    CHECK(s.retries == 0 && s.drops == 0);
    CHECK(s.recovers == (recovers(k) ? 1 : 0));               // R-1 with no retry in the way
    CHECK(TEnv<0>::resets == s.recovers);
    CHECK(TEnv<0>::rowStateSeen == uint8_t(RowState::Stale));
    CHECK(!R::ctl.held);
    R::step(10, true);                                       // the next fresh poll passes: Alive again, counters keep
    CHECK(TEnv<0>::rowStateSeen == uint8_t(RowState::Alive));
    CHECK(R::ctl.status().fails == 1 && R::ctl.status().lastKind == uint8_t(k));
  }
}

// R-1 holds on an edge that would retry every kind
static void recoverRuleTests() {
  using R = Rig<0x7F>;
  for (Kind k : kinds) {
    R::reset(); R::load({Outcome::Fail(k, 1)});
    R::step(0, true);
    for (uint32_t t = 1; t <= 120; ++t) R::step(t, false);   // the gate re-issues once, which passes
    const FailStatus s = R::ctl.status();
    CHECK(s.retries == 1);
    CHECK(s.recovers == (recovers(k) ? 1 : 0));
    CHECK(TEnv<0x7F>::resets == (recovers(k) ? 1 : 0));
  }
}

// ---- retry is keyed on (edge, kind) ------------------------------------------------------------------------------
static void keyedTests() {
  constexpr uint8_t X = KindSet<Kind::Timeout, Kind::Fault>::mask;            // a busy or noisy bus
  constexpr uint8_t Y = KindSet<Kind::Corrupt, Kind::Absent>::mask;           // a lossy link, a device that may come back
  for (Kind k : kinds) {
    const bool x = (X & bit(k)) != 0, y = (Y & bit(k)) != 0;
    Rig<X>::reset(); Rig<X>::load({Outcome::Fail(k, 1)});
    Rig<X>::step(0, true); for (uint32_t t = 1; t <= 200; ++t) Rig<X>::step(t, false);
    CHECK(Rig<X>::nops == (x ? 2 : 1)); CHECK(Rig<X>::ctl.status().retries == (x ? 1 : 0));
    CHECK(TEnv<X>::rowStateSeen == uint8_t(x ? RowState::Alive : RowState::Stale));
    Rig<Y>::reset(); Rig<Y>::load({Outcome::Fail(k, 1)});
    Rig<Y>::step(0, true); for (uint32_t t = 1; t <= 200; ++t) Rig<Y>::step(t, false);
    CHECK(Rig<Y>::nops == (y ? 2 : 1)); CHECK(Rig<Y>::ctl.status().retries == (y ? 1 : 0));
  }
  // the same kind is retried on one edge and not on the other
  CHECK((X & bit(Kind::Timeout)) && !(Y & bit(Kind::Timeout)));
  CHECK(!(X & bit(Kind::Corrupt)) && (Y & bit(Kind::Corrupt)));
}

// exhaustion is counted, marks the row Gone, and stops the operations
static void exhaustionTests() {
  using R = Rig<KindSet<Kind::Timeout>::mask>; constexpr uint8_t M = KindSet<Kind::Timeout>::mask;
  R::reset(); R::load({Outcome::Fail(Kind::Timeout, 8), Outcome::Fail(Kind::Timeout, 8),
                       Outcome::Fail(Kind::Timeout, 8), Outcome::Fail(Kind::Timeout, 8)});
  R::step(100, true); for (uint32_t t = 101; t <= 600; ++t) R::step(t, false);
  const FailStatus s = R::ctl.status();
  CHECK(R::nops == 4);                                       // one fresh + Max(3) re-issues, then no more
  CHECK(s.retries == 3 && s.drops == 1 && s.fails == 4 && s.recovers == 4);
  CHECK(TEnv<M>::resets == 4 && TEnv<M>::rowStateSeen == uint8_t(RowState::Gone));
  CHECK(!R::ctl.held);
  R::step(700, true);                                        // the device is back: the next fresh poll brings the row back
  CHECK(TEnv<M>::rowStateSeen == uint8_t(RowState::Alive) && R::ctl.status().drops == 1);
}

// ---- gate spacing on mock time ----------------------------------------------------------------------------------------
static void gateTests() {
  using R = Rig<KindSet<Kind::Timeout>::mask>;
  const Outcome t8 = Outcome::Fail(Kind::Timeout, 8);
  R::reset(); R::load({t8, t8, t8});                          // fresh at 100 fails, three re-issues: the third passes
  R::step(100, true); for (uint32_t t = 101; t <= 400; ++t) R::step(t, false);
  CHECK(R::nops == 4);
  CHECK(R::opTimes[0] == 100 && R::opTimes[1] == 150 && R::opTimes[2] == 200 && R::opTimes[3] == 250);
  // ticks every 7 ms: the first tick at or after each due time, the gate re-armed from the tick that re-issued
  R::reset(); R::load({t8, t8, t8});
  R::step(100, true); for (uint32_t t = 107; t <= 400; t += 7) R::step(t, false);
  CHECK(R::nops == 4);
  CHECK(R::opTimes[0] == 100 && R::opTimes[1] == 156 && R::opTimes[2] == 212 && R::opTimes[3] == 268);
  // R-6: with the tick before the poll, the gate is armed one tick later: the first re-issue shifts by one, the spacing stays 50
  R::reset(); R::load({t8, t8, t8});
  for (uint32_t t = 100; t <= 400; ++t) { R::now = t; R::ctl.tick(t); if (t == 100) R::poll(); }
  CHECK(R::nops == 4);
  CHECK(R::opTimes[0] == 100 && R::opTimes[1] == 151 && R::opTimes[2] == 201 && R::opTimes[3] == 251);
  // a fresh poll while an operation is held is refused and counted, and never touches the edge
  R::reset(); R::load({t8});
  R::step(0, true); R::step(10, true); R::step(20, true);
  CHECK(R::nops == 1 && R::ctl.status().drops == 2 && R::ctl.held);
}

// ---- deadline-form Detect ------------------------------------------------------------------------------------------------
static void staleTests() {
  using R = Rig<0>;
  R::reset();
  R::step(0, true);                                          // one success at t = 0, then silence
  for (uint32_t t = 1; t <= 199; ++t) R::step(t, false);
  CHECK(R::ctl.status().fails == 0);
  R::step(200, false);
  CHECK(R::ctl.status().fails == 1 && R::ctl.status().lastKind == uint8_t(Kind::Timeout) && R::ctl.status().lastDetail == 0xDD);
  CHECK(TEnv<0>::rowStateSeen == uint8_t(RowState::Stale));
  for (uint32_t t = 201; t <= 400; ++t) R::step(t, false);
  CHECK(R::ctl.status().fails == 1);                          // one event per episode
  CHECK(TEnv<0>::resets == 0);                                // an elapsed Detect reports; it does not act
  R::step(410, true);                                        // a success re-arms it, from the tick that saw it
  CHECK(TEnv<0>::rowStateSeen == uint8_t(RowState::Alive));
  for (uint32_t t = 411; t <= 609; ++t) R::step(t, false);
  CHECK(R::ctl.status().fails == 1);
  R::step(610, false);
  CHECK(R::ctl.status().fails == 2);
}

// ---- the same, across the millis() wrap ---------------------------------------------------------------------------------------
static void wrapTests() {
  using R = Rig<KindSet<Kind::Timeout>::mask>;
  const Outcome t8 = Outcome::Fail(Kind::Timeout, 8);
  const uint32_t base = 0xFFFFFFF0u;                          // 16 ms before the wrap
  R::reset(); R::load({t8, t8});
  R::step(base, true); for (uint32_t i = 1; i <= 200; ++i) R::step(base + i, false);
  CHECK(R::nops == 3);
  CHECK(R::opTimes[0] == base && R::opTimes[1] == base + 50 && R::opTimes[2] == base + 100);   // exact spacing across the wrap
  // deadline-form across the wrap: last success just before it, the event exactly 200 ms later
  using Q = Rig<0>;
  Q::reset(); Q::step(base, true);
  for (uint32_t i = 1; i <= 199; ++i) Q::step(base + i, false);
  CHECK(Q::ctl.status().fails == 0);
  Q::step(base + 200, false);
  CHECK(Q::ctl.status().fails == 1);
}

struct OkOp { Outcome operator()() const { return Outcome::Ok(); } };

static void faceTests() {
  using R = Rig<0>;
  static_assert(std::is_void<decltype(R::ctl.serve(0, Cause::Fresh, std::declval<OkOp>()))>::value, "outer face: serve() returns nothing");
  static_assert(noexcept(R::ctl.status()), "the status query cannot fail");
  static_assert(has_tick<Full<0>>::value, "a stack with TickPart layers has tick(now)");
  static_assert(!has_tick<Controller<TEnv<0>, DetectError, Status>>::value, "a stack without one does not");
  static_assert(__is_empty(Bare), "bare has no state");
  static_assert(sizeof(Table<Bare, 8>) == 1 && __is_empty(Table<Bare, 8>), "bare has no per-row table");
  printf("sizeof(full stack, one row) = %zu B  (Status %zu, Deadline %zu)\n", sizeof(Full<0>), sizeof(FailStatus), sizeof(Deadline));
}

int main() {
  deadlineTests(); kindTests(); recoverRuleTests(); recoverPerEdgeTests(); keyedTests(); exhaustionTests(); gateTests(); staleTests(); wrapTests(); faceTests();
  printf(failures ? "FAILED (%d)\n" : "OK: failCompose F1 unit\n", failures);
  return failures != 0;
}

// ---- compile-fail cases (each must be rejected with its own message) ----------------------------------------------------------
#ifdef NEG_TICK_HIDES
struct HidingLayer { template<typename T> struct Part : T { void tick(uint32_t) {} }; };
static_assert(sizeof(Controller<TEnv<0>, HidingLayer, DetectError, Status>) > 0, "");
#endif
#ifdef NEG_TICK_BODY
struct BodyWithTick { template<typename T> struct Part : T { void tick(uint32_t) {} }; };
static_assert(sizeof(Controller<TEnv<0>, TickPart<BodyWithTick>, DetectError, Status>) > 0, "");
#endif
#ifdef NEG_RETRY_OVERWRITE
static_assert(sizeof(Controller<TEnv<0>, TickPart<Retry<3>>, HoldOp<Overwrite>, Gate<50>, Status>) > 0, "");
#endif
#ifdef NEG_RECOVER_ABOVE_RETRY
static_assert(sizeof(Controller<TEnv<0>, Recover, TickPart<Retry<3>>, HoldOp<RejectNewest>, Gate<50>, Status>) > 0, "");
#endif
#ifdef NEG_RETRY_NO_GATE
static_assert(sizeof(Controller<TEnv<0>, TickPart<Retry<3>>, HoldOp<RejectNewest>, Status>) > 0, "");
#endif
#ifdef NEG_NO_STATUS
static_assert(sizeof(Controller<TEnv<0>, DetectError>) > 0, "");
#endif
