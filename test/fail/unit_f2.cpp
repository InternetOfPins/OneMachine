// failCompose F2, unit level: the return path (_f up the layers, _serve at the top), Reply / Within / Overall, back-off, coalescing.
// No registry, no bus: the operation is a script of Outcomes or a mock conversion, time is an argument.
// -DNEG_* builds the compile-fail cases.
#include <stdint.h>
#include <stdio.h>
#include <type_traits>
#include <oneMachine/fail/layers.h>

using namespace fail;

static int failures;
#define CHECK(...) do { if (!(__VA_ARGS__)) { ++failures; printf("FAIL line %d: %s\n", __LINE__, #__VA_ARGS__); } } while (0)

static bool ok(Outcome o, uint8_t detail) { return o.isOk() && o.detail == detail; }
static bool isFail(Outcome o, Kind k, uint8_t detail) { return o.failed() && o.kind() == k && o.detail == detail; }

// ---- an edge: scripted synchronous operation, or a mock asynchronous conversion, on mock time ---------------------------------
template<bool Idem, bool Async, uint8_t Mask, template<typename> class Stack>
struct Edge2 {
  struct Env {
    static constexpr uint8_t retryMask = Mask;
    static constexpr bool    rawStatus = true, returnPath = true, idempotent = Idem, async = Async;
    static void busReset()                    { ++resets; }
    static void setRowState(RowId, uint8_t s) { ++writes; lastState = s; stateAt = now; }
    static void reissue(RowId r)              { ctl.serve(r, Cause::Reissue, [] { return op(); }); }
    static Outcome result(RowId, Id id)       { return id < 4 && started[id] ? (now >= doneAt[id] ? Outcome::Ok(value[id]) : Outcome::Pending(id)) : Outcome::Idle(); }
  };
  using Ctl = Stack<Env>;
  inline static Ctl        ctl;
  inline static Outcome    script[64];
  inline static int        slen = 0, spos = 0, nops = 0, resets = 0, writes = 0;
  inline static uint8_t    lastState = 0;
  inline static uint32_t   now = 0, stateAt = 0, opTimes[64], doneAt[4], startedAt[4];
  inline static uint8_t    value[4];
  inline static bool       started[4];

  static void reset() {
    ctl = Ctl{}; slen = spos = nops = resets = writes = 0; lastState = 0; now = stateAt = 0;
    for (int i = 0; i < 4; ++i) { doneAt[i] = 0xFFFFFFFFu; value[i] = 0; started[i] = false; }
  }
  static void load(std::initializer_list<Outcome> l) { slen = 0; spos = 0; for (auto o : l) script[slen++] = o; }
  static Outcome op() { opTimes[nops++ % 64] = now; return spos < slen ? script[spos++] : Outcome::Ok(); }
  static void poll() { ctl.serve(0, Cause::Fresh, [] { return op(); }); }
  // an asynchronous operation: accepted now, done `ms` later with `v`
  static void start(Id id, uint32_t ms, uint8_t v) {
    ctl.serve(0, Cause::Fresh, [=] { started[id] = true; doneAt[id] = ms == 0xFFFFFFFFu ? ms : now + ms; value[id] = v; startedAt[id] = now; return Outcome::Pending(id); });
  }
  static void step(uint32_t t, bool doPoll = false) { now = t; if (doPoll) poll(); if constexpr (has_tick<Ctl>::value) ctl.tick(t); }
};

// ---- stacks --------------------------------------------------------------------------------------------------------------------------------
template<typename E> using NoRetry   = Controller<E, CausePart<Reply<false>>, DetectError, Status>;
template<typename E> using WithRetry = Controller<E, CausePart<Reply<false>>, TickPart<Retry<3>>, DetectError, HoldOp<RejectNewest>, Gate<50>, Status>;
template<typename E> using AsyncSticky   = Controller<E, CausePart<Reply<false, 1>>, TickPart<CausePart<Within<100, 1>>>, Status>;
template<typename E> using AsyncOnce     = Controller<E, CausePart<Reply<true, 1>>,  TickPart<CausePart<Within<100, 1>>>, Status>;
template<typename E> using AsyncMany     = Controller<E, CausePart<Reply<false, 3>>, TickPart<CausePart<Within<200, 3>>>, Status>;
template<typename E> using PerAttempt    = Controller<E, TickPart<Retry<5>>, DetectError, HoldOp<RejectNewest>, Gate<100>, Status>;
template<typename E> using OverallLoop   = Controller<E, TickPart<CausePart<Overall<350>>>, TickPart<Retry<5>>, DetectError, HoldOp<RejectNewest>, Gate<100>, Status>;
template<typename E> using CoalescedRead = Controller<E, CausePart<Reply<false>>, TickPart<Retry<3>>, DetectError, HoldOp<Coalesce>, Gate<50>, Status>;
template<typename E> using RefusedOp     = Controller<E, CausePart<Reply<false>>, TickPart<Retry<3>>, DetectError, HoldOp<RejectNewest>, Gate<50>, Status>;
template<typename E> using Backing       = Controller<E, TickPart<Retry<0>>, DetectError, HoldOp<RejectNewest>, Backoff<50, 400>, Status>;

#if defined(NEG_CAUSE_HIDES) || defined(NEG_CAUSE_BODY)
struct RawCause { template<typename T> struct Part : T { Outcome _f(Id) { return Outcome::Ok(); } }; };
using BadBody = RawCause;
#endif
#ifdef NEG_REPLY_NO_RETURNPATH
struct NoRp { static constexpr bool rawStatus = true; static void busReset() {} static void setRowState(RowId, uint8_t) {} static void reissue(RowId) {} };
#endif

static constexpr uint8_t kAbsent = uint8_t(1u << (uint8_t(Kind::Absent) - 1));

// ---- 1. the return path, synchronous --------------------------------------------------------------------------------------------------------
static void syncTests() {
  using N = Edge2<false, false, 0, NoRetry>;
  N::reset(); N::load({Outcome::Fail(Kind::Absent, 0x20)});
  N::step(0, true);
  CHECK(isFail(N::ctl._serve(0), Kind::Absent, 0x20));                 // no Retry: the NACK surfaces at the top
  CHECK(isFail(N::ctl._serve(0), Kind::Absent, 0x20));                 // sticky: the same until the next f
  CHECK(N::ctl.status().fails == 1);
  N::step(10, true);                                                   // the next f: Ok replaces it
  CHECK(N::ctl._serve(0).isOk());

  using R = Edge2<false, false, kAbsent, WithRetry>;
  R::reset(); R::load({Outcome::Fail(Kind::Absent, 0x20), Outcome::Ok()});
  CHECK(R::ctl._serve(0).isIdle());                                    // nothing issued yet
  R::step(0, true);
  CHECK(R::ctl._serve(0).isPending());                                 // with Retry the failure is absorbed: the operation is in flight
  for (uint32_t t = 1; t < 50; ++t) R::step(t);
  CHECK(R::ctl._serve(0).isPending());
  R::step(50);                                                         // the gated re-issue succeeds
  CHECK(R::ctl._serve(0).isOk());                                      // absorbed: the top sees Ok
  CHECK(R::ctl.status().fails == 1 && R::ctl.status().retries == 1);   // Status counted both
  CHECK(R::nops == 2 && R::opTimes[1] == 50);

  // exhaustion comes out as the failure
  R::reset(); R::load({Outcome::Fail(Kind::Absent, 1), Outcome::Fail(Kind::Absent, 2), Outcome::Fail(Kind::Absent, 3), Outcome::Fail(Kind::Absent, 4)});
  R::step(0, true);
  for (uint32_t t = 1; t <= 150; ++t) R::step(t);
  CHECK(isFail(R::ctl._serve(0), Kind::Absent, 4) && R::ctl.status().drops == 1 && R::lastState == uint8_t(RowState::Gone));
  CHECK(!R::ctl.held);
}

// ---- 2. asynchronous, read-once and sticky, and the deadline -----------------------------------------------------------------------------
static void asyncTests() {
  using S = Edge2<false, true, 0, AsyncSticky>;
  S::reset(); S::now = 0;
  S::start(0, 30, 0x5A); S::step(0);
  CHECK(S::ctl._serve(0).isPending());
  for (uint32_t t = 1; t < 30; ++t) S::step(t);
  CHECK(S::ctl._serve(0).isPending());
  S::step(30);
  CHECK(ok(S::ctl._serve(0), 0x5A));                                // done, with the value
  CHECK(ok(S::ctl._serve(0), 0x5A));                                // sticky: still there
  S::step(31); CHECK(ok(S::ctl._serve(0), 0x5A));
  S::now = 40; S::start(0, 30, 0x77);                                  // the next f replaces it
  CHECK(S::ctl._serve(0).isPending());

  using O = Edge2<false, true, 0, AsyncOnce>;
  O::reset(); O::start(0, 30, 0x5A);
  for (uint32_t t = 0; t <= 30; ++t) O::step(t);
  CHECK(ok(O::ctl._serve(0), 0x5A));
  CHECK(O::ctl._serve(0).isIdle());                                    // read once: consumed
  CHECK(O::ctl._serve(0).isIdle());
  O::now = 50; O::start(0, 10, 0x11);                                  // a new operation
  for (uint32_t t = 50; t <= 60; ++t) O::step(t);
  CHECK(ok(O::ctl._serve(0), 0x11) && O::ctl._serve(0).isIdle());

  // it never completes: the deadline turns the silence into a failure on the same path (R-2)
  S::reset(); S::start(0, 0xFFFFFFFFu, 0);
  for (uint32_t t = 0; t < 100; ++t) S::step(t);
  CHECK(S::ctl._serve(0).isPending());                                 // armed on the first tick (t=0), due at 100
  S::step(100);
  CHECK(isFail(S::ctl._serve(0), Kind::Timeout, 0));
  CHECK(isFail(S::ctl._serve(0), Kind::Timeout, 0));                   // sticky
  CHECK(S::ctl.status().fails == 1 && S::ctl.status().lastKind == uint8_t(Kind::Timeout));   // counted once
  O::reset(); O::start(0, 0xFFFFFFFFu, 0);
  for (uint32_t t = 0; t <= 100; ++t) O::step(t);
  CHECK(isFail(O::ctl._serve(0), Kind::Timeout, 0) && O::ctl._serve(0).isIdle());
}

// ---- 3. correlation --------------------------------------------------------------------------------------------------------------------------------
static void correlationTests() {
  using M = Edge2<false, true, 0, AsyncMany>;
  M::reset();
  M::start(0, 50, 0xA0); M::start(1, 20, 0xA1); M::start(2, 0xFFFFFFFFu, 0xA2);
  for (uint32_t t = 0; t <= 25; ++t) M::step(t);
  CHECK(M::ctl._serve(0, 0).isPending() && ok(M::ctl._serve(0, 1), 0xA1) && M::ctl._serve(0, 2).isPending());
  for (uint32_t t = 26; t <= 55; ++t) M::step(t);
  CHECK(ok(M::ctl._serve(0, 0), 0xA0) && ok(M::ctl._serve(0, 1), 0xA1) && M::ctl._serve(0, 2).isPending());
  for (uint32_t t = 56; t <= 205; ++t) M::step(t);
  CHECK(isFail(M::ctl._serve(0, 2), Kind::Timeout, 2));                // only the one that never answered
  CHECK(ok(M::ctl._serve(0, 0), 0xA0));
  CHECK(isFail(M::ctl._serve(0, 5), Kind::Refused, 5));                // an id that is not one of the three
  CHECK(M::ctl.status().fails == 1);
}

// ---- 4. R-5: a deadline per attempt against one around the retry loop ------------------------------------------------------------
static void r5Tests() {
  using A = Edge2<false, false, kAbsent, PerAttempt>;
  A::reset(); A::load({});
  for (int i = 0; i < 64; ++i) A::script[i] = Outcome::Fail(Kind::Absent, 1);
  A::slen = 64;
  A::step(0, true);
  for (uint32_t t = 1; t <= 700; ++t) A::step(t);
  CHECK(A::nops == 6 && A::lastState == uint8_t(RowState::Gone) && A::stateAt == 500);      // per attempt: 6 operations, gives up at 500 ms

  using O = Edge2<false, false, kAbsent, OverallLoop>;
  O::reset(); O::slen = 64;
  for (int i = 0; i < 64; ++i) O::script[i] = Outcome::Fail(Kind::Absent, 1);
  O::step(0, true);
  for (uint32_t t = 1; t <= 700; ++t) O::step(t);
  CHECK(O::nops == 4 && O::lastState == uint8_t(RowState::Gone) && O::stateAt == 350);      // overall 350: 4 operations, gives up at 350 ms
  CHECK(O::opTimes[0] == 0 && O::opTimes[1] == 100 && O::opTimes[2] == 200 && O::opTimes[3] == 300);
  CHECK(O::ctl.status().retries == 3 && O::ctl.status().drops == 1 && O::ctl.status().lastKind == uint8_t(Kind::Timeout));

  // a loop that ends in time is not cut
  O::reset(); O::load({Outcome::Fail(Kind::Absent, 1), Outcome::Fail(Kind::Absent, 1), Outcome::Ok()});
  O::step(0, true);
  for (uint32_t t = 1; t <= 700; ++t) O::step(t);
  CHECK(O::nops == 3 && O::lastState == uint8_t(RowState::Alive) && O::ctl.status().drops == 0);
}

// ---- 5. coalescing -----------------------------------------------------------------------------------------------------------------------------
static void coalesceTests() {
  using C = Edge2<true, false, kAbsent, CoalescedRead>;
  C::reset(); C::load({Outcome::Fail(Kind::Absent, 1)});
  for (uint32_t t = 0; t <= 60; ++t) C::step(t, t % 10 == 0);           // a fresh poll every 10 ms while the retry is held
  CHECK(C::nops == 3 && C::ctl.status().drops == 0 && C::ctl.status().retries == 1);   // the poll, the one re-issue, the healthy poll at 60: no refusal counted, no extra operation
  CHECK(C::ctl._serve(0).isOk());

  using R = Edge2<false, false, kAbsent, RefusedOp>;                    // not idempotent: refused, as R-3 says
  R::reset(); R::load({Outcome::Fail(Kind::Absent, 1)});
  for (uint32_t t = 0; t <= 60; ++t) R::step(t, t % 10 == 0);
  CHECK(R::nops == 3 && R::ctl.status().drops == 5 && R::ctl.status().retries == 1);   // the polls at 10..50 refused (the one at 50 comes before the tick that re-issues)
}

// ---- 6. back-off -----------------------------------------------------------------------------------------------------------------------------------
static void backoffTests() {
  using B = Edge2<false, false, kAbsent, Backing>;
  B::reset(); B::slen = 64;
  for (int i = 0; i < 64; ++i) B::script[i] = Outcome::Fail(Kind::Absent, 1);
  B::step(0, true);
  for (uint32_t t = 1; t <= 1400; ++t) B::step(t);
  // fail at 0; re-issues at +50, +100, +200, +400, then every 400: 50, 150, 350, 750, 1150
  CHECK(B::nops == 6 && B::opTimes[1] == 50 && B::opTimes[2] == 150 && B::opTimes[3] == 350 && B::opTimes[4] == 750 && B::opTimes[5] == 1150);
  CHECK(B::lastState == uint8_t(RowState::Stale) && B::ctl.status().drops == 0);   // never Gone: it stays Stale on its back-off
  // recovered: the next failure starts over at the shortest interval
  B::spos = 64; B::slen = 64;                                          // script exhausted -> Ok
  B::script[64 - 1] = Outcome::Ok(); B::spos = 63;
  B::step(1501);                                                       // (a re-issue may fall here or not; run on)
  for (uint32_t t = 1502; t <= 2000; ++t) B::step(t);
  CHECK(B::lastState == uint8_t(RowState::Alive));
  const int before = B::nops;
  B::slen = 1; B::spos = 0; B::script[0] = Outcome::Fail(Kind::Absent, 1);
  B::step(2100, true);
  for (uint32_t t = 2101; t <= 2200; ++t) B::step(t);
  CHECK(B::nops == before + 2 && B::opTimes[before] == 2100 && B::opTimes[before + 1] == 2150);   // 50 ms again, not the grown one
}

int main() {
  syncTests(); asyncTests(); correlationTests(); r5Tests(); coalesceTests(); backoffTests();

#ifdef NEG_CAUSE_HIDES
  Controller<Edge2<false, false, 0, NoRetry>::Env, RawCause, Status> x; (void)x;
#endif
#ifdef NEG_CAUSE_BODY
  Controller<Edge2<false, false, 0, NoRetry>::Env, CausePart<BadBody>, Status> x; (void)x;
#endif
#ifdef NEG_ASYNC_NO_DEADLINE
  using E = Edge2<false, true, 0, AsyncSticky>::Env;
  Controller<E, CausePart<Reply<false>>, Status> x; (void)x;
#endif
#ifdef NEG_REPLY_NO_RETURNPATH
  Controller<NoRp, CausePart<Reply<false>>, Status> x; (void)x;
#endif
#ifdef NEG_OVERALL_BELOW_RETRY
  using E = Edge2<false, false, kAbsent, PerAttempt>::Env;
  Controller<E, TickPart<Retry<3>>, TickPart<CausePart<Overall<350>>>, DetectError, HoldOp<RejectNewest>, Gate<100>, Status> x; (void)x;
#endif
#ifdef NEG_OVERALL_NO_RETRY
  using E = Edge2<false, false, 0, NoRetry>::Env;
  Controller<E, TickPart<CausePart<Overall<350>>>, DetectError, Status> x; (void)x;
#endif
#ifdef NEG_COALESCE_NOT_IDEMPOTENT
  using E = Edge2<false, false, kAbsent, RefusedOp>::Env;
  Controller<E, TickPart<Retry<3>>, DetectError, HoldOp<Coalesce>, Gate<50>, Status> x; (void)x;
#endif
  printf(failures ? "FAILED (%d)\n" : "OK: failCompose F2 unit\n", failures);
  return failures != 0;
}
