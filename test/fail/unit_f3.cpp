// failCompose F3, unit level: the delivery edge on mock time, with mock consumers. The consumer classes, one store each, the failure layers
// below them, the report on _deliver(), and the compile-time rules of each class (-DNEG_* must be rejected with their own message).
#include <cstdio>
#include "../support/mockdev.h"

using namespace fail;
using namespace mockdev;

static int failures = 0;
#define CHECK(...) do { if (!(__VA_ARGS__)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #__VA_ARGS__); } } while (0)

struct DispCfg {
  using Sink = Display; using Accepts = hapi::Chain<>;
  static constexpr Consumer cls = Consumer::Display;
  static constexpr uint8_t retryMask = 0, Burst = 2;
  template<typename E> using Stack = Controller<E, Latest<>, LazyStatus>;
};
struct SdCfg {
  using Sink = Sd; using Accepts = hapi::Chain<>;
  static constexpr Consumer cls = Consumer::Storage;
  static constexpr uint8_t retryMask = KindSet<Kind::Fault, Kind::Timeout, Kind::Absent>::mask, Burst = 4;
  template<typename E> using Stack = Controller<E, Buffer<4>, TickPart<Retry<0>>, HoldOp<RejectNewest>, Backoff<100, 800>, Recover, LazyStatus>;
};
struct FinalCfg {                                        // storage, but a Fault is not a kind this edge retries: the record goes, reported
  using Sink = Sd; using Accepts = hapi::Chain<>;
  static constexpr Consumer cls = Consumer::Storage;
  static constexpr uint8_t retryMask = KindSet<Kind::Timeout>::mask, Burst = 4;
  template<typename E> using Stack = Controller<E, Buffer<4>, TickPart<Retry<3>>, HoldOp<RejectNewest>, Backoff<100, 800>, LazyStatus>;
};
struct FfCfg {
  using Sink = Ring; using Accepts = hapi::Chain<>;
  static constexpr Consumer cls = Consumer::FireForget;
  static constexpr uint8_t retryMask = 0, Burst = 8;
  template<typename E> using Stack = Controller<E, Buffer<3>, LazyStatus>;
};
struct DirectCfg {                                       // status only: the sink is called from on()
  using Sink = Sd; using Accepts = hapi::Chain<>;
  static constexpr Consumer cls = Consumer::Direct;
  static constexpr uint8_t retryMask = 0, Burst = 1;
  template<typename E> using Stack = Controller<E, DetectError, LazyStatus>;
};
struct BareCfg {                                         // no layer at all
  using Sink = Display; using Accepts = hapi::Chain<>;
  static constexpr Consumer cls = Consumer::Direct;
  static constexpr uint8_t retryMask = 0, Burst = 1;
  template<typename E> using Stack = Bare;
};

using D  = DeliveryEdge<DispCfg>;
using S  = DeliveryEdge<SdCfg>;
using F  = DeliveryEdge<FfCfg>;
using FE = DeliveryEdge<FinalCfg>;
using DR = DeliveryEdge<DirectCfg>;
using B  = DeliveryEdge<BareCfg>;

static Rec rec(int32_t v) { return Rec{1, 0, v}; }
static bool isFail(Outcome o, Kind k, uint8_t d) { return o.failed() && o.kind() == k && o.detail == d; }

#ifdef NEG_DISPLAY_RETRY
struct NegCfg { using Sink = Display; using Accepts = hapi::Chain<>; static constexpr Consumer cls = Consumer::Display; static constexpr uint8_t retryMask = 0xFF, Burst = 1;
  template<typename E> using Stack = Controller<E, Latest<>, TickPart<Retry<3>>, HoldOp<RejectNewest>, Backoff<100, 800>, LazyStatus>; };
#elif defined(NEG_DISPLAY_BUFFER)
struct NegCfg { using Sink = Display; using Accepts = hapi::Chain<>; static constexpr Consumer cls = Consumer::Display; static constexpr uint8_t retryMask = 0, Burst = 1;
  template<typename E> using Stack = Controller<E, Buffer<4>, LazyStatus>; };
#elif defined(NEG_STORAGE_OVERWRITE)
struct NegCfg { using Sink = Sd; using Accepts = hapi::Chain<>; static constexpr Consumer cls = Consumer::Storage; static constexpr uint8_t retryMask = 0, Burst = 1;
  template<typename E> using Stack = Controller<E, Latest<>, LazyStatus>; };
#elif defined(NEG_FF_RETRY)
struct NegCfg { using Sink = Ring; using Accepts = hapi::Chain<>; static constexpr Consumer cls = Consumer::FireForget; static constexpr uint8_t retryMask = 0xFF, Burst = 1;
  template<typename E> using Stack = Controller<E, Buffer<3>, TickPart<Retry<3>>, HoldOp<RejectNewest>, Backoff<100, 800>, LazyStatus>; };
#elif defined(NEG_DIRECT_STORE)
struct NegCfg { using Sink = Sd; using Accepts = hapi::Chain<>; static constexpr Consumer cls = Consumer::Direct; static constexpr uint8_t retryMask = 0, Burst = 1;
  template<typename E> using Stack = Controller<E, Buffer<3>, LazyStatus>; };
#elif defined(NEG_DIRECT_RETRY)
struct NegCfg { using Sink = Sd; using Accepts = hapi::Chain<>; static constexpr Consumer cls = Consumer::Direct; static constexpr uint8_t retryMask = 0xFF, Burst = 1;
  template<typename E> using Stack = Controller<E, TickPart<Retry<3>>, HoldOp<RejectNewest>, Backoff<100, 800>, LazyStatus>; };
#elif defined(NEG_STORE_NO_STATUS)
struct NegCfg { using Sink = Sd; using Accepts = hapi::Chain<>; static constexpr Consumer cls = Consumer::Storage; static constexpr uint8_t retryMask = 0, Burst = 1;
  template<typename E> using Stack = Controller<E, Buffer<3>>; };
#endif
#if defined(NEG_DISPLAY_RETRY) || defined(NEG_DISPLAY_BUFFER) || defined(NEG_STORAGE_OVERWRITE) || defined(NEG_FF_RETRY) || defined(NEG_DIRECT_STORE) || defined(NEG_DIRECT_RETRY) || defined(NEG_STORE_NO_STATUS)
void negUse() { DeliveryEdge<NegCfg>::tick(0); }
#endif

// ---- a display: the latest record wins, what it replaced is counted and reported -------------------------------------------------------------
static void display() {
  D::reset(); Display::reset();
  D::offer(rec(1)); D::offer(rec(2)); D::offer(rec(3));
  CHECK(D::stats().offered == 3 && D::stats().replaced == 2 && D::stats().queued == 1);
  CHECK(D::failStatus().drops == 2);                                           // counted in Status as well
  CHECK(isFail(D::_deliver(), Kind::Overflow, 2));                             // reported on the next _deliver(), once
  CHECK(D::_deliver().isPending());                                            // one record waits
  D::tick(0);
  CHECK(Display::shown.v == 3 && Display::updates == 1 && D::stats().queued == 0 && D::_deliver().isOk());   // it shows the latest
  // slower than the samples: one refresh takes 300 ms, a sample every 100
  D::reset(); Display::reset(); Display::updateMs = 300;
  for (uint32_t t = 0; t < 1200; t += 100) { D::offer(rec(int32_t(t / 100) + 1)); D::tick(t); }
  CHECK(Display::updates == 4 && D::stats().offered == 12);
  CHECK(D::stats().replaced == 12 - 4 - D::stats().queued);                    // every sample is shown or replaced or waiting: none is lost unreported
  CHECK(Display::shown.v == 10 || Display::shown.v == 11 || Display::shown.v == 12);   // and what is shown is recent
  CHECK(D::failStatus().retries == 0);
}

// ---- storage: N records, in order, the newest refused when full; a card that goes away is retried, remounted, and nothing is lost -----------------------
static void storage() {
  S::reset(); Sd::reset(); Sd::rebegin();
  Sd::busyUntil = 1000;                                                        // busy: the sink says Blocked
  for (int i = 1; i <= 6; ++i) S::offer(rec(i));
  S::tick(0); S::tick(500);
  CHECK(S::stats().refused == 2 && S::stats().queued == 4 && Sd::n == 0);     // 5 and 6 refused, counted; nothing written yet; Blocked is not a failure
  CHECK(S::failStatus().retries == 0 && S::failStatus().fails == 0);
  CHECK(isFail(S::_deliver(), Kind::Overflow, 2));                             // the refusals, reported once
  CHECK(S::_deliver().isPending());
  S::tick(1000);
  CHECK(Sd::n == 4 && Sd::log[0].v == 1 && Sd::log[1].v == 2 && Sd::log[2].v == 3 && Sd::log[3].v == 4);   // in order
  CHECK(S::stats().queued == 0 && S::_deliver().isOk());

  // the card is pulled with records waiting
  S::offer(rec(7)); S::offer(rec(8));
  Sd::pull();
  uint32_t t = 1100;
  S::tick(t);                                                                  // the write fails: Fault, held, the card is re-begun (and is not there)
  CHECK(isFail(S::_deliver(), Kind::Fault, 1));                                // the failure, while it is being retried
  S::offer(rec(9)); S::offer(rec(10)); S::offer(rec(11));                       // 3 + the two waiting = 5 > 4: one is refused
  CHECK(S::stats().refused == 3);
  const uint16_t beginsWhilePulled = Sd::begins;
  for (t += 10; t < 3000; t += 10) S::tick(t);
  CHECK(Sd::n == 4 && Sd::mounted == false);                                   // still nothing written
  CHECK(Sd::begins > beginsWhilePulled + 2 && Sd::begins < beginsWhilePulled + 12);    // re-begun on the back-off, not every tick
  CHECK(S::failStatus().retries >= 3 && S::failStatus().recovers >= 3);
  Sd::insert();
  for (; t < 6000; t += 10) S::tick(t);
  CHECK(Sd::mounted && Sd::n == 8);                                            // 7, 8, 9, 10 written after the card came back
  CHECK(Sd::log[4].v == 7 && Sd::log[5].v == 8 && Sd::log[6].v == 9 && Sd::log[7].v == 10);
  CHECK(S::stats().queued == 0 && S::stats().delivered == 8);
}

// ---- a poison record, the real production default (SdCfg, not a custom test config): the sink refuses one record's own data,
// not the card -- dropped and reported like any kind this edge does not retry, and the records after it are not stuck behind it -------
static void poisonRecord() {
  S::reset(); Sd::reset(); Sd::rebegin();
  Sd::poison = 99;
  S::offer(rec(1)); S::offer(rec(99)); S::offer(rec(2));
  S::tick(0);
  CHECK(Sd::n == 2 && Sd::log[0].v == 1 && Sd::log[1].v == 2);                  // 1 and 2 reach the card; 99 never does
  CHECK(Sd::poisonWrites == 1 && S::failStatus().retries == 0);                // refused once, never retried: Refused is not in SdCfg's own retryMask
  CHECK(S::stats().failed == 1 && S::stats().delivered == 2 && S::stats().queued == 0);
  CHECK(isFail(S::_deliver(), Kind::Refused, 0));                              // reported, once
  CHECK(!isFail(S::_deliver(), Kind::Refused, 0));
}

// ---- a failure this edge does not retry: the record is dropped for good, counted, reported once; the next one goes on ------------------------------
static void finalFailure() {
  FE::reset(); Sd::reset(); Sd::rebegin(); Sd::pull();
  FE::offer(rec(1)); FE::offer(rec(2));
  FE::tick(0);                                                                 // the card is gone: a Fault, which this edge does not retry
  CHECK(FE::stats().failed == 2 && FE::stats().queued == 0 && FE::failStatus().retries == 0);
  CHECK(isFail(FE::_deliver(), Kind::Fault, 1));                               // reported, once
  CHECK(!isFail(FE::_deliver(), Kind::Fault, 1));
  Sd::insert(); (void)Sd::rebegin();
  FE::offer(rec(3)); FE::tick(100);
  CHECK(Sd::n == 1 && Sd::log[0].v == 3 && FE::stats().delivered == 1 && FE::_deliver().isOk());
}

// ---- fire-and-forget: the newest is refused when the ring is full, counted, never retried --------------------------------------------------------
static void fireForget() {
  F::reset(); Ring::reset(); Ring::stalled = true;
  for (int i = 1; i <= 5; ++i) F::offer(rec(i));
  for (uint32_t t = 0; t < 1000; t += 10) F::tick(t);
  CHECK(F::stats().refused == 2 && F::stats().queued == 3 && Ring::n == 0);
  CHECK(F::failStatus().retries == 0 && F::failStatus().drops == 2);
  CHECK(isFail(F::_deliver(), Kind::Overflow, 2));
  Ring::stalled = false;
  F::tick(1000);
  CHECK(Ring::n == 3 && Ring::got[0].v == 1 && Ring::got[2].v == 3);          // the ones it kept, in order
}

// ---- a direct shell: status only, the sink is called from on() ---------------------------------------------------------------------------------------
static void direct() {
  DR::reset(); Sd::reset(); Sd::rebegin();
  DR::offer(rec(1));
  CHECK(Sd::n == 1 && DR::stats().delivered == 1 && DR::_deliver().isOk());
  Sd::pull();
  DR::offer(rec(2));
  CHECK(Sd::n == 1 && DR::stats().failed == 1 && DR::failStatus().fails == 1);
  CHECK(isFail(DR::_deliver(), Kind::Fault, 1));                               // the failure, on the return path (and the record is not kept: it has no store)
  // no layer at all: nothing is kept, the sink is called at once
  B::reset(); Display::reset();
  B::offer(rec(9));
  CHECK(Display::updates == 1 && Display::shown.v == 9 && B::stats().offered == 0);
  CHECK(B::_deliver().isIdle());
}

// ---- isolation: what a consumer does with its store never reaches the caller of offer() ---------------------------------------------------------------
static void isolation() {
  Cost::reset(); S::reset(); Sd::reset(); Sd::rebegin(); Sd::pull();          // a card that is gone
  Cost::fan = true;
  for (int i = 1; i <= 20; ++i) S::offer(rec(i));                              // the fan-out side
  Cost::fan = false;
  CHECK(Cost::inFan == 0);                                                     // no sink time in offer(), full store or not
  S::tick(0); S::tick(10);
  CHECK(Cost::inTick > 0 && Cost::inFan == 0);
  // a direct shell has no store to stand between them
  Cost::reset(); DR::reset();
  Cost::fan = true; DR::offer(rec(1)); Cost::fan = false;
  CHECK(Cost::inFan > 0);
}

int main() {
  display(); storage(); poisonRecord(); finalFailure(); fireForget(); direct(); isolation();
  std::printf(failures ? "FAILED (%d)\n" : "OK: failCompose F3 unit\n", failures);
  return failures != 0;
}
