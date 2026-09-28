// failCompose F5: failure meets discovery. Failure components composed over discoverCompose's registry (R3): they turn
// observed failures into row status only through World::setStatus, gate everything below a faulted bus, and bring a Stale
// row back (or retire it to Gone) by a gated re-probe. The app is discoverCompose R3's (its drivers, consumers and mock
// bus); the drivers that are polled take a Mode, and Mode chooses the failure components.
//   F5_STEP 0 bare, R3's own program and bus          -> must be discoverCompose R3's image
//           1 scaffold: reporting bus core, no failure components
//           2 + bus edge: Detect(error) + Status
//           3 + Retry (Gate, Hold): the gated bus re-probe
//           4 + Recover
//           5 + device edge: Detect + Retry (Gate, Hold) + Status
//           6 + Reprobe on the device edge            -> full
//           7 F2: the bus edge retries without end on a back-off interval and stays Stale
//           8 + the return path: a Reply on the polled rows, Retry answers Pending while it holds
//           9 + coalescing: an idempotent read arriving while its retry is held is that retry
//          10 + an overall deadline around the device retry loop        -> full F2
//          15 the device edge alone (bus edge without a controller)
//          16, 17, 18 one F2 piece alone over step 6: the return path, coalescing, an overall deadline
//   native: ./roundF5 test | scenario      AVR: main() runs the scenario and stores a checksum
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include <oneMachine/discover/binding.h>
#include <oneMachine/discover/state.h>
#include "../support/mockTwi.h"
#include "../support/mockDisplay.h"
#include "../support/mockStateful.h"
#include "../support/sensors.h"
#include "../support/display.h"
#include "../support/stateful.h"
#include <oneMachine/fail/world.h>
#include "../support/faultbus.h"
#include <oneMachine/fail/busedge.h>
#include <oneMachine/fail/devedge.h>

#ifndef F5_STEP
#define F5_STEP 6
#endif

// the most rows of each kind a discovery can produce in this app: the tables hold that many, not one per registry row
static constexpr uint8_t kBusRows  = 3;                    // the root and the mux's two channels
#ifdef F5_SMALL_K
static constexpr uint8_t kCalRows  = 1;                    // (capacity test: fewer than the topology has)
#else
static constexpr uint8_t kCalRows  = 2;
#endif
static constexpr uint8_t kSenBRows = 1;

using discover::RowId;
using discover::Sample;
using discover::Status;
using hapi::Chain;

#ifdef F5_NEG_NO_LIFECYCLE
static constexpr bool kLife = false;
#else
static constexpr bool kLife = true;
#endif

// ---- consumers: discoverCompose R3's ------------------------------------------------------------------------
struct Log3 {
  using Accepts = Chain<Temperature, Humidity>;
  struct Entry { uint8_t cap; RowId row; int16_t v; };
  inline static Entry   log[16];
  inline static uint8_t n = 0;

  template<typename Cap> struct Body {
    template<typename T> struct Part : T {
      using T::T;
      void on(const Sample<Cap>& s) { if (n < 16) log[n++] = Entry{Cap::id, s.row, int16_t(s.value)}; }
    };
  };
};

template<typename W>
struct Banner3 {
  using Iface = StateDisplay<W>;
  inline static discover::Shell<W, Iface, Banner3<W>> lcd;

  template<typename Impl> static constexpr bool wants = std::is_same<Impl, Iface>::value;
  template<typename D>    static constexpr bool served = hapi::Exists<hapi::SameAs<Iface>, D>::value;
  template<typename I> static void bind(RowId row, I* d) { lcd.bind(row, d); }
  static void unbind() { lcd.unbind(); }
  static void release(RowId r) { lcd.release(r); }

  static auto& cursor() { return decltype(lcd)::client(); }

  static bool at([[maybe_unused]] uint8_t x, [[maybe_unused]] uint8_t y) {
    Iface* d = lcd.get();
    if (!d) return false;
    if constexpr (kClient) d->setCursor(lcd.row, cursor(), x, y);
    return true;
  }
  static bool print(const char* s) {
    Iface* d = lcd.get();
    if (!d) return false;
    if constexpr (kClient) d->print(lcd.row, cursor(), s); else d->print(lcd.row, s);
    return true;
  }
  static bool light(bool on) { Iface* d = lcd.get(); if (!d) return false; d->light(lcd.row, on); return true; }
  static bool lit()          { Iface* d = lcd.get(); return d && d->lit(lcd.row); }
};

template<typename W, typename Drivers, bool Last>
struct Handle3 {
  using Handle = discover::OutHandleT<W, discover::TextOut, Drivers>;
  inline static Handle out;

  template<typename Impl> static constexpr bool wants = discover::Provides<Impl, typename discover::TextOut::Needs>::value;
  template<typename D>    static constexpr bool served = true;
  template<typename I> static void bind(RowId row, I* d) { out.bind(row, d, Last); }
  static void unbind() { out.unbind(); }
  static void release(RowId r) { out.release(r); }

  static bool print(const char* s) { return out.print(s); }
  static bool clear()              { return out.clear(); }
};

#ifdef F5_TAP
// a binding on a failure-aware driver, to see a row's Gone reach it (native test only)
template<typename W, typename Iface>
struct Tap5 {
  inline static discover::Shell<W, Iface, Tap5<W, Iface>> sh;
  template<typename Impl> static constexpr bool wants = std::is_same<Impl, Iface>::value;
  template<typename D>    static constexpr bool served = hapi::Exists<hapi::SameAs<Iface>, D>::value;
  template<typename I> static void bind(RowId row, I* d) { sh.bind(row, d); }
  static void unbind() { sh.unbind(); }
  static void release(RowId r) { sh.release(r); }
};
#endif

// ---- the bus, and what is composed --------------------------------------------------------------------------
// the reporting core over R3's mock bus: the displays answer too, and sit on the root bus
struct Disp {
  static bool selected()        { return mock3::Display::sel || mockdisp::Screen2::sel; }
  static bool onRoot(uint8_t a) { return (mock3::Display::present && a == mock3::Display::addr) || (mockdisp::Screen2::present && a == mockdisp::Screen2::addr); }
};
using Twi3 = hapi::APIOf<oneBus::TwiAPI, oneBus::I2cMaster<100000>, fbus::CoreT<Disp>, mock3::Core, mock::TwiCore>;
static_assert(oneBus::is_twi_master<Twi3>::value, "Twi3 must satisfy TwiMaster");

struct DevBare { template<typename E> using Stack = fail::Bare; };
template<typename... L> struct DevCtl { template<typename E> using Stack = fail::Controller<E, L...>; };
using fail::BusBare;
using fail::BusCtl;

template<typename TwiT, bool Checked, typename BusM, typename DevM, bool ReturnPath = false, bool Idempotent = false>
struct ModeT {
  using Twi = TwiT;
  static constexpr bool checked    = Checked;       // the drivers observe the bus's verdicts
  static constexpr bool returnPath = ReturnPath;    // the layers answer _f (Reply, Overall, Retry's Pending)
  static constexpr bool idempotent = Idempotent;    // the polled reads may be coalesced into a held retry
  static constexpr bool lifecycle  = kLife;
  template<typename E> using BusStack = typename BusM::template Stack<E>;
  template<typename E> using DevStack = typename DevM::template Stack<E>;
};

using fail::RejectNewest;
#define BUS_RETRY fail::TickPart<fail::Retry<4>>
#define DEV_RETRY fail::TickPart<fail::Retry<2>>
#if   F5_STEP == 0
using Mode = ModeT<mock3::Twi, false, BusBare, DevBare>;
#elif F5_STEP == 1
using Mode = ModeT<Twi3, false, BusBare, DevBare>;
#elif F5_STEP == 2
using Mode = ModeT<Twi3, true, BusCtl<fail::DetectError, fail::Status>, DevBare>;
#elif F5_STEP == 3
using Mode = ModeT<Twi3, true, BusCtl<BUS_RETRY, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Gate<100>, fail::Status>, DevBare>;
#elif F5_STEP == 4
using Mode = ModeT<Twi3, true, BusCtl<BUS_RETRY, fail::Recover, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Gate<100>, fail::Status>, DevBare>;
#elif F5_STEP == 5
using Mode = ModeT<Twi3, true, BusCtl<BUS_RETRY, fail::Recover, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Gate<100>, fail::Status>,
                   DevCtl<DEV_RETRY, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Gate<50>, fail::LazyStatus>>;
#elif F5_STEP >= 7 && F5_STEP <= 10
// F2. The bus edge retries without end on a back-off interval (it stays Stale); the polled rows add, one step at a time, the return path,
// coalescing of idempotent reads, and an overall deadline around the retry loop.
#ifndef F5_OVERALL_MS
#define F5_OVERALL_MS 600
#endif
#define BUS_BACKOFF_RETRY fail::TickPart<fail::Retry<0>>
#if   F5_STEP == 7
using Mode = ModeT<Twi3, true, BusCtl<BUS_BACKOFF_RETRY, fail::Recover, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Backoff<100, 400>, fail::Status>,
                   DevCtl<DEV_RETRY, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Gate<50>, fail::TickPart<fail::Reprobe<200, 3>>, fail::LazyStatus>>;
#elif F5_STEP == 8
using Mode = ModeT<Twi3, true, BusCtl<BUS_BACKOFF_RETRY, fail::Recover, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Backoff<100, 400>, fail::Status>,
                   DevCtl<fail::CausePart<fail::Reply<false>>, DEV_RETRY, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Gate<50>, fail::TickPart<fail::Reprobe<200, 3>>, fail::LazyStatus>, true, false>;
#elif F5_STEP == 9
using Mode = ModeT<Twi3, true, BusCtl<BUS_BACKOFF_RETRY, fail::Recover, fail::DetectError, fail::HoldOp<fail::Coalesce>, fail::Backoff<100, 400>, fail::Status>,
                   DevCtl<fail::CausePart<fail::Reply<false>>, DEV_RETRY, fail::DetectError, fail::HoldOp<fail::Coalesce>, fail::Gate<50>, fail::TickPart<fail::Reprobe<200, 3>>, fail::LazyStatus>, true, true>;
#else
using Mode = ModeT<Twi3, true, BusCtl<BUS_BACKOFF_RETRY, fail::Recover, fail::DetectError, fail::HoldOp<fail::Coalesce>, fail::Backoff<100, 400>, fail::Status>,
                   DevCtl<fail::CausePart<fail::Reply<false>>, fail::TickPart<fail::CausePart<fail::Overall<F5_OVERALL_MS>>>, DEV_RETRY, fail::DetectError, fail::HoldOp<fail::Coalesce>, fail::Gate<50>,
                          fail::TickPart<fail::Reprobe<200, 3>>, fail::LazyStatus>, true, true>;
#endif
#elif F5_STEP == 16
// F2 alone, over step 6: the return path (a Reply on the polled rows)
using Mode = ModeT<Twi3, true, BusCtl<BUS_RETRY, fail::Recover, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Gate<100>, fail::Status>,
                   DevCtl<fail::CausePart<fail::Reply<false>>, DEV_RETRY, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Gate<50>, fail::TickPart<fail::Reprobe<200, 3>>, fail::LazyStatus>, true, false>;
#elif F5_STEP == 17
// F2 alone, over step 6: coalescing of the polled reads
using Mode = ModeT<Twi3, true, BusCtl<BUS_RETRY, fail::Recover, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Gate<100>, fail::Status>,
                   DevCtl<DEV_RETRY, fail::DetectError, fail::HoldOp<fail::Coalesce>, fail::Gate<50>, fail::TickPart<fail::Reprobe<200, 3>>, fail::LazyStatus>, false, true>;
#elif F5_STEP == 18
// F2 alone, over step 6: an overall deadline around the device retry loop
using Mode = ModeT<Twi3, true, BusCtl<BUS_RETRY, fail::Recover, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Gate<100>, fail::Status>,
                   DevCtl<fail::TickPart<fail::CausePart<fail::Overall<600>>>, DEV_RETRY, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Gate<50>, fail::TickPart<fail::Reprobe<200, 3>>, fail::LazyStatus>, true, false>;
#elif F5_STEP == 15
// the device edge alone: the bus edge has no controller
using Mode = ModeT<Twi3, true, BusBare,
                   DevCtl<DEV_RETRY, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Gate<50>, fail::TickPart<fail::Reprobe<200, 3>>, fail::LazyStatus>>;
#elif defined(F5_NEG_REPROBE_ABOVE_RETRY)
using Mode = ModeT<Twi3, true, BusCtl<BUS_RETRY, fail::Recover, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Gate<100>, fail::Status>,
                   DevCtl<fail::TickPart<fail::Reprobe<200, 3>>, DEV_RETRY, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Gate<50>, fail::LazyStatus>>;
#else
using Mode = ModeT<Twi3, true, BusCtl<BUS_RETRY, fail::Recover, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Gate<100>, fail::Status>,
                   DevCtl<DEV_RETRY, fail::DetectError, fail::HoldOp<RejectNewest>, fail::Gate<50>, fail::TickPart<fail::Reprobe<200, 3>>, fail::LazyStatus>>;
#endif

// ---- drivers that are polled: R3's CalSensor and SensorB with a device edge ----------------------------------------
template<typename W, typename M, uint8_t K>
struct FCal : discover::DriverBase<FCal<W, M, K>, W>, std::conditional_t<kDev, CalDev, NoCal>, fail::DevEdge<FCal<W, M, K>, W, M, K> {
  using B    = discover::DriverBase<FCal<W, M, K>, W>;
  using Edge = fail::DevEdge<FCal<W, M, K>, W, M, K>;
  using Produces = Chain<Temperature>;
  static constexpr uint8_t addrLo = 0x48, addrHi = 0x48, id = 0xA1;
  inline static uint8_t inits = 0;
#ifdef F5_RECHECK
  static constexpr bool reinitOnBusReturn = true;          // its init is safe to run again: the bus edge does it when the bus comes back
#endif

  static void init(RowId row) {
    if constexpr (kDev) {
      uint8_t c = 0;
      B::readRegs(B::addrOf(row), 3, &c, 1);
      B::dev(row).offset = int8_t(c);
    }
    ++inits;
  }
  static void reinit(RowId row) { W::clearState(row); init(row); }

  // no failure component: R3's own read, verbatim
  static void read(RowId row) {
    if constexpr (M::checked) Edge::serve(row, fail::Cause::Fresh);
    else {
      uint8_t b[2];
      B::readRegs(B::addrOf(row), 1, b, 2);
      int16_t t = int16_t(uint16_t((uint16_t(b[0]) << 8) | b[1]));
      if constexpr (kDev) t = int16_t(t + B::dev(row).offset);
      B::template emit<Temperature>(row, t);
    }
  }

  static fail::Outcome attempt(RowId row) {
    uint8_t b[2] = {0, 0};
    const fail::Outcome o = Edge::checkedRead(row, 1, b, 2);
    if (!o.isOk()) return o;
    int16_t t = int16_t(uint16_t((uint16_t(b[0]) << 8) | b[1]));
    if constexpr (kDev) t = int16_t(t + B::dev(row).offset);
    B::template emit<Temperature>(row, t);
    return o;
  }
};

template<typename W, typename M, uint8_t K>
struct FSensorB : discover::DriverBase<FSensorB<W, M, K>, W>, fail::DevEdge<FSensorB<W, M, K>, W, M, K> {
  using B    = discover::DriverBase<FSensorB<W, M, K>, W>;
  using Edge = fail::DevEdge<FSensorB<W, M, K>, W, M, K>;
  using Produces = Chain<Temperature, Humidity>;
  static constexpr uint8_t addrLo = 0x40, addrHi = 0x40, id = 0xB2;
#ifdef F5_PRESENCE
  static constexpr bool presenceOnly = true;                // no ID register: the reprobe takes an ACK for the identity
#endif
#ifdef F5_RECHECK
  inline static uint8_t rechecks = 0;
  static void recheck(RowId) { ++rechecks; }                // its own check, counted
#endif

  static void reinit(RowId) {}

  // no failure component: R3's own read, verbatim
  static void read(RowId row) {
    if constexpr (M::checked) Edge::serve(row, fail::Cause::Fresh);
    else {
      uint8_t b[3];
      B::readRegs(B::addrOf(row), 1, b, 3);
      B::template emit<Temperature>(row, int16_t(uint16_t((uint16_t(b[0]) << 8) | b[1])));
      B::template emit<Humidity>(row, b[2]);
    }
  }

  static fail::Outcome attempt(RowId row) {
    uint8_t b[3] = {0, 0, 0};
    const fail::Outcome o = Edge::checkedRead(row, 1, b, 3);
    if (!o.isOk()) return o;
    B::template emit<Temperature>(row, int16_t(uint16_t((uint16_t(b[0]) << 8) | b[1])));
    B::template emit<Humidity>(row, b[2]);
    return o;
  }
};

// ---- application: R3's, with the bus edge -----------------------------------------------------------------------
template<uint8_t N> struct AppN;
template<uint8_t N> using DriversF = Chain<FCal<AppN<N>, Mode, kCalRows>, FSensorB<AppN<N>, Mode, kSenBRows>, Mux<AppN<N>>, StateDisplay<AppN<N>>, LineDisplay<AppN<N>>>;
template<uint8_t N> using BindersF = Chain<Banner3<AppN<N>>, Handle3<AppN<N>, DriversF<N>, false>, Handle3<AppN<N>, DriversF<N>, true>
#ifdef F5_TAP
  , Tap5<AppN<N>, FCal<AppN<N>, Mode, kCalRows>>
#endif
>;

// the bus edge exists once the drivers observe the bus (Mode::checked); before that there is nothing for it to do
struct NoBusEdge {};
template<uint8_t N> using BusBase = std::conditional_t<Mode::checked, fail::BusEdge<AppN<N>, DriversF<N>, kBusRows, Mode>, NoBusEdge>;

template<uint8_t N>
struct AppN : discover::World<AppN<N>, typename Mode::Twi, Chain<Log3>, DriversF<N>, N, discover::I2cScan>, BusBase<N> {
  using WorldB = discover::World<AppN<N>, typename Mode::Twi, Chain<Log3>, DriversF<N>, N, discover::I2cScan>;
  static constexpr bool lifecycle = Mode::lifecycle;

  template<typename Impl> static void bind(RowId row) { discover::BinderSet<BindersF<N>>::template bind<Impl>(row); }
  static void release(RowId row)                       { discover::BinderSet<BindersF<N>>::release(row); }
  static void unbindAll()                              { discover::BinderSet<BindersF<N>>::unbind(); }

  // the bus's recovery is a reset of the master's state, counted; the mock has nothing else to reset
  static void busReset() { ++fbus::State::resets; }

#ifdef F5_COUNT
  inline static uint16_t statusWrites = 0, discovers = 0;       // every status change comes through setStatus
  static void setStatus(RowId r, Status s) { ++statusWrites; WorldB::setStatus(r, s); }
#endif
#if F5_STEP >= 2
  // the controllers' state starts over with the table: a row index is a new row after a discovery
  static void resetFail() {
    fail::BusEdge<AppN<N>, DriversF<N>, kBusRows, Mode>::resetCtl();
    FCal<AppN<N>, Mode, kCalRows>::resetFail(); FSensorB<AppN<N>, Mode, kSenBRows>::resetFail();
  }
  static void discover() {
#ifdef F5_COUNT
    ++discovers;
#endif
    resetFail(); WorldB::discover();
  }
#endif
};

using App  = AppN<10>;
using Cal  = FCal<App, Mode, kCalRows>;
using SenB = FSensorB<App, Mode, kSenBRows>;
using Ban  = Banner3<App>;
using Rd   = Handle3<App, DriversF<10>, false>;
using Mr   = Handle3<App, DriversF<10>, true>;
using SD   = StateDisplay<App>;
using D3   = mock3::Display;
using S2   = mockdisp::Screen2;

static_assert(discover::BinderSet<BindersF<10>>::served<DriversF<10>>, "a binding consumer names a driver that is not in the driver list");
static_assert(discover::DriverSet<DriversF<10>>::distinct, "driver list has a repeated type");
// zero cost where nothing is declared: drivers that declare no state need no slot
template<typename X> using R2Drivers = Chain<SensorA<X>, SensorB<X>, Mux<X>, TextDisplay<X>, LineDisplay<X>>;
struct Incomplete;
static_assert(__is_empty(discover::DevSlotOf<R2Drivers<Incomplete>>) && __is_empty(discover::CliSlotOf<R2Drivers<Incomplete>>),
              "drivers that declare no state need no slot");
static_assert(uint8_t(fail::RowState::Alive) == uint8_t(Status::Alive) && uint8_t(fail::RowState::Stale) == uint8_t(Status::Stale) &&
              uint8_t(fail::RowState::Gone) == uint8_t(Status::Gone), "the edges write RowState into the registry row as discover::Status");
#ifdef __AVR__
static_assert(sizeof(App::reg.rows[0]) == 5, "discoverCompose's row is untouched: busId + ptr + parent + flags");
#endif

// ---- everything a run leaves behind, folded into 16 bits: identical arithmetic on native and AVR (discoverCompose R3's) ----
template<typename T> static uint16_t hashBytes(uint16_t h, const T& t) {
  auto* p = reinterpret_cast<const uint8_t*>(&t);
  for (unsigned i = 0; i < sizeof(T); ++i) h = uint16_t(h * 31u + p[i]);
  return h;
}
template<bool Has, typename Tab = App::Dev<>::Table> static uint16_t hashSlot(uint16_t h, RowId r) {
  if constexpr (Has) return hashBytes(h, Tab::rows[r]);
  else { (void)r; return h; }
}
template<bool Has, typename B = Ban> static uint16_t hashCursor(uint16_t h) {
  if constexpr (Has) return hashBytes(h, B::cursor());
  else return h;
}
static uint16_t checksum() {
  uint16_t h = 0;
  for (RowId r = 0; r < App::reg.count; ++r) {
    const auto& row = App::reg.rows[r];
    h = uint16_t(h * 31u + row.busId + row.parent * 7u + row.isBus + (unsigned(row.status()) << 3));
    h = hashSlot<kDev>(h, r);
  }
  for (auto& r : D3::cell) for (char c : r) h = uint16_t(h * 31u + uint8_t(c));
  for (uint8_t i = 0; i < D3::logN; ++i) h = uint16_t(h * 31u + D3::log[i]);
  for (auto& r : S2::cell) for (char c : r) h = uint16_t(h * 31u + uint8_t(c));
  for (uint8_t i = 0; i < S2::logN; ++i) h = uint16_t(h * 31u + S2::log[i]);
  for (uint8_t i = 0; i < Log3::n; ++i) h = uint16_t(h * 31u + Log3::log[i].cap + Log3::log[i].row + uint16_t(Log3::log[i].v));
  h = uint16_t(h * 31u + Cal::inits + SD::inits + D3::backlight);
  return hashCursor<kClient>(h);
}

// R3's scenario, unchanged: no faults, the row lifecycle from the consumer side
[[maybe_unused]] static void scenario() {
  Cal::inits = 0; SD::inits = 0;
  D3::present = true; D3::addr = 0x27; D3::reset(); S2::reset();
  mock::Bus::mask = 2;
  mock::Bus::poke(0, 0x48, 3, 3);
  mock::Bus::poke(1, 0x48, 3, uint8_t(-2));
  App::discover();
  if constexpr (kClient) { Ban::at(0, 1); Ban::print("AB"); }
  Rd::print("CD");
  Ban::print("EF");
  Ban::light(false);
  Rd::print("GH");
  Log3::n = 0;
  App::pump();
  if constexpr (kLife) { App::setStatus(4, Status::Gone); App::pump(); }
  D3::addr = 0x41; D3::present = true; D3::powerCycle();
  App::discover();
  Ban::print("IJ");
  App::pump();
}

#if F5_STEP >= 1
using Ticker = fail::Ticks<App, DriversF<10>>;

// the tick: the bus controllers first (the root, then each channel while the bus above is up), then the device controllers
ONEMACHINE_NOINLINE static void tickAll(uint32_t now) {
#if F5_STEP >= 2
  App::tickBuses(now);
#endif
  Ticker::run(now);
}

// The fault script, in mock time (one tick per ms, a poll every 10th):
//   200   the sensor on channel 1 stops answering         -> retries, Stale, re-probes
//   900   it answers again, calibrated differently        -> Alive, its init runs again
//   1200  channel 1's bus sticks                          -> that channel Stale, the rest goes on
//   1450  it frees                                        -> the channel's re-probe brings it back
//   1800  the root bus sticks                             -> everything below Stale, one probe per interval
//   2150  it frees                                        -> the whole tree Alive again, nothing rediscovered
//   2400  the sensor on channel 0 is gone for good        -> retries, Stale, re-probes, Gone
[[maybe_unused]] static void faultScript() {
  App::discover();
  uint8_t every = 0;
  for (uint16_t t = 0; t < 3400; ++t) {
    switch (t) {
      case 200:  mock::Bus::devs[3].addr = 0x4F; break;
      case 900:  mock::Bus::devs[3].addr = 0x48; mock::Bus::devs[3].regs[3] = 5; break;
      case 1200: fbus::State::stuckCh[1] = true; break;
      case 1450: fbus::State::stuckCh[1] = false; break;
      case 1800: fbus::State::stuckRoot = true; break;
      case 2150: fbus::State::stuckRoot = false; break;
      case 2400: mock::Bus::devs[2].addr = 0x4E; break;
      default: break;
    }
    if (every == 0) App::pump();
    if (++every == 10) every = 0;
    tickAll(t);
  }
}

#if F5_STEP == 8 || F5_STEP == 9 || F5_STEP == 10 || F5_STEP == 16
// the return path's consumer: what each polled row's last operation came to (out of line, so its calls can be walked)
ONEMACHINE_NOINLINE static uint16_t foldServe(uint16_t h) {
  for (RowId r = 6; r <= 8; ++r) {
    const fail::Outcome o = r == 6 ? SenB::_serve(r) : Cal::_serve(r);
    h = uint16_t(h * 31u + o.raw()); h = uint16_t(h * 31u + o.detail);
  }
  return h;
}
#endif

[[maybe_unused]] static uint16_t failChecksum() {
  uint16_t h = checksum();
  for (RowId r = 0; r < App::reg.count; ++r) h = uint16_t(h * 31u + unsigned(App::reg.rows[r].status()));
  auto fold = [&h](const fail::FailStatus& s) {
    h = uint16_t(h * 31u + s.retries); h = uint16_t(h * 31u + s.recovers); h = uint16_t(h * 31u + s.drops);
    h = uint16_t(h * 31u + s.fails);   h = uint16_t(h * 31u + s.lastKind); h = uint16_t(h * 31u + s.lastDetail);
  };
#if F5_STEP >= 2
  for (RowId r = 0; r < 4; ++r) fold(App::busStatus(r));
#endif
  for (RowId r = 6; r <= 8; ++r) fold(r == 6 ? SenB::failStatus(r) : Cal::failStatus(r));
  h = uint16_t(h * 31u + uint16_t(fbus::State::starts) + uint16_t(fbus::State::timeouts));
  h = uint16_t(h * 31u + uint16_t(fbus::State::busyMs));
#if F5_STEP == 8 || F5_STEP == 9 || F5_STEP == 10 || F5_STEP == 16
  h = foldServe(h);
#endif
  return uint16_t(h * 31u + fbus::State::resets);
}
#endif

// ---- AVR entry ------------------------------------------------------------------------------------------------------
#ifdef __AVR__
extern "C" void __cxa_pure_virtual() { for (;;) {} }
volatile uint16_t g_sum;
__attribute__((noinline)) void done() { for (;;) {} }
int main() {
#if F5_STEP == 0
  scenario();
  g_sum = checksum();
#else
  faultScript();
  g_sum = failChecksum();
#endif
  done();
}
#elif defined(F5_NO_MAIN)
#elif defined(F5_PARITY)
#include <cstdio>
int main() {
#if F5_STEP == 0
  scenario();
  std::printf("checksum 0x%04X\n", checksum());
#else
  faultScript();
  std::printf("checksum 0x%04X\n", failChecksum());
#endif
  return 0;
}
#else
#include <cstdio>
#include <cstring>
#include <string>

static int failures = 0;
#define CHECK(...) do { if (!(__VA_ARGS__)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #__VA_ARGS__); } } while (0)

#if F5_STEP != 6 && F5_STEP != 10
int main(int argc, char** argv) {
  scenario();
  const uint16_t c = checksum();
  std::printf("checksum 0x%04X (discoverCompose R3: 0x4910)\n", c);
  CHECK(c == 0x4910);
  (void)argc; (void)argv;
  std::printf(failures ? "FAILED (%d)\n" : "OK: failCompose F5 native, F5_STEP=%d, no faults = R3\n", failures ? failures : F5_STEP);
  return failures != 0;
}
#else
#ifndef F5_COUNT
#error "the native test counts status writes: build with -DF5_COUNT -DF5_TAP"
#endif
#ifdef F5_SMALL_K
#pragma GCC diagnostic ignored "-Wunused-function"
#endif
#ifdef F5_RECHECK
static constexpr uint8_t kR = 1;                 // the drivers declare bus-return hooks: FCal reinitOnBusReturn, FSensorB recheck
#else
static constexpr uint8_t kR = 0;
#endif
static constexpr bool kF2 = F5_STEP == 10;      // the full F2 composition: back-off on the bus, the return path, coalescing
using discover::instOf;
using fail::Kind;
static auto& reg = App::reg;
using Slot = App::Dev<>::Slot;
using DTab = App::Dev<>::Table;

static mock::Dev pristine[mock::Bus::ndevs];

// ---- a run: mock time, one tick per ms, a poll every 10th; what the data path delivered; who wrote status ----------------
static uint32_t T = 0;
static uint8_t  every = 0;
static bool     pumpOn = true;
struct Seen { uint16_t n[10]; int16_t lo[10], hi[10]; };
static Seen seen;
static uint32_t probeAt[64]; static uint8_t probes = 0;       // the times a tick reached the bus
static uint32_t startsInPump = 0;                              // bus starts made by the data path
static uint16_t unexplained = 0;                               // a status changed and setStatus was not called

static void drain() {
  for (uint8_t i = 0; i < Log3::n; ++i)
    if (Log3::log[i].cap == Temperature::id) {
      const RowId r = Log3::log[i].row; const int16_t v = Log3::log[i].v;
      if (seen.n[r]++ == 0) seen.lo[r] = seen.hi[r] = v; else { if (v < seen.lo[r]) seen.lo[r] = v; if (v > seen.hi[r]) seen.hi[r] = v; }
    }
  Log3::n = 0;
}
static void clearSeen() {
  for (uint8_t i = 0; i < 10; ++i) { seen.n[i] = 0; seen.lo[i] = seen.hi[i] = 0; }
  probes = 0; startsInPump = 0;
}

static void snapshotStatus(uint8_t* out) { for (RowId r = 0; r < 10; ++r) out[r] = r < reg.count ? uint8_t(reg.rows[r].status()) : 0xFF; }

static void step() {
  uint8_t before[10], after[10]; snapshotStatus(before);
  const uint16_t w0 = App::statusWrites;
  const uint32_t s0 = fbus::State::starts;
  if (every == 0 && pumpOn) App::pump();
  if (++every == 10) every = 0;
  const uint32_t s1 = fbus::State::starts;
  tickAll(T);
  if (fbus::State::starts != s1 && probes < 64) probeAt[probes++] = T;
  startsInPump += s1 - s0;
  drain();
  snapshotStatus(after);
  if (std::memcmp(before, after, 10) != 0 && App::statusWrites == w0) ++unexplained;
  ++T;
}
static void run(uint32_t until) { while (T < until) step(); }

static void fresh() {
  fbus::State::reset();
  for (uint8_t i = 0; i < mock::Bus::ndevs; ++i) mock::Bus::devs[i] = pristine[i];
  mock::Bus::mask = 0; mock::Bus::contention = 0; mock::Bus::state = mock::Bus::Idle; mock::Bus::sel = nullptr;
  D3::present = true; D3::addr = 0x27; D3::reset(); S2::present = true; S2::reset();
  Cal::inits = 0; SD::inits = 0; Log3::n = 0;
#ifdef F5_RECHECK
  SenB::rechecks = 0;
#endif
  mock::Bus::poke(0, 0x48, 3, 3);
  mock::Bus::poke(1, 0x48, 3, uint8_t(-2));
  App::statusWrites = 0; App::discovers = 0;
  App::discover();
  fbus::State::clearCounters();
  clearSeen();
  T = 0; every = 0; pumpOn = true; unexplained = 0;
}

static bool slotIsZero(RowId r) {
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&DTab::rows[r]);
  for (unsigned i = 0; i < sizeof(Slot); ++i) if (p[i]) return false;
  return true;
}
static uint8_t recheckCount() {
#ifdef F5_RECHECK
  return SenB::rechecks;
#else
  return 0;
#endif
}
static Status st(RowId r) { return reg.status(r); }
static fail::FailStatus dev(RowId r) { return r == 6 ? SenB::failStatus(r) : Cal::failStatus(r); }
static bool allAlive() { for (RowId r = 0; r < reg.count; ++r) if (st(r) != Status::Alive) return false; return true; }
static bool devicesClean() { for (RowId r = 6; r <= 8; ++r) { const auto s = dev(r); if (s.fails || s.retries || s.recovers || s.drops) return false; } return true; }
static bool noOverflow() { return Cal::overflows() == 0 && SenB::overflows() == 0 && App::overflows() == 0; }   // every row of every kind had a slot
static bool spaced(uint32_t gap) { for (uint8_t i = 1; i < probes; ++i) if (probeAt[i] - probeAt[i - 1] < gap) return false; return true; }

static void topology() {
  struct E { uint8_t busId; RowId parent; bool isBus; };
  const E want[] = {{0x00, discover::noRow, true}, {0x70, 0, false}, {0x00, 1, true}, {0x01, 1, true}, {0x27, 0, false},
                    {0x3F, 0, false}, {0x40, 0, false}, {0x48, 2, false}, {0x48, 3, false}};
  CHECK(reg.count == 9 && reg.overflow == 0);
  for (RowId r = 0; r < reg.count && r < 9; ++r) CHECK(reg.rows[r].busId == want[r].busId && reg.rows[r].parent == want[r].parent && bool(reg.rows[r].isBus) == want[r].isBus);
  CHECK(reg.rows[7].drv == instOf<Cal>() && reg.rows[8].drv == instOf<Cal>() && reg.rows[6].drv == instOf<SenB>());
  CHECK(Tap5<App, Cal>::sh.row == 8);
}

// ---- 0. no failure: nothing is counted, nothing is written, the data path is R3's --------------------------------------
static void healthy() {
  fresh(); topology();
  run(300);
  CHECK(allAlive() && App::statusWrites == 0 && unexplained == 0);
  CHECK(App::busStatus(0).fails == 0 && App::busStatus(2).fails == 0 && App::busStatus(3).fails == 0 && devicesClean());
  CHECK(seen.n[6] == 30 && seen.n[7] == 30 && seen.n[8] == 30);
  CHECK(seen.lo[6] == 187 && seen.hi[6] == 187 && seen.lo[7] == 218 && seen.hi[7] == 218 && seen.lo[8] == 251 && seen.hi[8] == 251);   // per row: its own reading and calibration
  CHECK(fbus::State::timeouts == 0 && fbus::State::probeTxns == 0 && fbus::State::resets == 0);
  CHECK(noOverflow());
  scenario();                                                   // R3's own scenario, on the reporting core
  CHECK(checksum() == 0x4910);
}

// ---- 1a. a transient failure on a device: NACK twice (the poll, the first re-issue), then it answers: retries within the budget, no status written ----
static void transientDevice() {
  fresh();
  run(200); mock::Bus::devs[3].addr = 0x4F;                      // the poll at 200 fails
  run(280); mock::Bus::devs[3].addr = 0x48;                      // the re-issue at 250 fails, the one at 300 succeeds
  run(400);
  const auto d = dev(8);
  CHECK(allAlive() && App::statusWrites == 0 && unexplained == 0);
  CHECK(d.fails == 2 && d.retries == 2 && d.recovers == 0 && d.lastKind == uint8_t(Kind::Absent) && d.drops == (kF2 ? 0 : 10));   // the polls at 210..300 while the operation is held: refused and counted (F5b), coalesced (F2)
  CHECK(fbus::State::resets == 0 && App::busStatus(3).fails == 0 && App::busStatus(0).fails == 0);
  CHECK(Cal::inits == 2 && App::devState<Cal>(8).offset == -2);  // it never left Alive: nothing initialised again, its state kept
  CHECK(seen.n[8] == 30 && seen.lo[8] == 251 && seen.hi[8] == 251 && seen.n[7] == 40 && seen.n[6] == 40);   // the late sample at 300 is delivered
}

// ---- 1. a transient bus timeout seen on a device operation: the bus owns it, nothing is counted against a device ------------
static void transient() {
  fresh();
  run(105); fbus::State::stuckRoot = true;
  run(150); fbus::State::stuckRoot = false;
  run(400);
  const auto b = App::busStatus(0);
  CHECK(allAlive() && reg.count == 9 && App::discovers == 1);
  CHECK(b.fails == 1 && b.retries == 1 && b.recovers == 1 && b.drops == 0 && b.lastKind == uint8_t(Kind::Timeout) && b.lastDetail == uint8_t(oneBus::TwiCause::Timeout));
  CHECK(fbus::State::resets == 1);                              // Recover: once, for the Timeout
  CHECK(devicesClean() && App::busStatus(2).fails == 0 && App::busStatus(3).fails == 0);
  CHECK(App::statusWrites == 2 && unexplained == 0);            // Stale, then Alive, both through setStatus
  CHECK(Cal::inits == 2 + 2 * kR && SD::inits == 1 && recheckCount() == kR);   // a bus that came back has not touched its devices, except to ask the ones that declared it: none else initialised again
  CHECK(App::devState<Cal>(7).offset == 3 && App::devState<Cal>(8).offset == -2);
  CHECK(seen.n[6] == 29 && seen.n[7] == 29 && seen.n[8] == 29); // 40 polls, none from 110 to 210 (the bus is Stale); the failed one made no sample
  CHECK(seen.lo[6] == 187 && seen.hi[6] == 187 && seen.lo[7] == 218 && seen.hi[7] == 218 && seen.lo[8] == 251 && seen.hi[8] == 251);
  CHECK(probes == 1 && probeAt[0] == 210);
}

// ---- 2. a device NACKs: Absent, retried, Stale, re-probed, Alive, and its init runs again --------------------------------------
static void nackAndReturn() {
  fresh();
  run(200); mock::Bus::devs[3].addr = 0x4F;
  run(299);
  CHECK(st(8) == Status::Alive);                                // the retries are still running
  run(301);
  CHECK(st(8) == Status::Stale && st(3) == Status::Alive && st(7) == Status::Alive && st(6) == Status::Alive);   // the device, not its bus
  const auto d = dev(8);
  CHECK(d.fails == 3 && d.retries == 2 && d.lastKind == uint8_t(Kind::Absent) && d.lastDetail == uint8_t(oneBus::TwiCause::Nack) && d.recovers == 0);
  CHECK(fbus::State::resets == 0 && App::busStatus(3).fails == 0 && App::busStatus(0).fails == 0);   // Recover never runs on Absent; the bus is not blamed
  CHECK(!Tap5<App, Cal>::sh.get() && Tap5<App, Cal>::sh.row == 8);                                 // Stale: refused, but still bound
  const uint16_t n8 = seen.n[8], n7 = seen.n[7];
  run(600);
  CHECK(seen.n[8] == n8 && seen.n[7] == n7 + 29 && st(8) == Status::Stale);                     // no samples from a Stale row; the sibling goes on
  mock::Bus::devs[3].addr = 0x48; mock::Bus::devs[3].regs[3] = 5;                                // it returns, calibrated differently
  run(1000);
  CHECK(st(8) == Status::Alive && allAlive());
  CHECK(Cal::inits == 3 && SD::inits == 1);                                                        // the init decision: a device that came back on its own is initialised again
  CHECK(App::devState<Cal>(8).offset == 5 && App::devState<Cal>(7).offset == 3);
  CHECK(seen.lo[8] == 251 && seen.hi[8] == 258);                                                   // -2 before, +5 after
  CHECK(Tap5<App, Cal>::sh.get() != nullptr);
  CHECK(App::statusWrites == 2 && unexplained == 0 && App::discovers == 1 && reg.count == 9);
  CHECK(fbus::State::resets == 0 && noOverflow());
}

// ---- 2b. a device returns and answers, but its register 0 is not its id: somebody else there, unless the device has no ID register ----------------
static void reprobeIdentity() {
  fresh();
  run(200); mock::Bus::devs[1].addr = 0x4F;                      // SensorB (row 6) NACKs from the poll at 200
  run(600);
  CHECK(st(6) == Status::Stale && st(8) == Status::Alive);
  mock::Bus::devs[1].addr = 0x40; mock::Bus::devs[1].regs[0] = 0x11;   // it answers again; register 0 reads 0x11, its id is 0xB2
  run(kF2 ? 12000 : 1300);
#ifdef F5_PRESENCE
  CHECK(st(6) == Status::Alive && allAlive());                   // an ACK is its identity
#else
  CHECK(st(6) == Status::Gone && st(7) == Status::Alive);        // somebody else answers there: every re-probe refused, then given up
#endif
}

// ---- 3. gone for good: Gone, bindings released, final until discover() -------------------------------------------------------
static void goneForGood() {
  fresh();
  run(200); mock::Bus::devs[3].addr = 0x4F;
  run(900);
  CHECK(st(8) == Status::Stale);
  run(902);
  CHECK(st(8) == Status::Gone && st(7) == Status::Alive && st(3) == Status::Alive);
  CHECK(Tap5<App, Cal>::sh.row == discover::noRow && Tap5<App, Cal>::sh.get() == nullptr);      // the binding is released
  CHECK(slotIsZero(8) && !slotIsZero(7));                                                        // its state is cleared
  CHECK(App::statusWrites == 2);                                                                  // Stale, Gone
  mock::Bus::devs[3].addr = 0x48;                                                                 // back on the bus
  const uint32_t pt = fbus::State::probeTxns;
  run(1500);
  CHECK(st(8) == Status::Gone && fbus::State::probeTxns == pt && App::statusWrites == 2);         // final: no re-probing
  App::discover();
  CHECK(allAlive() && Tap5<App, Cal>::sh.row == 8 && dev(8).fails == 0 && App::busStatus(3).fails == 0);
  CHECK(App::devState<Cal>(8).offset == -2 && Cal::inits == 4);
}

// ---- 4. a stuck root bus: one probe per interval, the tree Stale, then the whole tree Alive with no rediscovery ------------
[[maybe_unused]] static void stuckRoot() {
  fresh();
  run(105); fbus::State::stuckRoot = true;
  run(111);
  CHECK(st(0) == Status::Stale && st(1) == Status::Stale && st(3) == Status::Stale && st(8) == Status::Stale && st(6) == Status::Stale);
  CHECK(App::busStatus(0).fails == 1 && devicesClean() && App::busStatus(2).fails == 0);
  const uint32_t s0 = fbus::State::starts;
  clearSeen();
  run(460); fbus::State::stuckRoot = false;
  CHECK(probes == 3 && probeAt[0] == 210 && probeAt[1] == 310 && probeAt[2] == 410);            // one per gate interval
  CHECK(fbus::State::starts - s0 == 3 && startsInPump == 0);                                     // and nothing else touched the bus
  CHECK(spaced(100));
  CHECK(seen.n[6] == 0 && seen.n[7] == 0 && seen.n[8] == 0);                                     // nothing polled while the bus is down
  run(700);
  CHECK(allAlive() && reg.count == 9 && App::discovers == 1);                                    // no rediscovery
  CHECK(Cal::inits == 2 + 2 * kR && recheckCount() == kR && App::devState<Cal>(7).offset == 3 && App::devState<Cal>(8).offset == -2);
  CHECK(Tap5<App, Cal>::sh.get() != nullptr);
  CHECK(App::statusWrites == 2 && unexplained == 0);
  const auto b = App::busStatus(0);
  CHECK(b.fails == 4 && b.retries == 4 && b.recovers == 4 && fbus::State::resets == 4);
  CHECK(devicesClean() && fbus::State::busyMs == 5 * 287);                                   // the fault (route + op) and three probes, all timed out: 5 x 287
  CHECK(seen.n[6] > 0 && seen.lo[7] == 218 && seen.hi[7] == 218);
}

// ---- 4b. a root bus that does not come back: Gone, the whole tree, bindings released ---------------------------------------
[[maybe_unused]] static void stuckRootForever() {
  fresh();
  run(105); fbus::State::stuckRoot = true;
  run(700);
  for (RowId r = 0; r < reg.count; ++r) CHECK(st(r) == Status::Gone);
  CHECK(Tap5<App, Cal>::sh.row == discover::noRow && Ban::lcd.row == discover::noRow && Rd::out.row == discover::noRow);
  CHECK(slotIsZero(7) && slotIsZero(8) && slotIsZero(4));
  CHECK(App::busStatus(0).retries == 4 && App::busStatus(0).drops == 1);
  fbus::State::stuckRoot = false;
  const uint32_t s0 = fbus::State::starts;
  run(1200);
  CHECK(fbus::State::starts == s0 && st(0) == Status::Gone);                                     // final: no more probing
}

// ---- 5. one stuck mux channel: scoped to it -------------------------------------------------------------------------------------
[[maybe_unused]] static void stuckChannel() {
  fresh();
  run(105); fbus::State::stuckCh[1] = true;
  run(111);
  CHECK(st(3) == Status::Stale && st(8) == Status::Stale);
  CHECK(st(0) == Status::Alive && st(1) == Status::Alive && st(2) == Status::Alive && st(6) == Status::Alive && st(7) == Status::Alive && st(4) == Status::Alive);
  CHECK(App::busStatus(3).fails == 1 && App::busStatus(0).fails == 0 && devicesClean());
  const uint16_t n6 = seen.n[6], n7 = seen.n[7], n8 = seen.n[8];
  run(450); fbus::State::stuckCh[1] = false;
  CHECK(seen.n[6] - n6 == 33 && seen.n[7] - n7 == 33 && seen.n[8] == n8);                       // the rest goes on every poll; the channel's device is silent
  run(700);
  CHECK(allAlive() && reg.count == 9 && App::discovers == 1 && Cal::inits == 2 + kR && recheckCount() == 0);      // only the channel's device is asked
  CHECK(App::statusWrites == 2 && unexplained == 0 && devicesClean());
  CHECK(App::busStatus(3).fails == 4 && App::busStatus(0).fails == 0);
  CHECK(fbus::State::timeouts == 4 && fbus::State::busyMs == 4 * 287);                           // the fault and three probes; the root answered every check
  CHECK(App::devState<Cal>(8).offset == -2 && seen.hi[8] == 251);
}
[[maybe_unused]] static void stuckChannelForever() {
  fresh();
  run(105); fbus::State::stuckCh[1] = true;
  run(600);
  CHECK(st(3) == Status::Gone && st(8) == Status::Gone && st(0) == Status::Alive && st(2) == Status::Alive && st(7) == Status::Alive && st(6) == Status::Alive);
  CHECK(Tap5<App, Cal>::sh.row == discover::noRow && slotIsZero(8) && !slotIsZero(7));
  const uint16_t n7 = seen.n[7];
  run(800);
  CHECK(seen.n[7] - n7 == 20 && st(3) == Status::Gone);
}

#if F5_STEP == 10
// ---- F2-1. a stuck root bus on a back-off: probes at 100, 200, 400 ms; the interval starts over after a recovery --------------------------
static void stuckRootBackoff() {
  fresh();
  run(105); fbus::State::stuckRoot = true;
  run(111);
  CHECK(st(0) == Status::Stale && st(1) == Status::Stale && st(3) == Status::Stale && st(8) == Status::Stale && st(6) == Status::Stale);
  CHECK(App::busStatus(0).fails == 1 && devicesClean() && App::busStatus(2).fails == 0);
  const uint32_t s0 = fbus::State::starts;
  clearSeen();
  run(460); fbus::State::stuckRoot = false;
  CHECK(probes == 2 && probeAt[0] == 210 && probeAt[1] == 410);                                  // 100 ms, then 200
  CHECK(fbus::State::starts - s0 == 2 && startsInPump == 0);                                     // and nothing else touched the bus
  CHECK(seen.n[6] == 0 && seen.n[7] == 0 && seen.n[8] == 0);
  run(800);
  CHECK(st(0) == Status::Stale);                                                                 // freed at 460, but the next probe is at 810
  run(900);
  CHECK(allAlive() && reg.count == 9 && App::discovers == 1);                                    // no rediscovery
  CHECK(Cal::inits == 2 + 2 * kR && recheckCount() == kR && App::devState<Cal>(7).offset == 3 && App::devState<Cal>(8).offset == -2);
  CHECK(Tap5<App, Cal>::sh.get() != nullptr && App::statusWrites == 2 && unexplained == 0);
  const auto b = App::busStatus(0);
  CHECK(b.fails == 3 && b.retries == 3 && b.recovers == 3 && fbus::State::resets == 3);
  CHECK(devicesClean() && fbus::State::busyMs == 4 * 287);                                       // the fault (route + op) and two probes that timed out
  // the interval starts over: the next fault is probed after 100 ms again, not 400
  clearSeen();
  run(1005); fbus::State::stuckRoot = true;
  run(1200);
  CHECK(probes == 1 && probeAt[0] == 1110);
  fbus::State::stuckRoot = false;
  run(1400);
  CHECK(allAlive() && probes == 2 && probeAt[1] == 1310 && App::statusWrites == 4);
}

// ---- F2-2. a root bus that does not come back stays Stale: bounded probing, nothing Gone, bindings kept --------------------------------
static void stuckRootStale() {
  fresh();
  run(105); fbus::State::stuckRoot = true;
  run(111);
  const uint32_t s0 = fbus::State::starts;
  clearSeen();
  run(3000);
  CHECK(probes == 8 && probeAt[0] == 210 && probeAt[1] == 410 && probeAt[2] == 810 && probeAt[3] == 1210 && probeAt[7] == 2810);   // 100, 200, then every 400
  CHECK(fbus::State::starts - s0 == 8 && startsInPump == 0);
  for (RowId r = 0; r < reg.count; ++r) CHECK(st(r) == Status::Stale);                           // Stale, never Gone
  CHECK(Tap5<App, Cal>::sh.row == 8 && Ban::lcd.row == 4 && !slotIsZero(4));                     // bindings kept, state kept
  CHECK(App::devState<Cal>(7).offset == 3 && App::devState<Cal>(8).offset == -2);
  CHECK(App::busStatus(0).drops == 0 && App::statusWrites == 1 && unexplained == 0);
  fbus::State::stuckRoot = false;
  run(3300);
  CHECK(allAlive() && App::statusWrites == 2 && Tap5<App, Cal>::sh.get() != nullptr);            // the probe at 3210 brings the whole tree back
}

// ---- F2-3. one stuck channel on a back-off ------------------------------------------------------------------------------------------------
static void stuckChannelBackoff() {
  fresh();
  run(105); fbus::State::stuckCh[1] = true;
  run(111);
  CHECK(st(3) == Status::Stale && st(8) == Status::Stale);
  CHECK(st(0) == Status::Alive && st(1) == Status::Alive && st(2) == Status::Alive && st(6) == Status::Alive && st(7) == Status::Alive && st(4) == Status::Alive);
  CHECK(App::busStatus(3).fails == 1 && App::busStatus(0).fails == 0 && devicesClean());
  const uint16_t n6 = seen.n[6], n7 = seen.n[7], n8 = seen.n[8];
  run(450); fbus::State::stuckCh[1] = false;
  CHECK(seen.n[6] - n6 == 33 && seen.n[7] - n7 == 33 && seen.n[8] == n8);                        // the rest goes on every poll
  run(900);
  CHECK(allAlive() && reg.count == 9 && App::discovers == 1 && Cal::inits == 2 + kR && recheckCount() == 0);
  CHECK(App::statusWrites == 2 && unexplained == 0 && devicesClean());
  CHECK(App::busStatus(3).fails == 3 && App::busStatus(0).fails == 0);                            // the fault, and probes at 210 and 410
  CHECK(fbus::State::timeouts == 3 && fbus::State::busyMs == 3 * 287);
}
static void stuckChannelStale() {
  fresh();
  run(105); fbus::State::stuckCh[1] = true;
  run(1500);
  CHECK(st(3) == Status::Stale && st(8) == Status::Stale && st(0) == Status::Alive && st(2) == Status::Alive && st(7) == Status::Alive && st(6) == Status::Alive);
  CHECK(Tap5<App, Cal>::sh.row == 8 && App::devState<Cal>(8).offset == -2);                       // not Gone: nothing released, nothing cleared
  const uint16_t n7 = seen.n[7];
  run(1800);
  CHECK(seen.n[7] - n7 == 30 && st(3) == Status::Stale);
}

// ---- F2-4. the return path at the top: what a row's last operation came to -----------------------------------------------------------------
static void returnPathSync() {
  fresh();
  run(15);
  CHECK(Cal::_serve(8).isOk() && SenB::_serve(6).isOk());                                        // a healthy poll: Ok, sticky
  CHECK(Cal::_serve(8).isOk());
  run(200); mock::Bus::devs[3].addr = 0x4F;                                                       // NACK from the poll at 200
  run(201);
  CHECK(Cal::_serve(8).isPending());                                                              // Retry absorbs the failure: the operation is in flight
  run(260);
  CHECK(Cal::_serve(8).isPending() && Cal::_serve(7).isOk());
  run(301);
  CHECK(st(8) == Status::Stale && Cal::_serve(8).failed() && Cal::_serve(8).kind() == Kind::Absent && Cal::_serve(8).detail == uint8_t(oneBus::TwiCause::Nack));   // exhausted: the failure surfaces
  mock::Bus::devs[3].addr = 0x48;
  run(1000);
  CHECK(st(8) == Status::Alive && Cal::_serve(8).isOk());                                         // back, and the next poll's outcome replaces it
  CHECK(dev(8).fails == 3 && dev(8).retries == 2);                                                // Status counted every pass
  // a transient failure is absorbed: the top only ever sees Pending, then Ok
  fresh();
  run(200); mock::Bus::devs[3].addr = 0x4F;
  run(201); CHECK(Cal::_serve(8).isPending());
  mock::Bus::devs[3].addr = 0x48;
  run(310);
  CHECK(Cal::_serve(8).isOk() && dev(8).fails == 1 && dev(8).retries == 1 && App::statusWrites == 0);
  CHECK(noOverflow());
}
#endif

// ---- 6. a cause the core cannot tell: Unknown is retried like a NACK, never recovered, never blamed on the bus ------------------
static void unknownCause() {
  fresh();
  fbus::State::causeless = true;
  run(200); mock::Bus::devs[3].addr = 0x4F;
  run(301);
  const auto d = dev(8);
  CHECK(st(8) == Status::Stale && d.lastKind == uint8_t(Kind::Unknown) && d.lastDetail == uint8_t(oneBus::TwiCause::Unknown) && d.retries == 2);
  CHECK(fbus::State::resets == 0 && App::busStatus(3).fails == 0 && App::busStatus(0).fails == 0 && st(3) == Status::Alive);
  // and at the bus edge: neither Absent nor Unknown recovers the bus (R-1)
  fresh();
  App::busFault(0, fail::Outcome::Fail(Kind::Unknown, 6));
  App::busFault(2, fail::Outcome::Fail(Kind::Absent, 1));
  CHECK(fbus::State::resets == 0 && App::busStatus(0).recovers == 0 && App::busStatus(2).recovers == 0);
  App::busFault(3, fail::Outcome::Fail(Kind::Fault, 4));
  CHECK(fbus::State::resets == 1 && App::busStatus(3).recovers == 1);                            // a Fault does
}

// ---- 7. the root sticks and the first operation to hit it is on a channel: the bus above is asked, and it owns the fault -----
static void stuckRootFromChannel() {
  fresh();
  mock::Bus::devs[1].addr = 0x4D;                                // the root's own sensor is gone: Stale on its own, and no longer polled
  run(101);
  CHECK(st(6) == Status::Stale && st(0) == Status::Alive);
  run(200); fbus::State::stuckRoot = true;
  run(211);
  CHECK(st(0) == Status::Stale && st(2) == Status::Stale && st(3) == Status::Stale && st(7) == Status::Stale && st(8) == Status::Stale);
  CHECK(App::busStatus(0).fails == 1 && App::busStatus(2).fails == 0 && App::busStatus(3).fails == 0);   // the root's, not the channel's
  CHECK(dev(7).fails == 0 && dev(8).fails == 0);
  run(250); fbus::State::stuckRoot = false;
  run(320);
  CHECK(st(6) == Status::Stale);                                 // the root is back; the sensor that is gone is still gone
  CHECK(st(0) == Status::Alive && st(1) == Status::Alive && st(2) == Status::Alive && st(3) == Status::Alive && st(7) == Status::Alive && st(8) == Status::Alive);
  run(650); mock::Bus::devs[1].addr = 0x40;
  run(720);
  CHECK(allAlive() && unexplained == 0);
}

// ---- 6b. a read that fails without a cause while the bus sticks: the probe that follows supplies it -------------------------------
static void unknownReadLeg() {
  fresh();
  fbus::State::causeless = true;
  run(105); fbus::State::stickOnRead = true;                     // the read leg of the poll at 110 finds the bus stuck, and says Unknown
  run(111);
  CHECK(st(0) == Status::Stale && st(3) == Status::Stale && st(8) == Status::Stale);
  CHECK(App::busStatus(0).fails == 1 && App::busStatus(0).lastKind == uint8_t(Kind::Timeout));   // the probe's cause, on the bus row
  CHECK(devicesClean() && App::busStatus(2).fails == 0 && App::busStatus(3).fails == 0);         // nothing counted against the device that saw the Unknown
  run(250); fbus::State::stuckRoot = false;
  run(kF2 ? 500 : 400);                                          // the probe at 210 finds it stuck; the next is at 310 (F5b) or 410 (F2's back-off)
  CHECK(allAlive() && App::statusWrites == 2 && unexplained == 0 && reg.count == 9);
  // a device that simply is not there is still that device's: the probe answers, the Unknown stays with it
  fresh();
  fbus::State::causeless = true;
  run(200); mock::Bus::devs[3].addr = 0x4F;
  run(301);
  CHECK(st(8) == Status::Stale && st(3) == Status::Alive && st(0) == Status::Alive && App::busStatus(0).fails == 0 && App::busStatus(3).fails == 0);
  CHECK(dev(8).lastKind == uint8_t(Kind::Unknown));
}

// ---- 8. a device that is down on its own while its bus goes down and comes back --------------------------------------------------
static void ownFaultAcrossBusFault() {
  fresh();
  run(200); mock::Bus::devs[3].addr = 0x4F;
  run(301); pumpOn = false;
  CHECK(st(8) == Status::Stale);
  run(600); fbus::State::stuckRoot = true;                       // the next thing to touch the bus is a device's re-probe
  run(702);
  CHECK(st(0) == Status::Stale && st(3) == Status::Stale && App::busStatus(0).fails == 1 && st(8) == Status::Stale);
  CHECK(dev(8).fails == 3);                                      // the re-probe's timeout was the bus's: not counted against the device
  run(850); fbus::State::stuckRoot = false;
  run(kF2 ? 1010 : 950);                                         // the bus's probes: 802 (fails), then 902 (F5b) or 1002 (F2's back-off)
  CHECK(st(0) == Status::Alive && st(3) == Status::Alive && st(7) == Status::Alive && st(6) == Status::Alive);
  CHECK(st(8) == Status::Stale);                                 // the bus is back; the device's own fault is not the bus's to clear
  run(999); mock::Bus::devs[3].addr = 0x48;
  run(1300);
  CHECK(st(8) == Status::Alive && allAlive() && Cal::inits == 3 + kR && recheckCount() == kR);          // the bus's return asks row 7 and row 6, not row 8 (down for its own reasons): its own return is the third init
  CHECK(unexplained == 0);
}

// ---- 9. the bare data path is R3's: a scripted run's checksum is what simavr will reproduce ----------------------------------------
static void scriptedRun() {
  fresh();
  faultScript();
  std::printf("fault script: checksum 0x%04X, %u status writes, starts %u, busy %u ms\n", failChecksum(), App::statusWrites, unsigned(fbus::State::starts), unsigned(fbus::State::busyMs));
  CHECK(st(7) == Status::Gone && st(8) == Status::Alive && st(0) == Status::Alive && st(3) == Status::Alive);
  CHECK(noOverflow());
}

// ---- F2-5. an overall deadline around a device's retry loop (built with -DF5_OVERALL_MS=60, shorter than the loop) -------------------------------
#if F5_STEP == 10 && F5_OVERALL_MS < 100
static void overallDevice() {
  fresh();
  run(200); mock::Bus::devs[3].addr = 0x4F;                      // the poll at 200 fails; the retries would be at 250 and 300
  run(261);
  const auto d = dev(8);
  CHECK(st(8) == Status::Stale);                                  // the loop was cut at 260 (armed at the tick of 200): 2 operations, not 3 (250 was the only re-issue)
  CHECK(d.fails == 3 && d.retries == 1 && d.drops == 1 && d.lastKind == uint8_t(Kind::Timeout) && d.lastDetail == 0xEE);   // the poll, the re-issue at 250, and the Timeout the deadline reports
  CHECK(Cal::_serve(8).failed() && Cal::_serve(8).kind() == Kind::Timeout);
  CHECK(unexplained == 0 && App::statusWrites == 1 && noOverflow());
}
#endif

// ---- 10. a kind with more rows than its table holds: the extra rows are unprotected and counted, and never touch another row's state ----
#ifdef F5_SMALL_K
static void smallK() {
  fresh(); topology();
  CHECK(Cal::overflows() == 0);
  run(200); mock::Bus::devs[3].addr = 0x4F;                      // row 8 (the second row of its kind, no slot) stops answering
  run(400);
  CHECK(st(8) == Status::Alive && Cal::overflows() > 0);         // nothing handles it: no retry, no Stale, counted
  if (Mode::returnPath) {
    const fail::Outcome o = Cal::_serve(8);                       // the app hears it through the return path, without reading a counter
    CHECK(o.failed() && o.kind() == Kind::Overflow && o.detail == kCalRows);
    CHECK(Cal::_serve(8).isIdle());                                // once
    CHECK(!Cal::_serve(7).failed());                                // a row with a slot is not reported
  } else CHECK(Cal::_serve(8).isIdle());                            // without a return path the counter is the only report
  CHECK(dev(8).fails == 0 && dev(8).retries == 0 && dev(8).drops == 0);
  CHECK(dev(7).fails == 0 && dev(7).retries == 0 && dev(7).drops == 0 && st(7) == Status::Alive);   // row 7's slot was never touched by row 8's failures
  mock::Bus::devs[2].addr = 0x4E;                                // row 7 (slot 0) stops answering: handled as always
  run(700);
  CHECK(st(7) == Status::Stale && dev(7).fails == 3 && dev(7).retries == 2 && dev(7).lastKind == uint8_t(Kind::Absent));
  CHECK(dev(8).fails == 0 && st(8) == Status::Alive && st(3) == Status::Alive && unexplained == 0);
  CHECK(SenB::overflows() == 0 && App::overflows() == 0);
  App::discover();
  CHECK(Cal::overflows() == 0);                                  // a discovery starts the count over
}
#endif

int main(int argc, char** argv) {
#if F5_STEP == 10 && F5_OVERALL_MS < 100
  (void)argc; (void)argv;
  for (uint8_t i = 0; i < mock::Bus::ndevs; ++i) pristine[i] = mock::Bus::devs[i];
  overallDevice();
  std::printf(failures ? "FAILED (%d)\n" : "OK: failCompose F2 overall deadline around a device's retry loop\n", failures);
  return failures != 0;
#endif
#ifdef F5_SMALL_K
  (void)argc; (void)argv;
  for (uint8_t i = 0; i < mock::Bus::ndevs; ++i) pristine[i] = mock::Bus::devs[i];
  smallK();
  std::printf(failures ? "FAILED (%d)\n" : "OK: failCompose F5 capacity (a kind with more rows than slots)\n", failures);
  return failures != 0;
#endif
  if (argc > 1 && !std::strcmp(argv[1], "scenario")) { faultScript(); std::printf("checksum 0x%04X\n", failChecksum()); return 0; }
  for (uint8_t i = 0; i < mock::Bus::ndevs; ++i) pristine[i] = mock::Bus::devs[i];
  healthy(); transientDevice(); transient(); nackAndReturn(); reprobeIdentity(); goneForGood();
#if F5_STEP == 10
  stuckRootBackoff(); stuckRootStale(); stuckChannelBackoff(); stuckChannelStale(); returnPathSync();
#else
  stuckRoot(); stuckRootForever(); stuckChannel(); stuckChannelForever();
#endif
 
  stuckRootFromChannel(); unknownCause(); unknownReadLeg(); ownFaultAcrossBusFault(); scriptedRun();
  std::printf("per row: device stack %zu B, bus stack %zu B, registry row %zu B\n", sizeof(Cal::Stack), sizeof(App::Ctl), sizeof(reg.rows[0]));
  std::printf(failures ? "FAILED (%d)\n" : "OK: failCompose F5 native (full composition)\n", failures);
  return failures != 0;
}
#endif
#endif
