// discoverCompose R3b: identification as a composition of entries (identify.h), on an ACK-reporting mock bus (mockAck.h).
//   native: the scenarios (default lists, ACK-only device, decoy, clone, protected range, listing = linking) with the
//           mock's per-address counts asserted; -DNEG_* variants are the mutations build_r3b.sh must see fail.
//   AVR:    -DR3B_TWO / R3B_PIN / R3B_CLONE / R3B_CLAIM / R3B_EEP pick the entries of one image (size matrix);
//           -DR3B_PARITY builds the same variant natively and prints its checksum.
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include <oneMachine/discover/binding.h>
#include "../support/mockAck.h"
#include "../support/sensors.h"
#include "../support/display.h"

using discover::RowId;
using discover::Sample;
using discover::Use;
using discover::Own;
using discover::Ignore;
using discover::Claim;
using discover::Protect;
using discover::IdProbe;
using discover::AddressProbe;
using discover::ReadProbe;
using discover::InSet;
using discover::Behind;
using hapi::Chain;

// ---- the sample consumer -------------------------------------------------------------------------
struct Log {
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

// ---- a device that is only pinned: no id, no probe -------------------------------------------------
template<typename W>
struct Eeprom : DriverBase<Eeprom<W>, W> {
  static constexpr uint8_t addrLo = 0x50, addrHi = 0x57;
};

// ---- a device that produces nothing and is refreshed: a display's shape ------------------------------------
template<typename W>
struct Refreshed : DriverBase<Refreshed<W>, W> {
  static constexpr uint8_t addrLo = 0x3C, addrHi = 0x3D;
  static constexpr bool polled = true;
  inline static uint8_t reads = 0;
  inline static RowId   last = discover::noRow;
  static void read(RowId row) { ++reads; last = row; }
};

// ---- a device whose identity is not in register 0: SensorB's mock device holds 64 in register 3
template<typename W>
struct RevB : DriverBase<RevB<W>, W> {
  static constexpr uint8_t addrLo = 0x40, addrHi = 0x40, id = 64, idReg = 3;
};

// ---- a direct connection on the display ---------------------------------------------------------------
template<typename W>
struct Banner {
  using Iface = TextDisplay<W>;
  inline static discover::Shell<W, Iface> lcd;

  template<typename Impl> static constexpr bool wants = std::is_same<Impl, Iface>::value;
  template<typename I> static void bind(RowId row, I* d) { lcd.bind(row, d); }
  static void unbind() { lcd.unbind(); }
  static void release(RowId r) { lcd.release(r); }
  static bool print(const char* s) { Iface* d = lcd.get(); if (!d) return false; d->print(lcd.row, s); return true; }
};

// ---- the application: entries `E<App>`, on bus `Twi` ------------------------------------------------------
template<template<typename> class E, typename TwiT = mockack::Twi, int Tag = 0>
struct AppT : discover::World<AppT<E, TwiT, Tag>, TwiT, Chain<Log>, E<AppT<E, TwiT, Tag>>, 16, discover::I2cScan> {
  using Self = AppT;
  template<typename Impl> static void bind(RowId row) { discover::BinderSet<Chain<Banner<Self>>>::template bind<Impl>(row); }
};

template<typename... L> struct CatT;
template<> struct CatT<> { using Type = Chain<>; };
template<typename... A> struct CatT<Chain<A...>> { using Type = Chain<A...>; };
template<typename... A, typename... B, typename... R> struct CatT<Chain<A...>, Chain<B...>, R...> { using Type = typename CatT<Chain<A..., B...>, R...>::Type; };

// ---- entry lists -------------------------------------------------------------------------------------------
template<typename W> using EDefault = Chain<SensorA<W>, SensorB<W>, Mux<W>, TextDisplay<W>>;                                   // today's list
template<typename W> using EOwn     = Chain<Use<Own, SensorA<W>>, Use<Own, SensorB<W>>, Use<Own, Mux<W>>, Use<Own, TextDisplay<W>>>;
template<typename W> using ERead    = Chain<SensorA<W>, SensorB<W>, Mux<W>>;                                                     // R1's list
template<typename W> using EPin     = Chain<Mux<W>, SensorA<W>, SensorB<W>, Use<AddressProbe<0x27>, TextDisplay<W>>>;
template<typename W> using EDecoy   = Chain<Ignore<0x70>, Mux<W>, SensorA<W>, SensorB<W>>;
template<typename W> using EClone   = Chain<SensorA<W>, Use<IdProbe<0, 0xA9, 0x48>, SensorA<W>>, SensorB<W>, Mux<W>>;
template<typename W> using EBehind  = Chain<Mux<W>, Use<AddressProbe<0x48, 0x48, Behind<0x70, 1>>, SensorA<W>>, SensorB<W>>;
template<typename W> using EDecoyAfter = Chain<Mux<W>, Ignore<0x70>, SensorA<W>, SensorB<W>>;
template<typename W> using EFirst   = Chain<Use<IdProbe<0, 0xB2, 0x40>, SensorB<W>>, Use<AddressProbe<0x40>, Eeprom<W>>, Mux<W>, SensorA<W>>;
template<typename W> using EProt    = Chain<Protect<0x50, 0x5F>, Mux<W>, SensorA<W>, SensorB<W>, Use<AddressProbe<0x50, 0x57>, Eeprom<W>>, Use<ReadProbe<0x58, 0x5F, 0x5A>, Eeprom<W>>>;
template<typename W> using EUnprot  = Chain<Mux<W>, SensorA<W>, SensorB<W>, Use<IdProbe<0, 0x5A, 0x50, 0x57>, Eeprom<W>>>;

template<typename W> using EPolled  = Chain<Mux<W>, SensorB<W>, Use<AddressProbe<0x3C, 0x3D>, Refreshed<W>>, Use<AddressProbe<0x50, 0x57>, Eeprom<W>>>;

template<typename W> using EIgnoreBridge = Chain<discover::IgnoreBridge<Mux<W>>, SensorB<W>>;                        // the real mux at 0x70: not used, cleared
template<typename W> using EIgnoreBridgeBad = Chain<discover::IgnoreBridge<Mux<W>>, Mux<W>, SensorB<W>>;                    // -fsyntax-only: names Mux both ways

template<typename W> using EMemo = Chain<Use<Own, SensorA<W>>, Use<IdProbe<0, 0xA9, 0x48>, SensorA<W>>, Use<Own, SensorB<W>>, Use<Own, Mux<W>>>;

template<typename W> using EIdReg   = Chain<Use<Own, RevB<W>>>;
template<typename W> using ERevBare = Chain<RevB<W>>;

struct Ovr { inline static bool set[128] = {}; static bool has(uint8_t a) { return set[a & 0x7F]; } };
template<typename W> using EOvr = Chain<Claim<InSet<Ovr>>, SensorA<W>, SensorB<W>, Mux<W>>;

// ---- compile-fail: each conflict rule, with its own message -------------------------------------------------------------
#if defined(NEG_REPEAT) || defined(NEG_PIN_PIN) || defined(NEG_CLAIM_PIN) || defined(NEG_PIN_PROBE) || defined(NEG_PROTECT) || defined(NEG_IGNORE_BRIDGE_USED)
#if defined(NEG_REPEAT)
  template<typename W> using ENeg = Chain<Mux<W>, Use<Own, SensorA<W>>, SensorB<W>, Use<Own, SensorA<W>>>;
#elif defined(NEG_PIN_PIN)
  template<typename W> using ENeg = Chain<Mux<W>, Use<AddressProbe<0x27>, TextDisplay<W>>, Use<AddressProbe<0x27, 0x28>, Eeprom<W>>>;
#elif defined(NEG_CLAIM_PIN)
  template<typename W> using ENeg = Chain<Mux<W>, Ignore<0x27>, Use<AddressProbe<0x27>, TextDisplay<W>>>;
#elif defined(NEG_PIN_PROBE)
  template<typename W> using ENeg = Chain<Mux<W>, Use<AddressProbe<0x40, 0x4F>, Eeprom<W>>, SensorB<W>>;
#elif defined(NEG_IGNORE_BRIDGE_USED)
  template<typename W> using ENeg = EIgnoreBridgeBad<W>;
#else
  template<typename W> using ENeg = Chain<Mux<W>, Protect<0x50, 0x5F>, Use<IdProbe<0, 0x5A, 0x50, 0x57>, Eeprom<W>>>;
#endif
void negUse() { AppT<ENeg>::discover(); }
#endif

// ---- size matrix (AVR) and its native parity run ---------------------------------------------------------------
#ifdef R3B_TWO
  template<typename W> using PBase = Chain<Use<Own, SensorA<W>>, Use<Own, SensorB<W>>, Use<Own, Mux<W>>>;
#else
  template<typename W> using PBase = Chain<SensorA<W>, SensorB<W>, Mux<W>>;
#endif
#ifdef R3B_PIN
  template<typename W> using PPin = Chain<Use<AddressProbe<0x27>, TextDisplay<W>>>;
#elif defined(R3B_LCD_BARE)
  template<typename W> using PPin = Chain<TextDisplay<W>>;
#else
  template<typename W> using PPin = Chain<>;
#endif
#ifdef R3B_CLONE
  template<typename W> using PClone = Chain<Use<IdProbe<0, 0xA9, 0x48>, SensorA<W>>>;
#else
  template<typename W> using PClone = Chain<>;
#endif
#ifdef R3B_CLAIM
  template<typename W> using PClaim = Chain<Ignore<0x72>>;
#else
  template<typename W> using PClaim = Chain<>;
#endif
#ifdef R3B_EEP
  template<typename W> using PEep  = Chain<Use<AddressProbe<0x50, 0x51>, Eeprom<W>>>;
  template<typename W> using PProt = Chain<Protect<0x50, 0x5F>>;
#else
  template<typename W> using PEep  = Chain<>;
  template<typename W> using PProt = Chain<>;
#endif
template<typename W> using EVar = typename CatT<PClaim<W>, PProt<W>, PBase<W>, PPin<W>, PClone<W>, PEep<W>>::Type;

// ---- table + samples folded into 16 bits, identical arithmetic on native and AVR ---------------------------------
template<typename A>
static uint16_t checksum() {
  uint16_t h = 0;
  for (RowId r = 0; r < A::reg.count; ++r) {
    const auto& row = A::reg.rows[r];
    h = uint16_t(h * 31u + row.busId + row.parent * 7u + row.isBus);
  }
  for (uint8_t i = 0; i < Log::n; ++i) h = uint16_t(h * 31u + Log::log[i].cap + Log::log[i].row + uint16_t(Log::log[i].v));
  return h;
}

#ifdef __AVR__
extern "C" void __cxa_pure_virtual() { for (;;) {} }
volatile uint16_t g_sum;
__attribute__((noinline)) void done() { for (;;) {} }

using App = AppT<EVar>;

int main() {
  mockack::State::reset();
  mockack::State::add(0x27, 0x00, false);     // the display keeps answering, its ID register reads 0
  mockack::State::add(0x50, 0x5A, true);      // two EEPROMs
  mockack::State::add(0x51, 0x5A, true);
  mockack::State::add(0x72, 0x00, true);      // something that answers and is nothing this bus knows
  mock::Bus::poke(1, 0x48, 0, 0xA9);          // a SensorA clone behind channel 1
  App::discover();
  App::pump();
  g_sum = checksum<App>();
  done();
}

#elif defined(R3B_PARITY)
#include <cstdio>
using App = AppT<EVar>;
int main() {
  mockack::State::reset();
  mockack::State::add(0x27, 0x00, false);
  mockack::State::add(0x50, 0x5A, true);
  mockack::State::add(0x51, 0x5A, true);
  mockack::State::add(0x72, 0x00, true);
  mock::Bus::poke(1, 0x48, 0, 0xA9);
  App::discover();
  App::pump();
  std::printf("rows %u\nchecksum 0x%04X\n", App::reg.count, checksum<App>());
  return 0;
}

#else
// ---------------------------------------------------------------------------------------------------------------------
#include <cstdio>
#include <cstring>

static int failures = 0, checks = 0;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

using mockack::State;

// which driver a row holds: 0 none, 1 Mux, 2 SensorA, 3 SensorB, 4 TextDisplay, 5 Eeprom, 6 RevB
template<typename A> static int drvCode(const discover::IDriver* d) {
  if (!d) return 0;
  if (d == discover::instOf<Mux<A>>()) return 1;
  if (d == discover::instOf<SensorA<A>>()) return 2;
  if (d == discover::instOf<SensorB<A>>()) return 3;
  if (d == discover::instOf<TextDisplay<A>>()) return 4;
  if (d == discover::instOf<Eeprom<A>>()) return 5;
  if (d == discover::instOf<RevB<A>>()) return 6;
  return -1;
}

struct Row { uint8_t busId; RowId parent; bool isBus; int drv; };

template<typename A> static void dump(const char* title) {
  std::printf("%s (%u rows, overflow %u):\n", title, A::reg.count, A::reg.overflow);
  for (RowId r = 0; r < A::reg.count; ++r) {
    const auto& w = A::reg.rows[r];
    std::printf("  row %2u: busId 0x%02X parent %3u %s drv %d\n", r, unsigned(w.busId), w.parent, w.isBus ? "bus" : "dev", drvCode<A>(w.drv));
  }
}

template<typename A, unsigned N> static bool same(const Row (&want)[N]) {
  if (A::reg.count != N) return false;
  for (RowId r = 0; r < N; ++r) {
    const auto& w = A::reg.rows[r];
    if (w.busId != want[r].busId || w.parent != want[r].parent || bool(w.isBus) != want[r].isBus || drvCode<A>(w.drv) != want[r].drv) return false;
  }
  return true;
}
template<typename A> static int rowsAt(uint8_t addr, bool bus = false) {
  int n = 0;
  for (RowId r = 0; r < A::reg.count; ++r) if (A::reg.rows[r].busId == addr && bool(A::reg.rows[r].isBus) == bus) ++n;
  return n;
}
template<typename A> static int rowOf(uint8_t addr) {
  for (RowId r = 0; r < A::reg.count; ++r) if (A::reg.rows[r].busId == addr && !A::reg.rows[r].isBus) return r;
  return -1;
}
static void clearLog() { Log::n = 0; }

// no data byte reached an address whose every address byte was refused
static bool noDataToSilent() {
  for (uint8_t a = 0x08; a <= 0x77; ++a) if (State::addressBytes(a) && State::nack[a] == State::addressBytes(a) && State::dataW[a]) return false;
  return true;
}
static bool noWriteIn(uint8_t lo, uint8_t hi) { for (uint8_t a = lo; a <= hi; ++a) if (State::writesTo(a)) return false; return true; }
static bool allReadIn(uint8_t lo, uint8_t hi)  { for (uint8_t a = lo; a <= hi; ++a) if (!State::slaR[a]) return false; return true; }

// the registry of the R1 topology plus the display at 0x27
static constexpr Row kDefaultRows[] = {
  {0x00, discover::noRow, true, 0},    // 0 root bus
  {0x70, 0, false, 1},                 // 1 mux
  {0x00, 1, true, 1},                  // 2 ch0 (a bus row points at its bridge's driver)
  {0x01, 1, true, 1},                  // 3 ch1
  {0x27, 0, false, 4},                 // 4 display
  {0x40, 0, false, 3},                 // 5 SensorB
  {0x48, 2, false, 2},                 // 6 SensorA behind ch0
  {0x48, 3, false, 2},                 // 7 SensorA behind ch1
};

// ---- 1. the default: today's lists, and the same list written as two-stage entries ---------------------------------
static void scenarioDefault() {
  using A = AppT<EDefault>;
  using B = AppT<EOwn>;
  static_assert(std::is_same<discover::DriversIn<EDefault<A>>, EDefault<A>>::value, "a plain list is its own driver list");
  static_assert(std::is_same<discover::DriversIn<EOwn<B>>, EDefault<B>>::value, "Use<Own,D> names D");

  State::reset();
  A::discover();
  dump<A>("default list (one-stage probes)");
  CHECK(same<A>(kDefaultRows));
  CHECK(A::reg.overflow == 0);
  clearLog(); A::pump();
  uint8_t na = Log::n; Log::Entry la[16]; std::memcpy(la, Log::log, sizeof la);
  CHECK(na == 4);                                         // SensorB: T + H, SensorA x2: T
  const uint16_t sumA = checksum<A>();
  // silent addresses inside a driver's range: the ID probe sends SLA+W and SLA+R per scan (root, ch0, ch1)
  CHECK(State::addressBytes(0x72) == 6 && State::slaW[0x72] == 3 && State::slaR[0x72] == 3);
  CHECK(State::addressBytes(0x41) == 0);                  // no entry names it
  const uint16_t oneStage = State::addressBytes(0x72);

  State::reset();
  B::discover();
  dump<B>("the same list as Use<Own,D> entries (two-stage)");
  CHECK(same<B>(kDefaultRows));
  clearLog(); B::pump();
  CHECK(Log::n == na && std::memcmp(Log::log, la, sizeof(Log::Entry) * na) == 0);
  CHECK(checksum<B>() == sumA);
  // stage 1 only: one write-probe per silent address per scan
  CHECK(State::addressBytes(0x72) == 3 && State::slaW[0x72] == 3 && State::slaR[0x72] == 0);
  CHECK(State::addressBytes(0x72) < oneStage);
  CHECK(noDataToSilent());                                                   // nothing written where nothing answered
  std::printf("  address bytes to the silent 0x72 over three scans: one-stage %u, two-stage %u\n", oneStage, State::addressBytes(0x72));
}

// ---- 2. an ACK-only display: found by an address entry, and bound ---------------------------------------------------
static void scenarioPin() {
  using A = AppT<EDefault>;
  using B = AppT<EPin>;
  State::reset();
  State::add(0x27, 0x00, false);                     // the display still takes its commands; its ID register reads 0
  A::discover();
  CHECK(rowsAt<A>(0x27) == 0);                       // the ID probe does not find it
  State::clearCounts();
  B::discover();
  dump<B>("ACK-only display, Use<AddressProbe<0x27>, TextDisplay>");
  CHECK(rowsAt<B>(0x27) == 1);
  const int r = rowOf<B>(0x27);
  CHECK(r >= 0 && drvCode<B>(B::reg.rows[r].drv) == 4 && B::reg.rows[r].parent == 0);
  CHECK(Banner<B>::lcd.row == r && Banner<B>::lcd.get() != nullptr);      // the binding hook ran for the pinned row
  CHECK(Banner<B>::print("HI"));
  CHECK(mockdisp::Screen::cell[0][0] == 'H' && mockdisp::Screen::cell[0][1] == 'I');
  CHECK(State::dataW[0x27] == 3 && State::slaW[0x27] >= 2);                // 0x03 'H' 'I'; the probe was SLA+W, STOP
  CHECK(State::slaR[0x27] == 0);                                           // discovery never read it: no ID register
  CHECK(rowsAt<B>(0x48) == 2 && rowsAt<B>(0x40) == 1 && rowsAt<B>(0x70) == 1);   // the rest of the bus is found as before
}

// ---- 3. a decoy at 0x70 and a real mux at 0x71 -------------------------------------------------------------------
static void scenarioDecoy() {
  using A = AppT<ERead>;
  using B = AppT<EDecoy>;
  State::reset();
  mock::Bus::devs[0].addr = 0x71;
  State::add(0x70, 0xC3, true);                       // answers, and its register 0 says "mux"
  A::discover();
  dump<A>("decoy, no entry for it");
  CHECK(rowsAt<A>(0x70) == 1 && rowsAt<A>(0x71) == 1);   // the decoy became a bridge
  CHECK(State::dataW[0x70] > 0);                         // and was written to

  State::reset();
  mock::Bus::devs[0].addr = 0x71;
  State::add(0x70, 0xC3, true);
  B::discover();
  dump<B>("decoy, Ignore<0x70> before the mux entry");
  CHECK(rowsAt<B>(0x70) == 0);                           // no row
  CHECK(rowsAt<B>(0x71) == 1 && drvCode<B>(B::reg.rows[rowOf<B>(0x71)].drv) == 1);   // the real mux is found
  CHECK(State::addressBytes(0x70) == 0 && State::writesTo(0x70) == 0 && State::dataW[0x70] == 0);   // never touched, not even probed
  CHECK(rowsAt<B>(0x48) == 2 && rowsAt<B>(0x40) == 1);
  const int mux = rowOf<B>(0x71);
  int behind = 0;
  for (RowId q = 0; q < B::reg.count; ++q) if (B::reg.rows[q].isBus && B::reg.rows[q].parent == mux) ++behind;
  CHECK(behind == 2);                                    // both channels of the real mux, none for the decoy
  clearLog(); B::pump();
  CHECK(Log::n == 4);

  // list order is precedence: the same claim listed after the mux entry does not hide the mux at 0x70; listed before it, it does
  State::reset();
  using C = AppT<EDecoyAfter>;
  C::discover();
  CHECK(rowsAt<C>(0x70) == 1 && rowsAt<C>(0x48) == 2);
  State::reset();                                   // a mux that is never identified is never cleared either
  B::discover();
  dump<B>("Ignore<0x70> before the mux entry, mux at 0x70");
  CHECK(rowsAt<B>(0x70) == 0 && rowsAt<B>(0x48) == 0 && rowsAt<B>(0x40) == 1);
  using F = AppT<EFirst>;
  F::discover();
  const int r40 = rowOf<F>(0x40);
  CHECK(r40 >= 0 && drvCode<F>(F::reg.rows[r40].drv) == 3);   // the ID entry is listed first; the pin behind it does not win

  // a runtime table: an address an app decides to ignore later
  using G = AppT<EOvr>;
  State::reset();
  G::discover();
  CHECK(rowsAt<G>(0x40) == 1);
  Ovr::set[0x40] = true;
  State::clearCounts();
  G::discover();
  CHECK(rowsAt<G>(0x40) == 0 && State::addressBytes(0x40) == 0);
  Ovr::set[0x40] = false;
  G::discover();
  CHECK(rowsAt<G>(0x40) == 1);
}

// ---- 4. a clone that answers a different ID ------------------------------------------------------------------------
static void scenarioClone() {
  using A = AppT<ERead>;
  using B = AppT<EClone>;
  using C = AppT<EBehind>;
  static_assert(discover::DriversIn<EClone<B>>::size == 3, "SensorA named by two entries is one driver");
  State::reset();
  mock::Bus::poke(1, 0x48, 0, 0xA9);
  A::discover();
  CHECK(rowsAt<A>(0x48) == 1 && A::reg.rows[rowOf<A>(0x48)].parent == 2);          // only the ordinary one
  B::discover();
  dump<B>("clone ID accepted: Use<IdProbe<0,0xA9,0x48>, SensorA>");
  CHECK(rowsAt<B>(0x48) == 2);
  CHECK(B::reg.rows[5].busId == 0x48 && B::reg.rows[5].parent == 2 && B::reg.rows[6].busId == 0x48 && B::reg.rows[6].parent == 3);
  CHECK(B::reg.rows[5].drv == B::reg.rows[6].drv && drvCode<B>(B::reg.rows[5].drv) == 2);   // the unchanged driver, twice
  clearLog(); B::pump();
  CHECK(Log::n == 4 && Log::log[3].row == 6 && Log::log[3].v == 253);                // the clone's own reading, through channel 1

  // a pin limited to one channel
  C::discover();
  CHECK(rowsAt<C>(0x48) == 1 && C::reg.rows[rowOf<C>(0x48)].parent == 3);            // behind channel 1 only, ID or not
}

// ---- 4c. two entries over one address share one stage-1 probe ---------------------------------------------------------
static void scenarioSharedPresence() {
  using M = AppT<EMemo>;
  State::reset();
  mock::Bus::devs[2].addr = 0x49; mock::Bus::devs[3].addr = 0x49;      // nothing at 0x48 now; two entries name it
  M::discover();
  CHECK(rowsAt<M>(0x48) == 0);
  CHECK(State::addressBytes(0x48) == 3 && State::slaW[0x48] == 3);      // one write-probe per scan, not one per entry
}

// ---- 5. a range that is only read-probed ---------------------------------------------------------------------------
static void scenarioProtected() {
  using A = AppT<EProt>;
  using U = AppT<EUnprot>;
  State::reset();
  State::add(0x50, 0x5A, true); State::add(0x51, 0x5A, true);
  State::add(0x58, 0x5A, true); State::add(0x59, 0x00, true);
  A::discover();
  dump<A>("protected 0x50-0x5F: AddressProbe pin + ReadProbe");
  CHECK(rowsAt<A>(0x50) == 1 && rowsAt<A>(0x51) == 1 && rowsAt<A>(0x58) == 1);
  CHECK(rowsAt<A>(0x59) == 0);                          // it answers a read, with the wrong value
  CHECK(noWriteIn(0x50, 0x5F));                                                  // no SLA+W and no data byte, anywhere in the range
  CHECK(allReadIn(0x50, 0x5F));                                                  // every address was read-probed
  CHECK(State::addressBytes(0x55) == 3 && State::slaR[0x55] == 3);                // a silent one: one read-probe per scan
  CHECK(State::slaR[0x50] == 1 && State::slaR[0x58] == 2);                       // pin: stage 1; ReadProbe: stage 1 and its one read
  CHECK(noDataToSilent());
  CHECK(rowsAt<A>(0x48) == 2 && rowsAt<A>(0x40) == 1);

  // what the rule keeps out: an ID entry over the same range writes the pointer byte to the devices there
  State::reset();
  State::add(0x50, 0x5A, true);
  U::discover();
  CHECK(rowsAt<U>(0x50) == 1);
  CHECK(State::dataW[0x50] > 0 && State::slaW[0x50] > 0);

  // a master that does not report ACK: stage 1 cannot see an absent address, so the pointer byte still reaches it
  using V = AppT<EOwn, mockack::TwiVoid, 1>;
  State::reset();
  V::discover();
  CHECK(same<V>(kDefaultRows));                                            // same table
  CHECK(State::dataW[0x71] > 0 && State::nack[0x71] > 0);                  // the pointer byte went to an address that refused it
}

// ---- 5c. Use<Own, D> reads the register the driver names as its identity ------------------------------------------------------------------------
static void scenarioIdReg() {
  using A = AppT<EIdReg>;
  using B = AppT<ERevBare>;
  State::reset();
  A::discover();
  CHECK(rowsAt<A>(0x40) == 1 && drvCode<A>(A::reg.rows[rowOf<A>(0x40)].drv) == 6);      // idReg = 3: 64, as declared
  CHECK(State::addressBytes(0x40) >= 3);
  B::discover();
  CHECK(rowsAt<B>(0x40) == 0);                                                           // the bare driver reads register 0 (0xB2): not it
}

// ---- 5d. a bus timeout is not absence: the presence probe tries once more ---------------------------------------------------------------------------
static void scenarioBusFault() {
  using A = AppT<EOwn>;
  State::reset();
  State::timeouts = 1;                                     // the first address byte of the scan times out, and the interface starts over
  A::discover();
  CHECK(same<A>(kDefaultRows));                            // the table is the default one: the device that timed out once was found
  State::reset();
  State::timeouts = 200;                                   // a bus that is down: every address byte times out
  A::discover();
  CHECK(A::reg.count == 1);                                // nothing found, and the scan ended (one retry per probe, not a loop)
  // and a NACK is an answer: no second try
  State::reset(); State::clearCounts();
  A::discover();
  CHECK(State::addressBytes(0x72) == 3);                   // as scenario 1: one write-probe per scan
}

// ---- 5e. a device that produces nothing is still refreshed when it says so; one that says nothing is not ------------------------------------------------
static void scenarioPolled() {
  using A = AppT<EPolled>;
  State::reset();
  State::add(0x3C, 0, true);                               // the display answers, it has no ID register
  State::add(0x50, 0, true);                               // a pinned device that produces nothing and does not say polled
  A::discover();
  const int oled = rowOf<A>(0x3C), eep = rowOf<A>(0x50);
  CHECK(oled >= 0 && eep >= 0 && rowOf<A>(0x40) >= 0);
  Refreshed<A>::reads = 0; clearLog();
  A::pump(); A::pump(); A::pump();
  CHECK(Refreshed<A>::reads == 3 && Refreshed<A>::last == RowId(oled));   // once per pump, for its own row
  CHECK(Log::n == 3 * 2);                                                  // SensorB's two capabilities per pump: the producer is polled as before
  CHECK(State::writesTo(0x50) == State::slaW[0x50]);                       // nothing but the presence probe ever reached the pinned device
}

// ---- 5f. IgnoreBridge: a real mux not used, its stale channel selection cleared (identify.h's own standing note 3) ---
static void scenarioIgnoreBridge() {
  using A = AppT<EIgnoreBridge>;
  State::reset();
  mock::Bus::mask = 3;                                     // a selection left over from before this run (both channels open)
  A::discover();
  CHECK(rowsAt<A>(0x70) == 0 && rowsAt<A>(0x71) == 0);      // no row for the mux: it is not used
  CHECK(mock::Bus::mask == 0);                              // cleared, not left stale
  CHECK(rowsAt<A>(0x40) == 1);                              // SensorB still found on the root, unaffected
  CHECK(rowsAt<A>(0x48) == 0);                              // SensorA behind the (cleared) mux does not leak onto the root
}

// ---- 6. what is not listed is not linked (the symbol side is build_r3b.sh's) -----------------------------------------
static void scenarioListing() {
  using A = AppT<ERead>;
  using B = AppT<EPin>;
  static_assert(discover::HasIn<Eeprom<A>, discover::DriversIn<EProt<A>>>::value && !discover::HasIn<Eeprom<A>, discover::DriversIn<ERead<A>>>::value, "");
  static_assert(!discover::HasIn<TextDisplay<A>, discover::DriversIn<ERead<A>>>::value && discover::HasIn<TextDisplay<B>, discover::DriversIn<EPin<B>>>::value, "");
  static_assert(discover::DriversIn<EDecoy<A>>::size == 3, "a Claim names no driver");
  static_assert(discover::DriversIn<EProt<A>>::size == 4, "Protect names no driver; Eeprom is one driver in two entries");
  ++checks;
}

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);      // a mutation that crashes still shows the checks it failed first
  scenarioDefault();
  scenarioPin();
  scenarioDecoy();
  scenarioClone();
  scenarioSharedPresence();
  scenarioProtected();
  scenarioIdReg();
  scenarioBusFault();
  scenarioPolled();
  scenarioIgnoreBridge();
  scenarioListing();
  std::printf("checks %d\n", checks);
  std::printf(failures ? "FAILED (%d)\n" : "OK: discoverCompose R3b native\n", failures);
  return failures != 0;
}
#endif
