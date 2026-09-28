// discoverCompose Round 3: what a row owns. Device state in the row (sized by the driver list), client state in each
// binding consumer, and removal reaching the bindings through the registry.
//   native: shared backlight and separate cursors on one display row, per-row calibration in the samples, and the
//           removal / return / bridge-removal / rediscovery scenarios, with no discoverAll() anywhere.
//   AVR:    build.sh links this for atmega328p; R3_NO_DEV / R3_NO_CLIENT / R3_NO_LIFE drop each piece for the size matrix.
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

using discover::RowId;
using discover::Sample;
using discover::Status;
using hapi::Chain;

#ifdef R3_NO_LIFE
  static constexpr bool kLife = false;
#else
  static constexpr bool kLife = true;
#endif

// ---- a sample consumer: what reaches the fan-out -----------------------------------------------------------
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

// ---- direct connection: a shell on the display with a cursor of its own -------------------------------------
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

// ---- capability-set output class, first and last provider row ----------------------------------------------
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

// ---- application -------------------------------------------------------------------------------------------
#ifdef NEG_UNDECLARED_STATE
template<typename W> struct Rogue : discover::DriverBase<Rogue<W>, W> {
  using B = discover::DriverBase<Rogue<W>, W>;
  static constexpr uint8_t addrLo = 0x50, addrHi = 0x50, id = 0x77;
  static void init(RowId row) { (void)B::dev(row); }
};
#endif

template<uint8_t N> struct App3N;
template<uint8_t N> using Drivers3 = Chain<CalSensor<App3N<N>>, SensorB<App3N<N>>, Mux<App3N<N>>, StateDisplay<App3N<N>>, LineDisplay<App3N<N>>
#ifdef NEG_UNDECLARED_STATE
  , Rogue<App3N<N>>
#endif
>;
template<uint8_t N> using Binders3 = Chain<Banner3<App3N<N>>, Handle3<App3N<N>, Drivers3<N>, false>, Handle3<App3N<N>, Drivers3<N>, true>>;

template<uint8_t N>
struct App3N : discover::World<App3N<N>, mock3::Twi, Chain<Log3>, Drivers3<N>, N, discover::I2cScan> {
  static constexpr bool lifecycle = kLife;

  template<typename Impl> static void bind(RowId row) { discover::BinderSet<Binders3<N>>::template bind<Impl>(row); }
  static void release(RowId row)                       { discover::BinderSet<Binders3<N>>::release(row); }
  static void unbindAll()                              { discover::BinderSet<Binders3<N>>::unbind(); }
};

using App = App3N<10>;
using Ban = Banner3<App>;
using Rd  = Handle3<App, Drivers3<10>, false>;    // first provider row
using Mr  = Handle3<App, Drivers3<10>, true>;     // last provider row
using CS  = CalSensor<App>;
using SD  = StateDisplay<App>;
using D3  = mock3::Display;
using S2  = mockdisp::Screen2;

static_assert(discover::BinderSet<Binders3<10>>::served<Drivers3<10>>, "a binding consumer names a driver that is not in the driver list");
// zero cost where nothing is declared: R1's and R2's driver lists have no slot at all
template<typename X> using R2Drivers = Chain<SensorA<X>, SensorB<X>, Mux<X>, TextDisplay<X>, LineDisplay<X>>;
// the world these drivers are named with: none is needed. MSVC instantiates a class template's virtual members along with
// the class, so there the world has to be a complete type that has route() and reg.
#ifdef _MSC_VER
using NoWorld = App;
#else
struct NoWorld;
#endif
static_assert(__is_empty(discover::DevSlotOf<R2Drivers<NoWorld>>) && __is_empty(discover::CliSlotOf<R2Drivers<NoWorld>>),
              "drivers that declare no state need no slot");
static_assert(!discover::DeviceStateOf<SensorB<NoWorld>>::has && !discover::DeviceStateOf<Mux<NoWorld>>::has &&
              !discover::DeviceStateOf<LineDisplay<NoWorld>>::has, "SensorB, Mux and LineDisplay declare nothing");
#ifndef __AVR__
static_assert(!kDev || sizeof(App::Dev<>::Slot) == sizeof(int16_t), "one slot per row, the size of the largest DeviceState");
#endif

// ---- everything a run leaves behind, folded into 16 bits: identical arithmetic on native and AVR ------------
template<typename T> static uint16_t hashBytes(uint16_t h, const T& t) {
  auto* p = reinterpret_cast<const uint8_t*>(&t);
  for (unsigned i = 0; i < sizeof(T); ++i) h = uint16_t(h * 31u + p[i]);
  return h;
}
// templates, so that the branch for state that is not declared is discarded, not compiled
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
  h = uint16_t(h * 31u + CS::inits + SD::inits + D3::backlight);
  return hashCursor<kClient>(h);
}

#ifdef NEG_STATUS_WITHOUT_LIFECYCLE
static void negProbe() { App::setStatus(4, Status::Gone); }      // an app without lifecycle must not write status
#endif
#ifdef NEG_STATUS_FIELD_WRITE
static void negField() { App::reg.rows[4].st_ = 1; }             // the field is private
#endif
#ifdef NEG_STATUS_REGISTRY_WRITE
static void negWriter() { App::reg.writeStatus(4, Status::Gone); }   // and so is the registry's writer
#endif

static void scenario() {
  CS::inits = 0; SD::inits = 0;
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

// ---- AVR entry -----------------------------------------------------------------------------------------
#ifdef __AVR__
extern "C" void __cxa_pure_virtual() { for (;;) {} }
volatile uint16_t g_sum;
__attribute__((noinline)) void done() { for (;;) {} }
int main() {
  scenario();
  g_sum = checksum();
  done();
}
#elif defined(R3_PARITY)
#include <cstdio>
int main() {
  scenario();
  std::printf("checksum 0x%04X\n", checksum());
  return 0;
}
#else

#if defined(R3_NO_DEV) || defined(R3_NO_CLIENT) || defined(R3_NO_LIFE)
  #error "the native test needs the full configuration; the R3_NO_* switches are for the size builds"
#endif

// ---- native test -----------------------------------------------------------------------------------------
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

using discover::instOf;
using Slot = App::Dev<>::Slot;
using Tab  = App::Dev<>::Table;
static auto& reg = App::reg;

static const char* drvName(const discover::IDriver* d) {
  if (!d) return "-";
  if (d == instOf<CS>())                 return "CalSensor";
  if (d == instOf<SensorB<App>>())       return "SensorB";
  if (d == instOf<Mux<App>>())           return "Mux";
  if (d == instOf<SD>())                 return "StateDisplay";
  if (d == instOf<LineDisplay<App>>())   return "LineDisplay";
  return "?";
}

// the command log a display receives
struct Log {
  std::string s;
  Log& C() { s += 'C'; return *this; }
  Log& W(const char* t) { for (; *t; ++t) { s += 'W'; s += *t; } return *this; }
  Log& S(char x, char y) { s += 'S'; s += x; s += y; return *this; }
  Log& B(uint8_t b) { s += 'B'; s += char(b); return *this; }
};
template<typename S> static std::string lineOf(uint8_t l) { return std::string(S::cell[l], S::cols); }
template<typename S> static std::string logOf() { return std::string(reinterpret_cast<const char*>(S::log), S::logN); }

static bool slotIsZero(RowId r) {
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&Tab::rows[r]);
  for (unsigned i = 0; i < sizeof(Slot); ++i) if (p[i]) return false;
  return true;
}
static void setBridge(bool present) {
  for (auto& d : mock::Bus::devs) if (d.bridge) d.addr = present ? 0x70 : 0x7E;     // 0x7E is outside the scanned range
}

// everything discovery leaves: the table, the bindings, every row's DeviceState, every consumer's ClientState
struct Snap {
  std::string s;
  bool operator==(const Snap& o) const { return s == o.s; }
};
static Snap snap() {
  Snap r;
  char b[96];
  for (RowId i = 0; i < reg.count; ++i) {
    std::snprintf(b, sizeof b, "row %u %02x p%u bus%u st%u %s |", i, unsigned(reg.rows[i].busId), unsigned(reg.rows[i].parent),
                  unsigned(reg.rows[i].isBus), unsigned(reg.rows[i].status()), drvName(reg.rows[i].drv));
    r.s += b;
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&Tab::rows[i]);
    for (unsigned k = 0; k < sizeof(Slot); ++k) { std::snprintf(b, sizeof b, " %02x", p[k]); r.s += b; }
    r.s += '\n';
  }
  std::snprintf(b, sizeof b, "ban %u %u,%u | rd %u %s | mr %u %s\n", unsigned(Ban::lcd.row), Ban::cursor().col, Ban::cursor().row,
                unsigned(Rd::out.row), drvName(Rd::out.drv), unsigned(Mr::out.row), drvName(Mr::out.drv));
  r.s += b;
  const auto* rc = reinterpret_cast<const uint8_t*>(&Rd::out.cs); const auto* mc = reinterpret_cast<const uint8_t*>(&Mr::out.cs);
  std::snprintf(b, sizeof b, "client slots rd %02x%02x mr %02x%02x\n", rc[0], rc[1], mc[0], mc[1]);
  r.s += b;
  return r;
}
static void clearInits() { CS::inits = 0; SD::inits = 0; }
static void clearLogs() { D3::clearScreen(); S2::reset(); Log3::n = 0; }

int main() {
  // ---- baseline discovery: R2's topology, the display with state, the sensors calibrated ---------------------
  D3::present = true; D3::addr = 0x27; D3::reset(); S2::reset();
  mock::Bus::mask = 2;
  mock::Bus::poke(0, 0x48, 3, 3);                                   // calibration register of the SensorA behind ch0
  mock::Bus::poke(1, 0x48, 3, uint8_t(-2));                          // and behind ch1
  clearInits();
  App::discover();

  struct Expect { uint8_t busId; RowId parent; bool isBus; const char* drv; };
  const Expect want[] = {
    {0x00, discover::noRow, true, "-"}, {0x70, 0, false, "Mux"}, {0x00, 1, true, "Mux"}, {0x01, 1, true, "Mux"},
    {0x27, 0, false, "StateDisplay"}, {0x3F, 0, false, "LineDisplay"}, {0x40, 0, false, "SensorB"},
    {0x48, 2, false, "CalSensor"}, {0x48, 3, false, "CalSensor"},
  };
  CHECK(reg.count == 9 && reg.overflow == 0);
  for (RowId r = 0; r < reg.count && r < 9; ++r) {
    CHECK(reg.rows[r].busId == want[r].busId && reg.rows[r].parent == want[r].parent && bool(reg.rows[r].isBus) == want[r].isBus);
    CHECK(std::strcmp(drvName(reg.rows[r].drv), want[r].drv) == 0);
  }
  CHECK(Ban::lcd.row == 4 && Rd::out.row == 4 && Rd::out.drv == instOf<SD>() && Mr::out.row == 5 && Mr::out.drv == instOf<LineDisplay<App>>());
  const Snap first = snap();

  // ---- device state: written at found(), per row, visible to every consumer --------------------------------
  CHECK(sizeof(Slot) == 2 && sizeof(Tab::rows) == (10 + 1) * 2);   // one slot per row, the size of the largest DeviceState, plus a scratch slot
  CHECK(App::devState<CS>(7).offset == 3 && App::devState<CS>(8).offset == -2);
  CHECK(App::devState<SD>(4).backlight && D3::backlight);          // found() switched it on, on the device and in the row
  CHECK(CS::inits == 2 && SD::inits == 1);                         // each device initialised once
  CHECK(slotIsZero(0) && slotIsZero(1) && slotIsZero(2) && slotIsZero(3) && slotIsZero(5) && slotIsZero(6));   // rows of drivers that declare nothing

  Log3::n = 0;
  App::pump();
  CHECK(Log3::n == 4);
  CHECK(Log3::log[0].row == 6 && Log3::log[0].v == 187 && Log3::log[1].row == 6 && Log3::log[1].v == 64);      // SensorB: stateless, unchanged
  CHECK(Log3::log[2].row == 7 && Log3::log[2].v == 215 + 3);                                                    // 21.5 C + its own offset
  CHECK(Log3::log[3].row == 8 && Log3::log[3].v == 253 - 2);                                                    // 25.3 C + its own offset

  // ---- client state: one display row, two consumers, two cursors; one backlight -------------------------------
  clearLogs();
  CHECK(Ban::at(0, 1) && Ban::print("AB"));
  CHECK(Rd::print("CD"));
  CHECK(Ban::print("EF") && Rd::print("GH"));
  CHECK(lineOf<D3>(0) == "CDGH            " && lineOf<D3>(1) == "ABEF            ");
  CHECK(logOf<D3>() == Log().S(0, 1).W("AB").S(0, 0).W("CD").S(2, 1).W("EF").S(2, 0).W("GH").s);   // each print repositions the device cursor
  CHECK(Ban::cursor().col == 4 && Ban::cursor().row == 1);
  CHECK(Rd::out.client<SD>().col == 4 && Rd::out.client<SD>().row == 0);
  CHECK(Mr::out.cs.head.col == 0);                                                  // the other handle's slot was not touched (it is on the other display)

  clearLogs();
  CHECK(Ban::lit() && Ban::light(false) && !Ban::lit() && !D3::backlight);
  CHECK(Rd::print("X"));                                            // the class consumer prints: the driver wakes the display
  CHECK(Ban::lit() && D3::backlight && App::devState<SD>(4).backlight);   // and the direct consumer sees it in the row
  CHECK(logOf<D3>() == Log().B(0).B(1).S(4, 0).W("X").s);          // off, then the wake-up, then the print at this consumer's cursor

  // ---- Stale: refused like Gone, but the binding and its session stay, and it comes back --------------------
  App::setStatus(4, Status::Stale);
  CHECK(reg.status(4) == Status::Stale && Ban::lcd.row == 4 && Rd::out.row == 4);        // bound, not released
  clearLogs();
  CHECK(!Ban::print("S") && !Rd::print("S") && !Ban::light(true) && D3::logN == 0);
  App::setStatus(4, Status::Alive);
  CHECK(Ban::print("S"));
  CHECK(logOf<D3>() == Log().S(4, 1).W("S").s);                     // the same cursor: the session was kept

  // ---- nothing indexes past a table: a row that is not a row is refused or lands in scratch -----------------
  CHECK(reg.driverOf(discover::noRow) == nullptr && reg.driverOf(reg.count) == nullptr && reg.status(discover::noRow) == Status::Gone);
  CHECK(&Tab::at(discover::noRow) == &Tab::rows[10] && &Tab::at(200) == &Tab::rows[10] && &Tab::at(3) == &Tab::rows[3]);   // the address, not the symptom: an out-of-bounds write is silent without a sanitizer
  {
    const Snap before = snap();
    App::devState<SD>(discover::noRow).backlight = true;            // what a call with a released binding's row would do
    App::devState<CS>(200).offset = 77;
    CHECK(snap() == before);                                        // no real row changed
    Tab::clear(discover::noRow);
    CHECK(Tab::at(discover::noRow).template as<CS::DeviceState>().offset == 0);
  }

  // ---- 1. the display is removed: through the registry, no discoverAll() ------------------------------------
  D3::present = false;
  App::setStatus(4, Status::Gone);
  CHECK(reg.status(4) == Status::Gone && reg.status(5) == Status::Alive && reg.status(6) == Status::Alive);
  CHECK(slotIsZero(4));                                             // its DeviceState is cleared
  CHECK(Ban::lcd.row == discover::noRow && Ban::lcd.get() == nullptr && Rd::out.row == discover::noRow);    // both bindings released
  CHECK(Ban::cursor().col == 0 && Ban::cursor().row == 0);          // and the consumer's session is over
  CHECK(Mr::out.row == 5);                                          // the other display's binding is untouched
  clearLogs();
  CHECK(!Ban::print("Z") && !Ban::light(true) && !Rd::print("Z"));
  CHECK(Mr::print("MM") && lineOf<S2>(0) == "MM              ");
  Log3::n = 0;
  App::pump();
  CHECK(D3::logN == 0);
  CHECK(Log3::n == 4 && Log3::log[2].v == 218 && Log3::log[3].v == 251);   // the other rows go on, calibration intact

  // ---- 2. the display returns at another row index ---------------------------------------------------------
  D3::addr = 0x41; D3::present = true; D3::powerCycle();
  clearInits(); clearLogs();
  App::discover();                                                  // a rediscovery: nothing calls discoverAll()
  const Expect want2[] = {
    {0x00, discover::noRow, true, "-"}, {0x70, 0, false, "Mux"}, {0x00, 1, true, "Mux"}, {0x01, 1, true, "Mux"},
    {0x3F, 0, false, "LineDisplay"}, {0x40, 0, false, "SensorB"}, {0x41, 0, false, "StateDisplay"},
    {0x48, 2, false, "CalSensor"}, {0x48, 3, false, "CalSensor"},
  };
  CHECK(reg.count == 9);
  for (RowId r = 0; r < reg.count && r < 9; ++r) CHECK(reg.rows[r].busId == want2[r].busId && std::strcmp(drvName(reg.rows[r].drv), want2[r].drv) == 0);
  CHECK(Ban::lcd.row == 6 && Ban::lcd.get() != nullptr);            // the direct connection follows its type to the new index
  CHECK(Rd::out.row == 4 && Rd::out.drv == instOf<LineDisplay<App>>());     // first provider row is now the other driver type
  CHECK(Mr::out.row == 6 && Mr::out.drv == instOf<SD>());                   // last provider row
  CHECK(Ban::cursor().col == 0 && Ban::cursor().row == 0);          // a new session: the cursor starts over
  CHECK(App::devState<SD>(6).backlight && D3::backlight);           // DeviceState re-initialised by found()
  const Snap second = snap();
  CHECK(slotIsZero(4) && slotIsZero(5));                            // the rows that held the old display and SensorB slots
  CHECK(App::devState<CS>(7).offset == 3 && App::devState<CS>(8).offset == -2);
  CHECK(CS::inits == 2 && SD::inits == 1);                          // one init per device, none doubled
  CHECK(Ban::print("Q") && lineOf<D3>(0) == "Q               ");
  CHECK(Rd::print("R") && lineOf<S2>(0) == "R               ");     // the first-provider consumer now reaches the other display

  // ---- 4. rediscovery with nothing changed ends identical to the first discovery ----------------------------
  CHECK(Ban::print("dirty") && Ban::light(false) && Mr::print("dirty") && Rd::print("dirty"));      // leave state behind
  clearInits();
  App::discover();
  CHECK(snap() == second);
  CHECK(CS::inits == 2 && SD::inits == 1);

  // ---- back to the first topology: identical to the first discovery, bindings and states -------------------
  D3::addr = 0x27; D3::powerCycle();
  App::discover();
  CHECK(snap() == first);

  // ---- 3. the bridge is removed: everything below it goes, the root devices stay ---------------------------
  clearLogs();
  setBridge(false);
  mock::Bus::mask = 0;
  const uint8_t hits = D3::hits70;
  App::setStatus(1, Status::Gone);
  const RowId gone[] = {1, 2, 3, 7, 8}, alive[] = {0, 4, 5, 6};
  for (RowId r : gone)  CHECK(reg.status(r) == Status::Gone);
  for (RowId r : alive) CHECK(reg.status(r) == Status::Alive);
  CHECK(slotIsZero(7) && slotIsZero(8) && !slotIsZero(4));          // the sensors' state is cleared, the display's is not
  CHECK(Ban::lcd.get() != nullptr && Rd::out.row == 4 && Mr::out.row == 5);
  CHECK(Ban::print("ok") && Rd::print("ok"));                       // a root device is used: routing must not touch the removed bridge
  App::pump();
  CHECK(Log3::n == 2 && Log3::log[0].row == 6 && Log3::log[1].row == 6);   // only SensorB (row 6) still reports
  CHECK(D3::hits70 == hits);                                        // nothing was addressed to the removed bridge
  CHECK(mock::Bus::contention == 0);

  // ---- the bridge returns, and so does the first discovery, exactly ------------------------------------------
  setBridge(true);
  clearInits();
  App::discover();
  CHECK(snap() == first);
  CHECK(CS::inits == 2 && SD::inits == 1);

  std::printf("state: slot %u B x (%u rows + scratch) = %u B; client: shell %u B, handle %u B\n", unsigned(sizeof(Slot)), 10u, unsigned(sizeof(Tab::rows)),
              unsigned(sizeof(Ban::cursor())), unsigned(sizeof(Rd::out.cs)));
  scenario();
  std::printf("checksum 0x%04X\n", checksum());
  std::printf(failures ? "FAILED (%d)\n" : "OK: discoverCompose R3 native\n", failures);
  return failures != 0;
}
#endif
