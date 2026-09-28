// discoverCompose Round 1: runtime-discovered registry -> one virtual hop per row -> static capability fan-out.
//   native: table == simulated topology, samples reach exactly the right consumers with the right row id.
//   AVR:    build.sh links this for atmega328p and disassembles World::pump and the poll() bodies.
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include "../support/mockTwi.h"
#include "../support/sensors.h"

using discover::RowId;
using discover::DriverBase;
using discover::Sample;
using hapi::Chain;

// ---- consumers (fixed by the build; storage is static, the fan-out itself is stateless) --------
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

// ---- applications ----------------------------------------------------------------------------
template<uint8_t N> struct AppN;
#ifdef NEG_DUP_DRIVER
template<uint8_t N> using DriversOf = Chain<SensorA<AppN<N>>, SensorB<AppN<N>>, Mux<AppN<N>>, SensorA<AppN<N>>>;
#elif defined(NEG_DERIVED_DRIVER)
template<typename W> struct SensorA2 : SensorA<W> { static constexpr uint8_t id = 0xEE; };
template<uint8_t N> using DriversOf = Chain<SensorA2<AppN<N>>, SensorB<AppN<N>>, Mux<AppN<N>>>;
#else
template<uint8_t N> using DriversOf = Chain<SensorA<AppN<N>>, SensorB<AppN<N>>, Mux<AppN<N>>>;
#endif
template<uint8_t N> struct AppN : discover::World<AppN<N>, mock::Twi, Consumers, DriversOf<N>, N, discover::I2cScan> {};

using App = AppN<8>;

// fan-outs: stateless, structural, one per capability
using FanT = discover::CapFanoutT<Temperature, Consumers>;
using FanH = discover::CapFanoutT<Humidity, Consumers>;
static_assert(!__is_polymorphic(FanT) && sizeof(FanT) == 1, "Temperature fan-out must be stateless");
static_assert(!__is_polymorphic(FanH) && sizeof(FanH) == 1, "Humidity fan-out must be stateless");
// capability -> producers is a static query too
static_assert(discover::ProducersOf<Temperature, DriversOf<8>>::size == 2, "Temperature: SensorA + SensorB");
static_assert(discover::ProducersOf<Humidity,    DriversOf<8>>::size == 1, "Humidity: SensorB");
static_assert(discover::DriverSet<DriversOf<8>>::distinct, "distinct");
static_assert(sizeof(App::BusIdT) == 1, "I2C-only bus list -> 8-bit busId");
#ifdef __AVR__
static_assert(sizeof(App::reg.rows[0]) == 5, "row = busId + ptr + parent + flags");
#endif

// ---- table + samples folded into 16 bits, identical arithmetic on native and AVR -------------------
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

// ---- AVR entry -------------------------------------------------------------------------------
#ifdef __AVR__
extern "C" void __cxa_pure_virtual() { for (;;) {} }
volatile uint16_t g_sum;
__attribute__((noinline)) void done() { for (;;) {} }
int main() {
  App::discover();
  App::pump();
  g_sum = checksum();
  done();
}
#else

// ---- native test -----------------------------------------------------------------------------
#include <cstdio>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

struct Expect { uint8_t busId; RowId parent; bool isBus; discover::IDriver* drv; };

static void clearLogs() { TempLogger::n = 0; ValuePrinter::n = 0; }

int main() {
  using discover::instOf;
  mock::Bus::mask = 2;   // stale bridge selection from before: ch1 is visible at power-up
  App::discover();
  auto& reg = App::reg;

  std::printf("registry (%u rows, overflow %u):\n", reg.count, reg.overflow);
  for (RowId r = 0; r < reg.count; ++r)
    std::printf("  row %u: busId 0x%02X parent %3u %s drv %p\n", r, unsigned(reg.rows[r].busId), reg.rows[r].parent,
                reg.rows[r].isBus ? "bus" : "dev", (void*)reg.rows[r].drv);

  // table == simulated topology (BFS; per bus: bridges first, then the rest; then each channel bus in row order)
  const Expect want[] = {
    {0x00, discover::noRow, true,  nullptr},                              // 0 root bus
    {0x70, 0, false, instOf<Mux<App>>()},                                 // 1 bridge on root
    {0x00, 1, true,  instOf<Mux<App>>()},                                 // 2 bridge ch0
    {0x01, 1, true,  instOf<Mux<App>>()},                                 // 3 bridge ch1
    {0x40, 0, false, instOf<SensorB<App>>()},                             // 4 SensorB on root
    {0x48, 2, false, instOf<SensorA<App>>()},                             // 5 SensorA behind ch0
    {0x48, 3, false, instOf<SensorA<App>>()},                             // 6 SensorA behind ch1
  };
  CHECK(reg.count == 7);
  CHECK(reg.overflow == 0);
  for (RowId r = 0; r < reg.count && r < 7; ++r) {
    CHECK(reg.rows[r].busId == want[r].busId);
    CHECK(reg.rows[r].parent == want[r].parent);
    CHECK(bool(reg.rows[r].isBus) == want[r].isBus);
    CHECK(reg.rows[r].drv == want[r].drv);
    CHECK(reg.status(r) == discover::Status::Alive);
  }
  CHECK(reg.rows[5].busId == reg.rows[6].busId && reg.rows[5].drv == reg.rows[6].drv);   // same address, same type
  CHECK(reg.rows[5].parent != reg.rows[6].parent);                                        // distinct rows, distinct parents
  CHECK(App::reg.status(5) == discover::Status::Alive);
  CHECK(App::reg.status(reg.count) == discover::Status::Gone);                            // total query: past the end reads Gone

  // route is exclusive: enter a channel, enter the other, back to the root
  App::route(2); CHECK(mock::Bus::mask == 1);
  App::route(3); CHECK(mock::Bus::mask == 2);
  App::route(0); CHECK(mock::Bus::mask == 0);

  // pump: every sample reaches exactly the consumers of its capability, tagged with its row
  clearLogs();
  App::pump();
  CHECK(TempLogger::n == 3);
  CHECK(TempLogger::log[0].row == 4 && TempLogger::log[0].v == 187);
  CHECK(TempLogger::log[1].row == 5 && TempLogger::log[1].v == 215);
  CHECK(TempLogger::log[2].row == 6 && TempLogger::log[2].v == 253);
  CHECK(ValuePrinter::n == 4);
  CHECK(ValuePrinter::log[0].cap == 1 && ValuePrinter::log[0].row == 4 && ValuePrinter::log[0].v == 187);
  CHECK(ValuePrinter::log[1].cap == 2 && ValuePrinter::log[1].row == 4 && ValuePrinter::log[1].v == 64);
  CHECK(ValuePrinter::log[2].cap == 1 && ValuePrinter::log[2].row == 5 && ValuePrinter::log[2].v == 215);
  CHECK(ValuePrinter::log[3].cap == 1 && ValuePrinter::log[3].row == 6 && ValuePrinter::log[3].v == 253);
  CHECK(mock::Bus::contention == 0);

  // change one physical device; only its row's sample changes (routing really reaches ch1, not ch0)
  mock::Bus::poke(1, 0x48, 2, 100);
  clearLogs();
  App::pump();
  CHECK(TempLogger::n == 3 && TempLogger::log[1].row == 5 && TempLogger::log[1].v == 215);
  CHECK(TempLogger::log[2].row == 6 && TempLogger::log[2].v == 100);
  CHECK(mock::Bus::contention == 0);
  mock::Bus::poke(1, 0x48, 2, 0xFD);

  // capacity: a 4-row registry stops cleanly, counts what it dropped, never writes past the table
  {
    using Small = AppN<4>;
    mock::Bus::mask = 2;
    Small::discover();
    CHECK(Small::reg.count == 4);
    CHECK(Small::reg.overflow > 0);    // refused adds (a root device re-sighted from a channel scan counts again)
    CHECK(Small::reg.rows[3].isBus && Small::reg.rows[3].parent == 1);
  }

  // fresh discovery + one pump, the same sequence the AVR image runs
  App::discover(); clearLogs(); App::pump();
  std::printf("checksum 0x%04X\n", checksum());
  std::printf(failures ? "FAILED (%d)\n" : "OK: discoverCompose R1 native\n", failures);
  return failures != 0;
}
#endif
