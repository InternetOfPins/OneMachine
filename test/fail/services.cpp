// fail::Services<W, Drivers>: the loop's step over every row of the drivers that serve (`static constexpr bool serves`, service(row, now)).
//   - each row of a driver that serves is called, with the loop's time; the rows of other drivers are not
//   - a driver that does not serve costs nothing: it has no service(), and the fold does not look at its rows
// Native only.
#include <stdint.h>
#include <cstdio>
#include <hapi/hapi.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include <oneMachine/fail/world.h>
#include "../support/mockTwi.h"

using discover::RowId;
using hapi::Chain;

template<typename W> struct Svc : discover::DriverBase<Svc<W>, W> {          // SensorB of the mock bus, at 0x40
  static constexpr uint8_t addrLo = 0x40, addrHi = 0x40, idReg = 0, id = 0xB2;
  static constexpr bool serves = true;
  inline static uint32_t calls = 0, lastNow = 0;
  inline static RowId rows[8]; inline static uint8_t n = 0;
  static void service(RowId row, uint32_t now) { if (n < 8) rows[n++] = row; lastNow = now; ++calls; }
};
template<typename W> struct Quiet : discover::DriverBase<Quiet<W>, W> {      // a driver with no interrupt part: no service()
  static constexpr uint8_t addrLo = 0x50, addrHi = 0x50, idReg = 0, id = 0x01;
};

struct App;
using S = Svc<App>;
using Q = Quiet<App>;
struct App : discover::World<App, mock::Twi, Chain<>, Chain<S, Q>, 4, discover::I2cScan> {};
using Services = fail::Services<App, App::DriverList>;

static int failures = 0;
#define CHECK(...) do { if (!(__VA_ARGS__)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #__VA_ARGS__); } } while (0)

int main() {
  static_assert(fail::Serves<S>::value && !fail::Serves<Q>::value, "Serves tells a driver that serves from one that does not");
  mock::Bus::mask = 0;
  App::discover();
  CHECK(App::reg.count == 2);                                    // the root, and SensorB at 0x40
  Services::run(1234);
  CHECK(S::calls == 1 && S::rows[0] == 1 && S::lastNow == 1234); // its row, with the time
  Services::run(1300);
  CHECK(S::calls == 2 && S::lastNow == 1300);
  App::reg.add(0x41, discover::instOf<S>(), discover::rootRow, false);   // a second row of the same driver
  Services::run(1400);
  CHECK(S::calls == 4 && S::rows[2] == 1 && S::rows[3] == 2);    // both rows, in row order
  const uint32_t before = S::calls;
  fail::Services<App, Chain<Q>>::run(1500);                      // a driver that does not serve: nothing is called, nothing is built for it
  CHECK(S::calls == before);
  if (failures) { std::printf("FAILED: %d\n", failures); return 1; }
  std::printf("OK: fail::Services native\n");
  return 0;
}
