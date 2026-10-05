// The BMP280 driver (examples/spi/src/bmp280.h) against a simulated chip that holds the datasheet's worked example (section 8.2): its calibration has
// negative terms, which the pressure compensation shifts. The compensation must give the datasheet's values (25.08 C, 100653.27 Pa), and, under UBSan,
// shift no negative value (undefined before C++20).
// Native only.
#include <stdint.h>
#include <cstdio>
#include <hapi/hapi.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include "../support/mockBmpTwi.h"
#include "../../examples/spi/src/bmp280.h"

using hapi::Chain;

struct Log {
  using Accepts = Chain<bmp::Temp, bmp::Press>;
  inline static int32_t temp = 0, press = 0; inline static int n = 0;
  template<typename Cap> struct Body {
    template<typename T> struct Part : T {
      using T::T;
      void on(const discover::Sample<Cap>& s) {
        if constexpr (std::is_same<Cap, bmp::Temp>::value) temp = s.value; else press = s.value;
        ++n;
      }
    };
  };
};

struct App;
struct App : discover::World<App, mockbmp::Twi, Chain<Log>, bmp::Entries<App>, 3, discover::I2cScan> {};

static int failures = 0;
#define CHECK(...) do { if (!(__VA_ARGS__)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #__VA_ARGS__); } } while (0)

int main() {
  using mockbmp::State;
  State::reset();
  State::c77.unplug();             // one sensor, at 0x76
  App::discover();
  CHECK(App::reg.count == 2);
  CHECK(State::c76.regs[0xF5] == 0x90 && State::c76.regs[0xF4] == 0x57);
  App::pump();
  CHECK(Log::n == 2);
  CHECK(Log::temp == 2508);        // 25.08 C
  CHECK(Log::press == 100653);     // 100653.27 Pa, in Pa
  // other raw values through the same calibration: a colder and a warmer sample, a lower pressure
  State::c76.setSample(415148, 519888 - 8000);
  App::pump();
  CHECK(Log::temp < 2508 && Log::temp > 1500);
  State::c76.setSample(380000, 519888 + 8000);
  App::pump();
  CHECK(Log::temp > 2508 && Log::press > 100653 - 20000 && Log::press != 100653);
  if (failures) { std::printf("FAILED: %d\n", failures); return 1; }
  std::printf("OK: BMP280 compensation native\n");
  return 0;
}
