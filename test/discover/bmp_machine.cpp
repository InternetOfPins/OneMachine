// The BMP280 as a static machine of OneMenu ItemDef nodes (examples/spi/src/bmp280_machine.h) against a simulated chip (test/support/mockBmpTwi.h)
// that holds the datasheet's worked example: 25.08 C and 100653.27 Pa.
//   - discovery finds it, init reads the calibration and writes the registers' defaults, config first
//   - one poll gives the datasheet's values; a register mimic reads and writes the real register
//   - a published node calls its function once per change, not on a repeat, and the inner node takes its copy (it is referred to, not copied)
//   - the description walk writes the codes, paths, fields and which ones notify
// Native only.
#include <stdint.h>
#include <cstdio>
#include <cstring>
#include <hapi/hapi.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include "../support/mockBmpTwi.h"
#include "../../examples/spi/src/bmp280_machine.h"

using discover::RowId;
using hapi::Chain;

struct App;
using M = bmpm::Machine<App>;
struct App : discover::World<App, mockbmp::Twi, Chain<>, M::Entries, 3, discover::I2cScan> {};

struct TagCode { };
struct CodeTemp  { ONEMACHINE_STATE_NAME(name, "temp"); };
struct CodePress { ONEMACHINE_STATE_NAME(name, "press"); };
struct CodeAir   { ONEMACHINE_STATE_NAME(name, "air"); };

static int32_t gotT[8], gotP[8]; static int nT = 0, nP = 0;
static void onTemp(int32_t v)  { if (nT < 8) gotT[nT] = v; ++nT; }
static void onPress(int32_t v) { if (nP < 8) gotP[nP] = v; ++nP; }

using PubTemp  = bmpm::Published<CodeTemp,  M::Temp,  M::temp,  oneData::OnSync<&onTemp>>;
using PubPress = bmpm::Published<CodePress, M::Press, M::press, oneData::OnSync<&onPress>>;
using PubAir   = bmpm::Published<CodeAir,   M::Ctrl,  M::ctrl>;
using Pubs = Chain<PubTemp, PubPress, PubAir>;

static int failures = 0;
#define CHECK(...) do { if (!(__VA_ARGS__)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #__VA_ARGS__); } } while (0)

struct Str { char b[1024]; unsigned n = 0; void operator()(char c) { if (n < sizeof(b) - 1) b[n++] = c; b[n] = 0; } };

int main() {
  using mockbmp::State;
  State::reset();
  App::discover();
  CHECK(App::reg.count == 2);
  CHECK(M::Dev::addr == 0x76);
  CHECK(M::Dev::cal.T1 == 27504 && M::Dev::cal.P9 == 6000 && M::Dev::cal.T3 == -1000);                    // the calibration read from the chip
  CHECK(State::regs[0xF5] == 0x90 && State::regs[0xF4] == 0x57);                                         // the defaults are the init
  CHECK(State::nlog == 2 && State::log[0] == 0xF5 && State::log[1] == 0xF4);                             // config first, then ctrl_meas
  CHECK(State::resets == 1);

  // ---- one poll: the datasheet's values --------------------------------------------------------------------
  App::pump();
  CHECK(M::Dev::temp == 2508);
  CHECK(M::Dev::press == 100653);

  // ---- publishing: once per change ------------------------------------------------------------------------------
  CHECK(M::temp.changed() && M::press.changed());                                                        // Watch starts from 0
  bmpm::PublishAll<Pubs>::sync();
  CHECK(nT == 1 && gotT[0] == 2508 && nP == 1 && gotP[0] == 100653);
  CHECK(!M::temp.changed() && !M::press.changed());                                                      // the inner node took its copy
  bmpm::PublishAll<Pubs>::sync();
  CHECK(nT == 1 && nP == 1);                                                                             // nothing moved: nothing said
  App::pump();                                                                                           // the same sample again
  bmpm::PublishAll<Pubs>::sync();
  CHECK(nT == 1 && nP == 1);
  State::setSample(415148, 519888 + 1600);                                                               // warmer
  App::pump();
  bmpm::PublishAll<Pubs>::sync();
  CHECK(nT == 2 && gotT[1] > gotT[0] && nP == 2);                                                        // temperature moved; pressure moves with it (compensated against it)

  // ---- path access: a node of the machine, then a register of the group; get() and set() hit the real register ----
  uint8_t seen = 0;
  M::visit(3, [&](auto& n) { (void)n; seen = 3; });
  CHECK(seen == 3);
  uint8_t v = 0;
  M::visitReg(1, [&](auto& r) { v = r.get(); });
  CHECK(v == 0x57);
  M::visitReg(1, [&](auto& r) { r.set(0x23); });
  CHECK(State::regs[0xF4] == 0x23);
  M::visitReg(0, [&](auto& r) { v = r.get(); });
  CHECK(v == 0x90);

  // ---- what the nodes are -----------------------------------------------------------------------------------------------
  CHECK(bmpm::NotifiesSync<PubTemp>::value && bmpm::NotifiesSync<PubPress>::value && !bmpm::NotifiesSync<PubAir>::value);
  CHECK(!bmpm::HasSet<M::Temp>::value);                                                                  // read-only
  CHECK(bmpm::HasSet<M::Config>::value);
  CHECK(M::IndexOf<M::Temp, M::Nodes>::value == 0 && M::IndexOf<M::Ctrl, M::Nodes>::value == 3);

  // ---- the description -------------------------------------------------------------------------------------------------
  Str s;
  bmpm::describe<M, Pubs>(s, 1, 0x76);
  static const char want[] =
    "machine bmp280 at 0x76\n"
    "  #0 temp ro value scaled 2\n"
    "  #1 press ro value scaled 2\n"
    "  #2 cal const 0x88 [24]\n"
    "  #3 ctrl group 2\n"
    "    #0 config reg 0xF5 default 0x90 rw\n"
    "    #1 ctrl_meas reg 0xF4 default 0x57 rw\n"
    "published\n"
    "  temp -> 1/118/0 notify sync ro value scaled 2\n"
    "  press -> 1/118/1 notify sync ro value scaled 2\n"
    "  air -> 1/118/3 silent group 2\n";
  if (std::strcmp(s.b, want) != 0) { ++failures; std::printf("FAIL description:\n%s--- wanted:\n%s", s.b, want); }

  if (failures) { std::printf("FAILED: %d\n", failures); return 1; }
  std::printf("OK: BMP280 machine native\n");
  return 0;
}
