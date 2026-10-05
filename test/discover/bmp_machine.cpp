// The BMP280 as a static machine of OneMenu ItemDef nodes (examples/spi/src/bmp280_machine.h) against simulated chips (test/support/mockBmpTwi.h):
// 0x76 holds the datasheet's worked example (25.08 C, 100653.27 Pa), 0x77 is another part with a calibration of its own.
//   - two machines, Machine<W, Addr<0x76>> and Machine<W, Addr<0x77>>: two types, each with its own statics, both found
//   - discovery finds a chip, init reads the calibration and writes the registers' defaults, config first
//   - one poll gives the datasheet's values; a register mimic reads and writes the real register
//   - a published node calls its function once per change, not on a repeat, and the inner node takes its copy: reached by a compile-time path
//     (PathRef, any node, a leaf of a group too) or by an ItemRef (a standalone object)
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
using M   = bmpm::Machine<App, bmpm::Addr<0x76>>;
using M77 = bmpm::Machine<App, bmpm::Addr<0x77>>;
struct App : discover::World<App, mockbmp::Twi, Chain<>, Chain<M::UseBmp, M::UseBme, M77::UseBmp, M77::UseBme>, 4, discover::I2cScan> {};

struct CodeTemp     { ONEMACHINE_STATE_NAME(name, "temp"); };
struct CodePress    { ONEMACHINE_STATE_NAME(name, "press"); };
struct CodeAir      { ONEMACHINE_STATE_NAME(name, "air"); };
struct CodeCtrlMeas { ONEMACHINE_STATE_NAME(name, "ctrl_meas"); };

static int32_t gotT[8], gotP[8], gotI[8]; static int nT = 0, nP = 0, nI = 0;
static void onTemp(int32_t v)  { if (nT < 8) gotT[nT] = v; ++nT; }
static void onPress(int32_t v) { if (nP < 8) gotP[nP] = v; ++nP; }
static void onTempRef(int32_t v) { if (nI < 8) gotI[nI] = v; ++nI; }

// by path
using PubTemp     = bmpm::PublishedAt<CodeTemp,     bmpm::PathRef<M, 0>,    oneData::OnSync<&onTemp>>;
using PubPress    = bmpm::PublishedAt<CodePress,    bmpm::PathRef<M, 1>,    oneData::OnSync<&onPress>>;
using PubAir      = bmpm::PublishedAt<CodeAir,      bmpm::PathRef<M, 3>>;
using PubCtrlMeas = bmpm::PublishedAt<CodeCtrlMeas, bmpm::PathRef<M, 3, 1>>;      // a leaf inside the group
using Pubs = Chain<PubTemp, PubPress, PubAir, PubCtrlMeas>;
// by an ItemRef to the standalone object
using PubTempRef  = bmpm::Published<CodeTemp, M::Temp, M::temp, oneData::OnSync<&onTempRef>>;

static int failures = 0;
#define CHECK(...) do { if (!(__VA_ARGS__)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #__VA_ARGS__); } } while (0)

struct Str { char b[1024]; unsigned n = 0; void operator()(char c) { if (n < sizeof(b) - 1) b[n++] = c; b[n] = 0; } };

int main() {
  using mockbmp::State;
  State::reset();
  App::discover();
  CHECK(App::reg.count == 3);                                                                             // the root, and one row per machine
  CHECK(M::Dev::addr == 0x76 && M77::Dev::addr == 0x77);                                                  // the Criteria
  CHECK(!std::is_same<M::Dev, M77::Dev>::value);                                                          // two types, two sets of statics
  CHECK(M::Dev::cal.T1 == 27504 && M::Dev::cal.P9 == 6000 && M::Dev::cal.T3 == -1000);                    // the calibration read from the chip
  CHECK(M77::Dev::cal.T2 != M::Dev::cal.T2);                                                             // 0x77 is another part
  CHECK(State::c76.regs[0xF5] == 0x90 && State::c76.regs[0xF4] == 0x57);                                  // the defaults are the init
  CHECK(State::c76.nlog == 2 && State::c76.log[0] == 0xF5 && State::c76.log[1] == 0xF4);                  // config first, then ctrl_meas
  CHECK(State::c76.resets == 1 && State::c77.resets == 1);
  CHECK(State::c77.regs[0xF5] == 0x90 && State::c77.regs[0xF4] == 0x57);

  // ---- one poll: the datasheet's values -------------------------------------------------------------------------
  App::pump();
  CHECK(M::Dev::temp == 2508);
  CHECK(M::Dev::press == 100653);
  CHECK(M77::Dev::temp != M::Dev::temp);                                                                  // its own calibration, its own value
  const int32_t t77 = M77::Dev::temp;
  State::c77.setSample(415148, 519888 + 3200);                                                            // only the other chip moves
  App::pump();
  CHECK(M::Dev::temp == 2508 && M77::Dev::temp > t77);                                                    // the statics are separate

  // ---- publishing by path: once per change -----------------------------------------------------------------------------
  CHECK(M::temp.changed() && M::press.changed());                                                         // Watch starts from 0
  bmpm::PublishAll<Pubs>::sync();
  CHECK(nT == 1 && gotT[0] == 2508 && nP == 1 && gotP[0] == 100653);
  CHECK(!M::temp.changed() && !M::press.changed());                                                       // the inner node took its copy
  bmpm::PublishAll<Pubs>::sync();
  CHECK(nT == 1 && nP == 1);                                                                              // nothing moved: nothing said
  App::pump();
  bmpm::PublishAll<Pubs>::sync();
  CHECK(nT == 1 && nP == 1);
  State::c76.setSample(415148, 519888 + 1600);                                                            // warmer
  App::pump();
  bmpm::PublishAll<Pubs>::sync();
  CHECK(nT == 2 && gotT[1] > gotT[0] && nP == 2);

  // ---- publishing by an ItemRef to the same node: the same behaviour -----------------------------------------------------
  State::c76.setSample(415148, 519888 + 3200);
  App::pump();
  bmpm::PublishAll<Chain<PubTempRef>>::sync();
  CHECK(nI == 1 && gotI[0] > gotT[1]);
  bmpm::PublishAll<Chain<PubTempRef>>::sync();
  CHECK(nI == 1);

  // ---- a leaf of a group published by path: its value is the real register ---------------------------------------------
  CHECK(PubCtrlMeas{}.get() == 0x57);
  CHECK(PubAir::Inner::Body::size() == 2);
  M::resolve<3, 1>().set(0x23);                                                                           // by path, to the leaf
  CHECK(State::c76.regs[0xF4] == 0x23 && PubCtrlMeas{}.get() == 0x23);
  M::resolve<3, 1>().set(0x57);

  // ---- run-time path: a node of the machine, then a register of the group -------------------------------------------------
  uint8_t seen = 0, v = 0;
  M::visit(3, [&](auto& n) { (void)n; seen = 3; });
  CHECK(seen == 3);
  M::visitReg(1, [&](auto& r) { v = r.get(); });
  CHECK(v == 0x57);
  M::visitReg(0, [&](auto& r) { v = r.get(); });
  CHECK(v == 0x90);
  CHECK(&M::resolve<0>() == &M::temp && &M::resolve<3>() == &M::ctrl);

  // ---- what the nodes are -----------------------------------------------------------------------------------------------
  CHECK(bmpm::NotifiesSync<PubTemp>::value && bmpm::NotifiesSync<PubPress>::value && !bmpm::NotifiesSync<PubAir>::value);
  CHECK(!bmpm::HasSet<M::Temp>::value);                                                                   // read-only
  CHECK(bmpm::HasSet<M::Config>::value);
  CHECK(M::IndexOf<M::Temp, M::Nodes>::value == 0 && M::IndexOf<M::Ctrl, M::Nodes>::value == 3);
  CHECK(std::is_same<M::NodeAt<3, 1>::type, M::CtrlMeas>::value && std::is_same<M::NodeAt<0>::type, M::Temp>::value);

  // ---- the description -------------------------------------------------------------------------------------------------
  Str s;
  bmpm::describe<M, Pubs>(s, 1);
  static const char want[] =
    "machine bmp280 at 0x76\n"
    "  #0 temp ro value scaled 2\n"
    "  #1 press ro value scaled 2\n"
    "  #2 cal const 0x88 [24]\n"
    "  #3 ctrl group 2\n"
    "    #0 config reg 0xF5 default 0x90 rw range 0..255\n"
    "    #1 ctrl_meas reg 0xF4 default 0x57 rw range 0..255\n"
    "published\n"
    "  temp -> 1/118/0 notify sync ro value scaled 2 status alive\n"
    "  press -> 1/118/1 notify sync ro value scaled 2 status alive\n"
    "  air -> 1/118/3 silent group 2 status alive\n"
    "  ctrl_meas -> 1/118/3/1 silent reg 0xF4 default 0x57 rw range 0..255 status alive\n";
  if (std::strcmp(s.b, want) != 0) { ++failures; std::printf("FAIL description:\n%s--- wanted:\n%s", s.b, want); }
  Str s2;
  bmpm::describe<M77, Chain<>>(s2, 1);
  CHECK(std::strstr(s2.b, "machine bmp280 at 0x77\n") != nullptr);                                        // the other machine names its own device

  if (failures) { std::printf("FAILED: %d\n", failures); return 1; }
  std::printf("OK: BMP280 machine native\n");
  return 0;
}
