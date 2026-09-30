// The test machine: an axis, two lights on the PCA9685 that discovery finds at 0x40 behind channel 2 of the mux at 0x70 (another PCA9685
// answers at 0x40 behind channel 3: the role must never reach it), a UV light pinned by path to the PCA9685 at 0x41 on the root bus,
// a pump, two references.
// Built in variants that stand for other firmware of the same machine (the consumer is the same Python in every case):
//   WIRING_B     the lights are rewired to another PCA9685, on the root bus: the consumer's view (hashes) must not change
//   FIRMWARE_V2  a role is added (fan): the command schema changes, a consumer must re-read and keep its values by role name
//   FIRMWARE_V3  the blue light is removed: a consumer that drives it must be told, not retargeted
#pragma once
#include <oneMachine/role/role.h>
#include <oneMachine/role/kinds.h>
#include <oneMachine/role/ref.h>
#include <oneMachine/role/found.h>
#include <oneMachine/role/route.h>
#include <oneMachine/role/sim.h>

using Sim = role::SimI2c<4, 4>;
struct SimMux { };                                         // driver types as discovery would bind them
struct SimRtc { };
struct SimWorld {                                          // the parts of a discover::World a role endpoint uses: reg, route
  static inline discover::Registry<uint8_t, 16> reg;
  static void route(discover::RowId bus) {                 // select the bridge channel of `bus` (a root bus needs none)
    const auto& b = reg.rows[bus];
    if (bus != discover::rootRow && b.isBus) Sim::select(uint8_t(b.busId));
  }
};

struct X     { ONEMACHINE_STATE_NAME(name, "x"); };
struct White { ONEMACHINE_STATE_NAME(name, "white"); };
struct Blue  { ONEMACHINE_STATE_NAME(name, "blue"); };
struct Pump  { ONEMACHINE_STATE_NAME(name, "pump"); };
struct Fan   { ONEMACHINE_STATE_NAME(name, "fan"); };
struct Uv    { ONEMACHINE_STATE_NAME(name, "uv"); };
struct Doc   { ONEMACHINE_STATE_NAME(name, "https://github.com/InternetOfPins/OneMachine/blob/main/docs/role.md"); };
struct UserRef { static inline char text[64] = ""; static const char* ref() { return text; } };

#ifdef WIRING_B
using WhiteAt = role::Found<SimWorld, Sim::Pca, 0x41, discover::Anywhere, 7>;         // rewired to the PCA9685 on the root bus
using BlueAt  = role::Found<SimWorld, Sim::Pca, 0x41, discover::Anywhere, 8>;
#else
using WhiteAt = role::Found<SimWorld, Sim::Pca, 0x40, discover::Behind<0x70, 2>, 0>;
using BlueAt  = role::Found<SimWorld, Sim::Pca, 0x40, discover::Behind<0x70, 2>, 1>;
#endif
using UvAt    = role::Pinned<SimWorld, role::path(0x41), Sim::Pca, 15>;
using XAt     = role::SimStepDir<0, 400>;

using M = role::Machine<
  role::Ref<Doc>, role::RefFrom<UserRef>,
  role::Role<X,     role::Axis<80, 0, 300000>, XAt>,
  role::Role<White, role::Light<4000>, WhiteAt>,
#ifndef FIRMWARE_V3
  role::Role<Blue,  role::Light<4095, 100>, BlueAt>,
#endif
#ifdef FIRMWARE_V2
  role::Role<Fan,   role::Switch<true>, role::SimPin<8>>,
#endif
  role::Role<Uv,    role::Light<4095>, UvAt>,
  role::Role<Pump,  role::Switch<false>, role::SimPin<7>>>;

// the bus as it is wired: ours at 0x40 behind channel 2, a decoy at 0x40 behind channel 3, a PCA9685 at 0x41 and an RTC on the root bus
inline void simWire() {
  auto put = [](unsigned i, uint8_t mux, uint8_t addr) { Sim::dev[i] = {}; Sim::dev[i].mux = mux; Sim::dev[i].addr = addr; Sim::dev[i].present = true; };
  put(0, 2, 0x40); put(1, Sim::direct, 0x41); put(2, Sim::direct, 0x68); put(3, 3, 0x40);
}
// what discover() leaves for the sim bus, row by row, with the binder call DriverBase::found() makes for each device; then the Pinned roles
inline void simDiscover() {
  auto& reg = SimWorld::reg;
  reg.reset(); M::unbind();
  for (auto& d : Sim::dev) if (d.present && d.mux == Sim::direct) {
    discover::RowId r = reg.add(d.addr, nullptr, discover::rootRow, false);
    if (d.addr == 0x68) M::bind(r, (SimRtc*)nullptr); else M::bind(r, (Sim::Pca*)nullptr);
  }
  discover::RowId bridge = reg.add(0x70, nullptr, discover::rootRow, false);
  M::bind(bridge, (SimMux*)nullptr);
  discover::RowId ch0 = reg.count;
  for (uint8_t c = 0; c < 4; c++) reg.add(c, nullptr, bridge, true);
  for (uint8_t c = 0; c < 4; c++)
    for (auto& d : Sim::dev) if (d.present && d.mux == c) M::bind(reg.add(d.addr, nullptr, discover::RowId(ch0 + c), false), (Sim::Pca*)nullptr);
  M::pin();
}
