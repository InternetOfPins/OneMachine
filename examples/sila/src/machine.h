// The machine: a switch `led`, a light `lamp`, a discrete `step`, a select `mode`, an analog input `vin`, a text `note`, a scaled `duty` and an action
// `ping`. A consumer sees the roles and their kinds, never the pins: the same roles run on the Nano (pins 13, 9 and A0; the rest have no wiring) and
// on a host (simulated endpoints), so the same gateway serves both.
#pragma once
#include <oneMachine/role/role.h>
#include <oneMachine/role/kinds.h>
#include "kinds.h"

struct Led  { ONEMACHINE_STATE_NAME(name, "led"); };
struct Lamp { ONEMACHINE_STATE_NAME(name, "lamp"); };
struct Step { ONEMACHINE_STATE_NAME(name, "step"); };
struct Ping { ONEMACHINE_STATE_NAME(name, "ping"); };
struct Mode { ONEMACHINE_STATE_NAME(name, "mode"); };
struct Vin  { ONEMACHINE_STATE_NAME(name, "vin"); };

struct LabelOff  { ONEMACHINE_STATE_NAME(name, "off"); };
struct LabelLow  { ONEMACHINE_STATE_NAME(name, "low"); };
struct LabelHigh { ONEMACHINE_STATE_NAME(name, "high"); };
struct Note { ONEMACHINE_STATE_NAME(name, "note"); };
struct Duty { ONEMACHINE_STATE_NAME(name, "duty"); };
struct PercentUnit { ONEMACHINE_STATE_NAME(name, "%"); };
struct VinUnit { ONEMACHINE_STATE_NAME(name, "V"); };

template<class LedAt, class LampAt, class VinAt>
using MachineOf = role::Machine<
  role::Role<Led,  role::Switch<false>, LedAt>,
  role::Role<Lamp, role::Light<200>, LampAt>,
  role::Role<Step, extra::Discrete<0, 10, 50, 100>, extra::Var<0>>,
  role::Role<Mode, extra::Select<LabelOff, LabelLow, LabelHigh>, extra::Var<1>>,
  role::Role<Vin,  extra::Analog<5, 1023, VinUnit>, VinAt>,
  role::Role<Note, extra::Text<16>, extra::Buf<16>>,
  role::Role<Duty, extra::Scaled<1000, 1, 10, PercentUnit>, extra::Var<2>>,
  role::Role<Ping, extra::Action, extra::Counter>>;
