// The example's machine: what its outputs are for. The same roles on the Nano (Arduino pins) and on a host (simulated pins), so
// drive.py is the same script against both. A consumer sees `led` and `lamp`, never a pin number.
#pragma once
#include <oneMachine/role/role.h>
#include <oneMachine/role/kinds.h>
#include <oneMachine/role/ref.h>

struct Led  { ONEMACHINE_STATE_NAME(name, "led"); };
struct Lamp { ONEMACHINE_STATE_NAME(name, "lamp"); };
struct Doc  { ONEMACHINE_STATE_NAME(name, "https://github.com/InternetOfPins/OneMachine/tree/main/examples/python"); };

// LedAt: a Switch endpoint (set/get bool); LampAt: a Light endpoint (set/get uint16_t, top)
template<class LedAt, class LampAt>
using MachineOf = role::Machine<
  role::Ref<Doc>,
  role::Role<Led,  role::Switch<false>, LedAt>,
  role::Role<Lamp, role::Tuned<role::Light<200>>, LampAt>>;   // at most 200 of 255: the lamp's LED is never driven at full duty.
                                                                // Tuned: a consumer may lower its max and safe level at run time,
                                                                // never above 200 (drive.py does)
