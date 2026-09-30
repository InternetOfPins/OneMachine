# Roles: what a machine's outputs are for

`state::describe` tells a peer the *structure* of a composition: layers, fields, types. It does not say what an output *means*:
that PCA9685 channel 3 is the white light, or that stepper 1 is the X axis. Three facts are involved, and discovery can find only
the first:

| Fact | Example | Discoverable |
|---|---|---|
| Resource | a PCA9685 at 0x40 behind mux channel 2, 16 outputs | yes, `discover::` |
| Wiring | its output 0 drives the white LED string | no: it is copper |
| Role | "white", a light, at most 4000 | no: it is the machine's design |

`role::` is the machine's own declaration of wiring and role. A consumer (a Python script, a supervisor, another machine) uses
role names and nothing else. The device routes every request itself, so a consumer never addresses a device, never reconfigures
when the wiring moves, and cannot drive a device that is not the one the role means.

## Components

| Header | What it gives |
|---|---|
| `role/role.h` | `Role<Tag, Kind, Endpoint>`; `Machine<Items...>` with `Command` and `Report` (state compositions, one layer per role; the report adds `live`), `apply`, `sense`, `safe`, `pin`, and the discover binder hooks (`wants`, `bind`, `unbind`, `release`) |
| `role/kinds.h` | `Switch<Safe>`, `Light<Max, Safe>`, `Axis<StepsPerMm, MinUm, MaxUm>`: fields, units, parameters, clamping, the safe command |
| `role/found.h` | `Found<W, Driver, Addr, Where, Unit>`: bound when discovery finds its device (the default for a discovered device) |
| `role/route.h` | `path(...)`, `pathOf`, `findPath`: a device location as one number; `Pinned<W, Path, Driver, Unit>`, an endpoint found by path |
| `role/ref.h` | `Ref<Text>` (fixed) and `RefFrom<Src>` (run time): a `ref` line pointing at a fuller description |
| `role/face.h` | `role::describe<M>(put)`: the machine description |
| `role/link.h` | `Link<M, Out, App>`: descriptions, report frames and command frames over any byte stream |
| `role/sim.h` | simulated endpoints and a simulated I2C bus with a mux, to run a machine on a host |
| `python/onemachine` | the consumer side: `Machine`, `Schema`, `StreamLink` |

A kind alone is not the meaning: two lights are both `light`. The role name and the kind's parameters complete it, and the
`ref` lines point at whatever fuller description the machine's author keeps (a document, a drawing, a robot description). What
a reference points to is the describer's concern; the machine only carries the pointer, fixed in the firmware or given at run
time.

## Binding: at discovery, by type

```cpp
using WhiteAt = role::Found<App, Pca9685, 0x40, discover::Behind<0x70, 2>, 0>;   // driver, address, bus, channel
using Roles = role::Machine<
  role::Ref<Doc>,
  role::Role<White, role::Light<4000>, WhiteAt>,
  role::Role<Blue,  role::Light<4095, 100>, role::Found<App, Pca9685, 0x40, discover::Behind<0x70, 2>, 1>>>;

struct App : World<...> {
  template<class Impl> static void bind(RowId r) { Roles::bind(r, (Impl*)nullptr); }   // DriverBase::found() calls this
  static void unbindAll() { Roles::unbind(); }                                          // with lifecycle: a rescan starts over
  static void release(RowId r) { Roles::release(r); }
};
```

The device's identity is its driver type, its address and discover's own `Where` (the same `Behind<Bridge, Ch>` a `Use<>`
entry can name), all types. `DriverBase::found()` already calls `W::bind<Impl>(row)`; the roles are one more binder, like
`discover::Shell`. Only the roles whose driver is `Impl` are compiled into `Impl`'s `found()`: no search and no path to parse.
Roles on one device share one binding (2 bytes). A PCA9685 answering at 0x40 behind another channel is never bound, and a role
whose device is gone reports `live = false` and is not applied; nothing is retargeted.

`Pinned` finds its device by path after a scan (`Roles::pin()`). It is for a location that is data rather than a type: a
run-time role profile, a path received from another machine.

## The consumer's contract

The consumer reads three descriptions once (`m`, `c`, `r` on the link): the machine description, and the command and report
state descriptions. The command and report hashes cover the role names and the kinds' fields; the machine hash covers roles,
kinds, parameters and references. The `at` lines (where each role is) are shown to people and are in no hash. The start of
the test machine's description (`test/role/machine.h`):

```
machine 1
hash a9c67c1d
ref https://github.com/InternetOfPins/OneMachine/blob/main/docs/role.md
role white light
param white max 4000
param white safe 0
at white pca9685 0x40 behind 0x70/2 #0
```

So rewiring, a device moving to another channel, or new firmware with the same roles changes nothing on the consumer side: the
same command frame is accepted as is (`test/role/check.py`, "rewired firmware"). When the roles do change, the device answers
BadHash; the consumer re-reads, keeps every value whose role and field still exist, and is told (RoleChanged) when a role it
drives is gone or has another kind.

```python
from onemachine import Machine, StreamLink
m = Machine(StreamLink.popen(['./device']))     # or a serial port's read/write
m.cmd.white.level = 3000
m.push()                                        # applied at the device's next cycle boundary
print(m.poll().white.level, m.roles['white'].params, m.where['white'])
```

A supervisor that goes quiet (no accepted command for the link's quiet time, a `fail::Deadline`) gets every role's safe command:
lights to their safe level, switches to their safe state, axes hold where they are.

## Cost, measured

ATmega328P, avr-g++ 7.3 -Os, simavr at 16 MHz, six lights on a PCA9685 behind a mux channel, the I2C transfer a stub
(`test/role/avr_cost.cpp`, printed by `test/role/build.sh`):

| Endpoint | Flash | RAM | One apply of 6 roles | After a scan |
|---|---|---|---|---|
| Fixed at compile time (the floor) | 1452 B | 161 B | 633 cycles | 642 cycles |
| `Found` | 1786 B | 163 B | 1029 cycles | 1581 cycles |
| `Pinned` | 2590 B | 173 B | 1125 cycles | 8168 cycles |
| `Found` + `Link` + the three descriptions | 6994 B | 223 B | | |

`Found` costs 66 cycles per role per request over a fixed endpoint, about 4 us, against about 135 us for the PCA9685 write
itself at 400 kHz. The text descriptions are most of the link's flash; they are read once per consumer.

## Stages

1. Roles fixed in the firmware (this module): zero cost when absent, rules checked at compile time.
2. A run-time role profile (not built): the same table stored in EEPROM through the planned `Store`, located by `Pinned` paths,
   with its own hash joining the machine hash.

## Open

- `discover::World::route` walks the table on every call; a one-byte cache of the last routed bus would make per-request
  routing through a real World as cheap as the simulated one here.
- Binary descriptions fetched by hash (the text is the fallback), to cut the link's flash on small targets.
- A second identity for devices that carry a unique id of their own (a 1-Wire ROM, a serial), compared at `found()`.
- Units beyond the kinds' fixed ones, and more kinds (heater, fan, valve) as machines need them.
- Tested against a simulated bus and the registry rows discovery builds, not yet inside a real `discover::World` on hardware.
