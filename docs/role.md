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
| `role/kinds.h` | `Switch<Safe>`, `Light<Max, Safe>`, `Axis<StepsPerMm, MinUm, MaxUm>`: fields, units, parameters, clamping, the safe command; `Tuned<Kind>` (in `role.h`) makes a role's parameters changeable at run time |
| `role/found.h` | `Found<W, Driver, Addr, Where, Unit>`: bound when discovery finds its device (the default for a discovered device) |
| `role/route.h` | `path(...)`, `pathOf`, `findPath`: a device location as one number; `Pinned<W, Path, Driver, Unit>`, an endpoint found by path |
| `role/ref.h` | `Ref<Text>` (fixed) and `RefFrom<Src>` (run time): a `ref` line pointing at a fuller description |
| `role/face.h` | `role::describe<M>(put)`: the machine description |
| `role/link.h` | `Link<M, Out, App>`: descriptions, report frames and command frames over any byte stream |
| `role/call.h` | `Call<M, App>`: the link as a function; `ONEMACHINE_CALL_EXPORT` gives it a C ABI (`onemachine_call`, `onemachine_cycle`) |
| `role/sim.h` | simulated endpoints and a simulated I2C bus with a mux, to run a machine on a host |
| `python/onemachine` | the consumer side: `Machine`, `Schema`, `StreamLink` (a byte stream), `CtypesLink` (`onemachine_call` in a shared library) |

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

A consumer that presents a machine in another protocol (SiLA 2, for one) is described in [consumers.md](consumers.md).

```
machine 1
hash a9c67c1d
ref https://github.com/InternetOfPins/OneMachine/blob/main/docs/role.md
role white light
param white max 4000
param white safe 0
at white pca9685 0x40 behind 0x70/2 #0
```

A kind may also print what its values mean (`Params`, in `role/face.h`; all inside the machine hash):

```
value mode 0 off            a value the role takes, with an optional label (v.value(raw, label))
scale vin raw 5 1023        presented = raw * num / den, for a field of the role (v.scale(field, num, den))
unit vin raw V              the unit symbol of a field of the role (v.unit(field, symbol))
```

The device holds integers only; the presented value is the consumer's arithmetic. A label is 1 to 32 of `A-Z a-z 0-9 _ -`
(`python/onemachine` refuses the description otherwise). A unit symbol is one token of `A-Z a-z 0-9 / * ^ . % -`, 1 to 16 characters (the same refusal). Kinds that print none of these lines print exactly what they did before. [`examples/sila`](../examples/sila) is a machine that uses all three, served over SiLA 2.

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

## Changing a parameter at run time: `Tuned`

A role's parameters are its kind's template arguments: `Light<200>` has max 200. Wrapping the kind in `role::Tuned` keeps those
as the firmware's limits and lets a consumer change the values inside them while the machine runs:

```cpp
role::Role<Lamp, role::Tuned<role::Light<200>>, LampAt>     // max and safe changeable, never above 200
```

The machine description marks the role (`tune lamp`, inside the machine hash), and its `param` lines are the limits. The link
gets three ops: `T` (the tuning description, a state composition with one layer per tuned role), `G` (the current values) and
`S` (new values). `S` is refused with BadValue, changing nothing, when any value is outside the firmware's limits. An accepted
one takes effect at once, and the next cycle applies the current command again under it. Values live in RAM, so a reset goes
back to the template arguments.

```python
m = Machine(link)                       # reads the tuning description and current values with the others
m.roles['lamp'].tuned, m.tune.lamp      # True, {'max': 200, 'safe': 0}
m.tune.lamp.max = 120; m.retune()       # the lamp's command is clamped to 120 from the next cycle
m.tune.lamp.max = 255; m.retune()       # OutOfLimits: the device keeps 120, m.tune is read back
```

`examples/python/drive.py` does this against the example's lamp, and `test/role/call_check.cpp` checks the ops through the C ABI.

## In the same process: `onemachine_call`

`role::Call<M, App>` is `role::Link` without the stream: one request in, one response out, the same bytes. It is for a consumer
in the same image or process, such as Python loading a host build, or Rust firmware calling into the C++. The flat C ABI is
the same pattern as HAPI's `rust_stm32_bridge`, and nothing in it is specific to one language:

```cpp
static role::Call<M, App> dev(2000);               // quiet time in ms, as for Link
ONEMACHINE_CALL_EXPORT(onemachine, dev)
// int32_t onemachine_call(uint8_t op, const uint8_t* in, uint16_t n, uint8_t* out, uint16_t cap, uint32_t now_ms);
// void    onemachine_cycle(uint32_t now_ms);
```

`onemachine_call` returns the response length (status, u16 length, payload). If `cap` is too small, it returns minus the length
it needs. Only the description and frame replies are long, and they change nothing, so the consumer asks again with more room.
Every reply that changes something fits in 3 bytes. `onemachine_cycle` is the cycle boundary: it applies an accepted command,
applies the safe command when the consumer has gone quiet, and refreshes the report. Whoever owns the loop calls it. One call
runs at a time; the reply buffer is shared per machine type.

The consumer implements the protocol the same way it does over a serial port. From Python, the only change is the link:

```python
m = Machine(CtypesLink('./libdevice.so'))       # autocycle: runs onemachine_cycle after each call
```

`test/role/call_check.cpp` calls the C ABI directly, as Rust over FFI would. `check.py --ctypes` runs the 39 consumer checks
through it against the same four firmware variants that run over a pipe.

## Cost, measured

ATmega328P, avr-g++ 7.3 -Os, simavr at 16 MHz, six lights on a PCA9685 behind a mux channel, the I2C transfer a stub
(`test/role/avr_cost.cpp`, printed by `test/role/build.sh`):

| Build | Flash | RAM | One apply of 6 roles | After a scan |
|---|---|---|---|---|
| No role layer: the app writes the six channels itself (the floor) | 992 B | 137 B | 319 cycles | 642 cycles |
| Roles, endpoints fixed at compile time | 1466 B | 161 B | 633 cycles | 642 cycles |
| Roles, `Found` (bound at discovery) | 1800 B | 163 B | 1029 cycles | 1581 cycles |
| Roles, `Pinned` (found by path) | 2620 B | 173 B | 1082 cycles | 8168 cycles |
| Roles, `Found`, + `Link` and the three descriptions | 6996 B | 224 B | | |
| Roles, `Found`, all six `Tuned` | 1824 B | 187 B | 1047 cycles | 1581 cycles |
| Roles, `Found`, all six `Tuned`, + `Link` and the four descriptions | 8534 B | 260 B | | |

The balance of roles with `Found` against no role layer: +808 B flash, +26 B RAM, +118 cycles per role per request, about
7 us at 16 MHz, against about 135 us for the PCA9685 write itself at 400 kHz (5 %), and +939 cycles once per scan. Of that,
the role layer itself (clamping, the read-back report, `live`, the command and report state) is +474 B and +52 cycles per role;
binding at discovery and routing is +334 B and +66 cycles per role. The text descriptions are most of the link's flash; they
are read once per consumer.

`Tuned` on all six roles adds 24 B flash, 24 B RAM (each light's max and safe) and 3 cycles per role per request over `Found`.
Its link side (the tuning description, `T`, `G`, `S` and the limit check) adds 1538 B flash and 36 B RAM, again mostly text.
A machine with no tuned role pays nothing for it.

## Stages

1. Roles fixed in the firmware (this module): zero cost when absent, rules checked at compile time.
2. Run-time parameters (`Tuned`, built): a role's parameters change over the link, inside the firmware's limits, in RAM.
3. Persistent parameters (not built): the tuning stored in EEPROM through the planned `Store`, restored at start.
4. A run-time role profile (not built): the role table itself stored through `Store`, located by `Pinned` paths, with its own
   hash joining the machine hash.

## Open

- `discover::World::route` walks the table on every call; a one-byte cache of the last routed bus would make per-request
  routing through a real World as cheap as the simulated one here.
- Binary descriptions fetched by hash (the text is the fallback), to cut the link's flash on small targets.
- A second identity for devices that carry a unique id of their own (a 1-Wire ROM, a serial), compared at `found()`.
- Units beyond the kinds' fixed ones, and more kinds (heater, fan, valve) as machines need them.
- Tested against a simulated bus and the registry rows discovery builds, not yet inside a real `discover::World` on hardware.
