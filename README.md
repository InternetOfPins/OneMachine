# OneMachine

Runtime device discovery, failure handling, and ROS-shaped composition for [HAPI](https://github.com/InternetOfPins/HAPI):
scan a bus once, get a compile-time-composed table of rows back — one indirect call in `pump()`, no per-device virtual
dispatch, no dynamic allocation. Failure edges (retry, recover, reprobe) and a health monitor (flap rate, bus cost,
quarantine, disconnect) compose over those rows the same way, at zero cost when not chosen.

Part of the [InternetOfPins](https://github.com/InternetOfPins) project family. Built on
[HAPI](https://github.com/InternetOfPins/HAPI) (the composition core) and [OneBus](https://github.com/InternetOfPins/OneBus)
(the I2C master a real scan talks over).

## Why

A runtime bus (I2C, mostly) can hold a device set nobody knows until the board boots: which sensor, on which mux
channel, at which address. That's inherently a runtime fact. What doesn't have to be runtime is everything after it —
which fan-out a sample reaches, which failure policy a device gets, how a health monitor is wired in. OneMachine scans
once into a small fixed-capacity table, then dispatches through it with exactly one indirect call per poll
(`World::pump()`); everything else — the entry matching, the failure layers, the capability fan-out — is ordinary
static composition, provably free when unused ([temp.alias], the same guarantee HAPI's own `Chain`/`Part` core rests on).

Measured, not just argued (an ATmega328P, `avr-g++ -Os`, real builds in `examples/`, checked by `test/`'s own
baselines on every change):

| Adding...                          | Flash    | RAM      |
|-------------------------------------|----------|----------|
| a failure edge (`DevEdge`+`BusEdge`) | +2848 B  | +49 B    |
| a health monitor (`HealthT`)         | +1532 B  | +110 B   |

Indirect calls, counted on the compiled image, not assumed: exactly one, inside `pump()` itself (`IDriver::poll()`'s
dispatch to the row's real driver). The only other indirect jumps in a full Arduino firmware are the framework's own
-- `Print::write`'s virtual dispatch and avr-libc's `__tablejump2__` -- neither reachable from OneMachine's code.

"Zero cost when not chosen" is the same claim `test/`'s NEG_* mutation tests and `test/baselines/` check on every
change, not just here: every optional layer, trait, and unaccepted capability compiles to nothing when absent,
verified by comparing sections and symbol sets, not by inspection.

## Discovery: `discover::`

An app lists **entries** — a bare driver, or one of three wrappers — and gets a `World<>` that scans a bus into rows:

```cpp
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>

using namespace discover;

// Use<Probe, Driver>: a row for Driver at the addresses Probe accepts. Use<Own, D> is the common case: D's own
// addrLo..addrHi, its idReg (0 if it declares none) checked against D::id, in two stages (an ACK, then the register).
using Entries = Chain<Use<Own, Mpu6050<App>>>;

struct App : World<App, Twi, Chain<MyConsumer>, Entries, /*rows*/ 4, I2cScan> { ... };

App::discover();   // scans 0x08..0x77 once; App::reg now holds the real topology
App::pump();       // one poll per alive row, one indirect call total
```

- `Ignore<Lo,Hi>` claims a range with no row — an address the app never wants to see. `IgnoreBridge<Bridge>` is the
  variant for a *real* mux the app doesn't use: it clears the bridge once claimed, so a channel selection left over
  from before this boot can't leak devices behind it onto the parent bus.
- `Protect<Lo,Hi>` is a rule, not an entry: no entry that writes to the device may cover the range (a compile error
  if one does).
- A driver's `Produces` (a `Chain` of capability tags) decides whether `pump()` calls its `read()` at all; a driver
  that produces nothing (a display) opts in with `static constexpr bool polled = true`.
- A capability fan-out is a plain static consumer list: a driver `emit`s a `Sample<Cap>`, every consumer whose
  `Accepts` names `Cap` gets it, at compile time.

## Failure handling: `fail::`

A driver that also derives from `DevEdge<Impl, W, Mode, K>` gets a failure controller: `Mode` picks the layers
(`Retry`, `Recover`, `Reprobe`, `Backoff`, `Gate`, ...), composed the same way HAPI composes anything else — the
layer list *is* the policy, nothing to configure at runtime.

```cpp
#include <oneMachine/fail/busedge.h>
#include <oneMachine/fail/devedge.h>

using namespace fail;

struct Mode {
  using Twi = MyTwi;
  static constexpr bool checked = true, returnPath = false, idempotent = true, lifecycle = true;
  template<typename E> using BusStack = Controller<E, TickPart<Retry<0>>, Recover, DetectError, HoldOp<Coalesce>, Backoff<100, 400>, Status>;
  template<typename E> using DevStack = Controller<E, TickPart<Retry<2>>, Recover, DetectError, HoldOp<Coalesce>, Gate<50>, TickPart<Reprobe<500, 120>>, LazyStatus>;
};

struct MyDriver : DriverBase<MyDriver<App>, App>, DevEdge<MyDriver<App>, App, Mode, 1> { ... };
```

A bus-level fault (a timeout, an arbitration loss) is attributed to the *bus* row, not the device: "a bus that returns
touches nothing" — a device that comes back with its bus is left alone; a device that came back **on its own** is
re-initialised (it may have lost state). A driver that wants a say in a bus's own return declares `recheck(row)` (force
its own check now) or `reinitOnBusReturn = true` (its init is safe to repeat) — optional, zero cost if not declared.

## Health monitoring: `fail::HealthT`

One monitor, composed once, watches every row's own status changes (never a bus fault following through to its
devices — that attribution is reused, not re-derived) and applies a policy: **Report** always; **Quarantine** a row
that flaps past a threshold (`pump()` and the failure edge's own ticking both stop touching it, a growing — Fibonacci
— block, then a trickle of probes to test recovery); **Disconnect** one that costs too much bus time (calls the
driver's own `isolate(row)`); **Escalate** instead, for a `required` row, to an app-declared hook.

```cpp
#include <oneMachine/fail/health.h>

struct App : World<...>, BusEdge<...> {
  using Health = HealthT<App, Drivers, /*rows*/ 4>;
  static void onEscalate(RowId row, EscalateReason) { /* a required row's condition, routed onward */ }
};

// in the app's own loop:
App::Health::onEdge();     // every tick: catches a flap that starts and clears inside one control period
App::Health::onTick(now);  // the monitor's own, coarser period: decay, cost sampling, policy
```

A driver opts a row in to isolation with `static constexpr bool mayIsolate = true;` and `static void isolate(RowId)`
(cut its own supply, disable its channel — whatever "off" means for that device); `static constexpr bool required = true;`
means it is never quarantined or disconnected, only escalated. Neither declaration costs anything when absent.

## ROS-shaped composition: `rosCompose::`

The same discovered rows can feed a pub/sub, request/response, and long-running-goal shape that mirrors `rclcpp`'s
own structure (a service is two correlated topic hops; an action is three services, two topics, and a pure
state-transition function over `rcl_action`'s own goal states) — without a DDS stack, a middleware thread, or
`std::function` anywhere.

```cpp
#include <oneMachine/rosCompose/transport.h>

using namespace rosCompose;

// a local, compile-time fan-out: every subscriber's on(msg) runs, no vtable, no container
struct LogIt { template<typename T> struct Part : T { void on(const Odom& m) { Serial.println(m.x); } }; };
using Topic = LocalFanout<Odom, Subscriber<Odom, LogIt>>;
```

Everything crossing a real process boundary — another node, discovered at runtime, reached over a network hop —
goes through one seam, `Cap<Msg>`: the only virtual call in the whole module, same shape as `discover::`'s own one
indirect call per poll. `qos.h`'s `WithHistory`/`WithDeadline` fold onto a `Subscriber` the same way any HAPI
decorator does; `service.h`'s `Client`/`Service` correlate a request/response pair over a fixed-capacity table (no
heap); `action.h`'s `ActionServer` tracks goals through `GoalState`/`GoalEvent` the same way.

## Examples

Four stages, each adding one thing to the last, all built around one real sensor (a GY-521/MPU6050 on a Nano):

- [`examples/discover`](examples/discover) — find the device by its own identity, print the table.
- [`examples/mpu6050`](examples/mpu6050) — read its real capabilities, print them as they arrive.
- [`examples/recover`](examples/recover) — a failure edge: transient faults retried, re-probed, re-initialised.
- [`examples/health`](examples/health) — a health monitor: correct fault attribution and flap tracking by hand;
  quarantine and disconnect themselves are proven by the test suite, not the hand demo (see its own README).

`test/examples/build.sh` builds all four for the Nano on every change, so they can't silently rot as the library
evolves.

## Status

Proven on real hardware (an ATmega328P, a real I2C bus, a real GY-521/MPU6050), in the shipped examples themselves:
discovery, and the failure edges' recover/reprobe/re-init paths, including a device coming back through recovery on
its own. The health monitor's full report → quarantine → disconnect sequence is proven by the test suite
(`test/fail/build_f7.sh`, mutation-tested, simavr-verified), not by the hand demo; `examples/health` itself
hardware-verifies the piece a hand test can actually reach -- correct fault attribution and flap tracking. Native
and AVR builds are both part of every check this library carries forward from its own development.

## License

MIT.
