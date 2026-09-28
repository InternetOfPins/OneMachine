# <img src="logo.png" alt="OneMachine logo" width="32" height="32"> OneMachine

Runtime device discovery and failure handling for [HAPI](https://github.com/InternetOfPins/HAPI): scan a bus once, get a
compile-time-composed table of rows back — one indirect call in the whole image, no per-device virtual dispatch, no
dynamic allocation. Failure edges (retry, recover, reprobe) and a health monitor (flap rate, bus cost, quarantine,
disconnect) compose over those rows the same way, at zero cost when not chosen.

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

## Status

Proven on real hardware (an ATmega328P, a real I2C bus, a real sensor and a real display): discovery, the failure
edges' recover/reprobe/re-init paths, and the health monitor's report → quarantine → disconnect sequence, including a
device it disconnected coming back through recovery on its own. Native and AVR builds are both part of every check
this library carries forward from its own development (mutation-tested, checksum-verified against a simulated
ATmega328 where the round called for it).

**Examples are not written yet** — this is the library material as it moved out of its own development history;
worked examples (a real I2C sensor, a health-monitored bus) are the next thing this repository gets.

## License

MIT.
