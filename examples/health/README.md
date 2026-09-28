# health

`recover` adds a health monitor: a row that flaps too often, or costs too much bus time retrying, is quarantined
(left alone, tried again on a growing schedule) and then disconnected -- its own supply cut -- instead of retried
forever.

## Wiring

Arduino Nano (ATmega328P). A GY-521 (MPU6050) breakout: SDA to A4, SCL to A5, GND to GND, **VCC to pin 8** (not
5V directly -- this is what the monitor's Disconnect action switches). Change `MPU_VCC_PIN` in `mpu6050.h` if pin 8
is inconvenient to wire.

## Build and flash

```
pio run -e nano -t upload
pio device monitor -b 115200
```

## Try it

Wiggling SDA or SCL exercises the same recovery path `recover` does -- the STATUS line flips and the samples come
back real, thanks to `reinitOnBusReturn` -- but on this wiring (nothing pulls the bus up except the module's own
resistors) it will **not** reach Quarantine or Disconnect: pulling either wire takes the whole bus down, and the
monitor deliberately does not count a bus-inherited Stale as the device's own flap (confirmed on real hardware,
not assumed -- see `HANDOFF.md`-style history in the library's own `test/`). That distinction is the point: a
device that goes down with its bus is treated as a bus problem, not a reason to isolate the device.

The canary is what you can actually see here: unplug and quickly replug the module's own VCC wire (pin 8) a few
times in under a second -- fast enough that a hand can just about do it, or use a jumper you can tap repeatedly.
Watch for the samples staying real instead of drifting to zero; that's `retryExtra`/`recoverMask` (Corrupt) forcing
a re-init the moment the sleep bit is caught, the same mechanism `recover`'s `reinitOnBusReturn` uses for the
bus-wide case.

Quarantine and Disconnect themselves are proven where they can be isolated cleanly: the library's own
`test/fail/build_f7.sh` (mutation-tested, simavr-verified) and, on real hardware with pull-ups that don't depend
on the sensor's own power, this same monitor composition disconnecting a genuinely flapping device (see the
library's development history). Reaching that state by hand on this minimal reference wiring specifically isn't
straightforward -- a real, disclosed limitation of this wiring, not of the monitor.

## What this proves

Quarantine and Disconnect are declared, not wired by hand: `mayIsolate = true` and an `isolate(RowId)` method are
the only two things the driver adds over `recover`'s; the monitor (`fail::HealthT`) decides when to call it from
the row's own flap-rate and cost history, tracked outside the driver entirely. And a bus fault is not a device
flap: confirmed here by the fact that wiggling the shared bus wires, no matter how much, never isolates the
device -- only a fault that is really the device's own does.
