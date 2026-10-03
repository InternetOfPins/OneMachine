# health

`recover` adds a health monitor: a row that flaps too often, or costs too much bus time retrying, is quarantined
(left alone, tried again on a growing schedule) and then disconnected -- its own supply cut -- instead of retried
forever. A quarantine ends only when a probe finds the row answering and quiet; a probe that finds it still bad starts a
longer block.

## Wiring

Arduino Nano (ATmega328P). A GY-521 (MPU6050) breakout: SDA to A4, SCL to A5, GND to GND, **VCC to pin 8** (not
5V directly -- this is what the monitor's Disconnect action switches). Change `MPU_VCC_PIN` in `mpu6050.h` if pin 8
is inconvenient to wire.

With only the module's own pull-up resistors on SDA/SCL, a fault on the device's own wiring and a fault on the
shared bus are the same electrical event -- pulling either wire drags the whole bus down, so every fault lands on
the bus row, never the device's own flap count (confirmed on real hardware). `setup()` also enables the Nano's own
internal pull-ups on A4/A5, on top of the module's: with those, disconnecting SDA or SCL alone reads as that device
failing to answer a bus that's otherwise fine, attributed to the device alone (also confirmed on real hardware --
five clean device-only edges, zero bus-row involvement, across a disconnect/reconnect test). Cutting the module's
own VCC is a different case: it can leave the device's pins driven mid-transaction, which still takes the bus with
it regardless of pull-ups -- use SDA or SCL, not VCC, to see the device-only path below.

## Build and flash

```
pio run -e nano -t upload
pio device monitor -b 115200
```

## Try it

Disconnect SDA (or SCL) a few times, a couple of seconds apart. Expect a `STATUS` line on the device row only --
the bus row's own STATUS should stay quiet -- and each one followed by an `HLTH` line with `flap=` visibly up from
where it was, e.g.:

```
STATUS 9252 row 1 0->1
HLTH 9751 row 1 flap=28 cost=1 quarantined=0 disconnected=0
STATUS 12252 row 1 1->0
HLTH 14752 row 1 flap=5 cost=0 quarantined=0 disconnected=0
```

That's the monitor correctly charging the row for a fault that really is its own, and decaying it back down between
edges the same way -- both real, both worth seeing. What you won't see by hand: `quarantined=1`. `flapEwma` decays
about 12.5% every 500ms it isn't pushed, and each push only moves it part-way to its ceiling, so crossing the
monitor's threshold needs edges roughly a second apart -- faster than this row's own `Reprobe<500,120>` schedule
recognizes a recovery and fails again, by hand or otherwise. Quarantine and Disconnect are proven where that timing
constraint doesn't apply: `test/fail/build_f7.sh` (mutation-tested, simavr-verified) drives the same monitor
through edges as fast as the policy itself allows, not as fast as a human can reconnect a wire.

## What this proves

Quarantine and Disconnect are declared, not wired by hand: `mayIsolate = true` and an `isolate(RowId)` method are
the only two things the driver adds over `recover`'s; the monitor (`fail::HealthT`) decides when to call either
from the row's own flap-rate and cost history, tracked outside the driver entirely. And a bus fault is not a device
flap: the monitor tells the two apart by attribution, not by guessing, which is what the wiring above is for -- with
nothing but the module's own pull-ups, a device fault and a bus fault are the same electrical event, so nothing
could tell them apart by watching the bus; give the bus its own pull-up and the same monitor, unmodified, starts
attributing device-only faults correctly, visibly, in real time.
