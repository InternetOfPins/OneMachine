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

Wiggle the SDA or SCL wire repeatedly for a few seconds -- enough transient faults for the monitor to notice a
pattern, not just one recovered glitch. Expect:

```
HLTH 12040 row 1 flap=40 cost=12 quarantined=0 disconnected=0
HLTH 18500 row 1 flap=110 cost=30 quarantined=1 disconnected=0
HLTH 31200 row 1 flap=140 cost=180 quarantined=1 disconnected=1
```

Once `disconnected=1`, pin 8 goes low: the module is powered off, `recover`'s own retry/reprobe machinery is no
longer even touching the bus for it. Power-cycle the Nano (or the module, once pin 8 is high again) to see it
re-discovered from a clean state.

## What this proves

Quarantine and Disconnect are declared, not wired by hand: `mayIsolate = true` and an `isolate(RowId)` method are
the only two things the driver adds over `recover`'s; the monitor (`fail::HealthT`) decides when to call it from
the row's own flap-rate and cost history, tracked outside the driver entirely.
