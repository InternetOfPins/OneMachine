# recover

`mpu6050` adds a failure edge: a transient I2C fault (a bumped wire, a glitch) is retried, the device is re-probed
if it stops answering, and re-initialised if it comes back. Nothing in `loop()` needs to know a fault happened.

## Wiring

Arduino Nano (ATmega328P). A GY-521 (MPU6050) breakout: SDA to A4, SCL to A5, VCC to 5V, GND to GND.

## Build and flash

```
pio run -e nano -t upload
pio device monitor -b 115200
```

## Try it

Disconnect SDA or SCL for a few seconds, then reconnect it. Expect:

```
STATUS 4210 row 1 0->1
...samples stop...
STATUS 9840 row 1 1->0
1002 ax[1]=12.375
...samples resume...
```

(`0` is Alive, `1` is Stale in this line's raw status value.) The row goes Stale while the wire is off, comes back
Alive on its own once it's reconnected, and the MPU6050 is reconfigured before sampling resumes -- it declares
`reinitOnBusReturn`, so every bus return re-runs its `init()` rather than assuming its configuration survived
(the most common real cause of a bus outage is the whole bus's power going with it, which really does reset the
device; confirmed on real hardware -- without this, the samples come back as a steady stream of zeros, not a
crash, easy to miss and worse to debug than a clean failure).

## What this proves

The failure edge is a compile-time choice, not a runtime one: `Mode`'s `BusStack`/`DevStack` name the layers
(`Retry`, `Recover`, `Reprobe`, ...) once, and the driver's own `read()`/`attempt()` shape barely changes from
`mpu6050`'s (`Edge::serve`/`Edge::checkedRead` in place of a direct register read) to get it.
