# mpu6050

`discover` adds real data: the same MPU6050, now woken and configured when it's found, polled once a second, its
seven capabilities (3-axis acceleration, 3-axis rotation, temperature) printed as they arrive.

## Wiring

Arduino Nano (ATmega328P). A GY-521 (MPU6050) breakout: SDA to A4, SCL to A5, VCC to 5V, GND to GND.

## Build and flash

```
pio run -e nano -t upload
pio device monitor -b 115200
```

## Expected output

```
MPU6050 found
1002 ax[1]=12.375
1002 ay[1]=-3.125
1002 az[1]=998.250
1002 gx[1]=0.4
1002 gy[1]=-0.2
1002 gz[1]=0.1
1002 temp[1]=24.3
```

One line per capability, once a second, scaled to the decimal places each capability declares (`az` near 1000 mg
at rest -- 1 g -- with the board flat).

## What this proves

A driver's `Produces` list becomes a real fan-out at zero extra dispatch cost: `emit<Cap>(row, value)` reaches only
the consumers that declared `Accepts` for that capability, decided entirely at compile time.
