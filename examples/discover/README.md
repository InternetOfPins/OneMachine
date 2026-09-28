# discover

The smallest real machine: find a real MPU6050 on the I2C bus by its own identity (its WHO_AM_I register), print
the table, stop.

## Wiring

Arduino Nano (ATmega328P). A GY-521 (MPU6050) breakout: SDA to A4, SCL to A5, VCC to 5V, GND to GND.

## Build and flash

```
pio run -e nano -t upload
pio device monitor -b 115200
```

## Expected output

```
rows: 2
  row 0 bus 0x00
  row 1 dev 0x68
MPU6050 found
```

If only `row 0` shows up, the MPU6050 didn't answer -- check the wiring (SDA/SCL are easy to swap) and that the
board is a real MPU6050 (some GY-521 clones ship an MPU6500, whose WHO_AM_I differs and this entry deliberately
does not accept).

## What this proves

`Use<Own, Mpu>` asks the driver's own `idReg` (WHO_AM_I) before accepting a row: something answering 0x68 or 0x69
that isn't really an MPU6050 (a different chip at a clashing address) is never claimed, only reported absent.
