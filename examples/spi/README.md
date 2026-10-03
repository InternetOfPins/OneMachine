# spi

Discovery on two buses of one board. An RC522 RFID reader is found on an SPI bus whose chip selects are declared
statically, one **slot** each. A BMP280/BME280 is found on I2C. Each bus is its own `World`: discover once, then pump
on its own period (the card reader every 100 ms, the air sensor every second).

On SPI, the address range of an I2C scan becomes the list of slots (`oneBus::SpiSlots<...>`). There is no address and
no acknowledge, so the only runtime question is which slot holds which device, if any. `discover::SpiScan` answers it
by reading each candidate driver's ID register, at that driver's own clock and mode, twice. An empty slot reads the
idle MISO level (0x00 or 0xFF) or noise, so an ID set may hold neither value (a compile error) and a match must repeat.

## Wiring

Wemos D1 mini (ESP8266), everything at 3.3V.

| D1 mini | RC522 | BMP280 |
|---|---|---|
| 3V3 / GND | 3.3V / GND | VCC / GND |
| D5 (GPIO14) | SCK | |
| D6 (GPIO12) | MISO | |
| D7 (GPIO13) | MOSI | |
| D8 (GPIO15) | SDA (its chip select) | |
| D0 or 3V3 | RST | D0 is held high by the sketch; it must not be a chip select |
| D2 (GPIO4) | | SDA |
| D1 (GPIO5) | | SCL |

Slot 1 is D4 (GPIO2) with nothing on it: the scan reports it empty.

## Build and flash

```
pio run -e d1_mini -t upload
pio device monitor -b 115200
```

## Expected output

```
OneMachine SPI + I2C discovery
SPI slots: 1 device(s)
  row 1 at 0x0
I2C: 1 device(s)
  row 1 at 0x76
RC522 version 0x92
1203 temp[1]=25.88
1203 hPa[1]=1017.45
4810 card[1]=DEADBEEF
6120 card[1]=0
```

A card's UID is printed once when it arrives and `0` when it leaves. A 7-byte UID card shows as `88` followed by its
first three bytes (cascade level 1 only).

## What this proves, and what it doesn't yet

- The SPI scan, the RC522 driver and the empty-slot rules are checked on every change by `test/discover/build_spi1.sh`
  (a register model of the RC522 on a mock SPI bus): identification at each driver's own mode, empty slots high, low
  and floating, a stuck-low MISO, a `Fixed` slot never probed, and the card path (arrival, departure, a corrupt BCC, a
  collision).
- Failure handling is composed over the RC522's row (`fail::DevEdge` with `fail::SpiAccess`): a reader that stops
  answering goes Stale, is probed, and is initialised again when it answers; one that was reset without the sketch
  knowing (its configuration read back is gone) is initialised again at once. `test/discover/spi_fail.cpp` runs both
  against the register model with RST driven. `HealthT` is not composed here.
- The BMP280 on I2C has no failure edge: a supply disturbance that resets it leaves it in sleep mode, silent.

## Faults

Type a key in the serial monitor to drive RC522 RST (D0) from the sketch:

| Key | Fault | Log |
| --- | --- | --- |
| `v` | RST low for 3 s: the reader vanishes | `rfid[1] stale`, `card[1]=0` if a card was held, then `rfid[1] alive, init #n` after release |
| `p` | RST low for 1 ms: a silent reset, the ID still answers | `rfid[1] reinit, init #n`, the row stays Alive |

A card that picks a new UID each time its field restarts (random-UID tags, phones; UIDs starting `08`) shows a new
UID after every fault.
