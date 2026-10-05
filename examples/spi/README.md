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
| D4 (GPIO2) | RST | held high by the sketch (the fault keys pull it low); or 3V3 |
| D0 (GPIO16) | IRQ | |
| D2 (GPIO4) | | SDA |
| D1 (GPIO5) | | SCL |

Slot 1 is D3 (GPIO0) with nothing on it: the scan reports it empty.

D0 (GPIO16) is the RC522's IRQ input (push-pull, active low, set by the driver). A poll starts its command and returns; the
loop's `fail::Services` step finishes it when the line is asserted, or after 40 ms, so nothing in the loop waits for the reader. The
ESP8266 has no interrupt on GPIO16, so the line is sampled (`irq::Sampled<16>`); on a pin that has one, `irq::IsrFlag<Pin>` sets a
flag in the ISR instead. The RxIRq/TimerIRq bits tell a card from none. The sketch also uses the optional line check: a line that
disagrees with the register (not connected, stuck) is reported to the failure edge as the delivery's fault, and that row polls the
register from then on (`rc522::PollOnLineFault`); without the fallback the same fault is the chip's and Recover initialises it. The
interrupt part is optional as a whole: a mode without `using Irq` is the polling driver, unchanged. Requests are enabled only while a
command runs. Keep the IRQ off the boot strapping pins (D3, D4, D8, rejected at compile time): the RC522 keeps its state across a
reset of the board, and a pending request would hold such a pin low at the next boot.

## The air sensor as a machine

The BMP280/BME280 is a static machine of OneMenu `ItemDef` nodes (`src/bmp280_machine.h`), and takes its Criteria: `Machine<W, Addr<0x76>>`
(a second sensor at 0x77 is a second type with its own data). Nodes: `#0 temp` and `#1 press` (read-only values that move when the sensor is
read), `#2 cal` (the device's calibration constants: read when it is found, never state) and `#3 ctrl`, a group of register mimics whose `get()`
reads the chip and `set()` writes it; their defaults are the init.

The App publishes nodes under its own codes with `PublishedAt<Code, PathRef<Machine, 3, 1>, OnSync<fn>>`: an outer node that reaches the inner one
by a compile-time path (node #3, child #1; any node, a leaf of a group too) without copying it. A sync pass calls `fn(value)` for each published
value that changed, so `temp=` and `press=` appear only when they move. Key `d` prints the description: the machine's nodes, then the published
codes with their path (`<bus>/<address>/<node>[/<child>]`), fields and whether they notify.

Each register keeps the last value set (`Capture`), also while the sensor is gone. The sensor is under failure handling: each poll reads the control
registers back, and a register that no longer holds what was set means the part was reset behind the host's back. When the sensor is back, after
that or after it was unplugged, it is validated (the same chip id and calibration as the part that was here): validated, the last settings are
written again; another part, the settings are dropped and the defaults are the init. The log shows `STATUS <ms> air <from>-><to>`, then
`air restored #n` or `air defaults #n`.

Keys: `a` reads the control group by path, `o` sets `ctrl_meas` to oversampling x1, `q` to 0x2B (also while the sensor is unplugged), `r` writes the
registers' defaults, `x` resets the sensor behind the host's back.

## A Python consumer over the serial port

`pio run -e d1_mini_link -t upload` builds the same sketch with the serial port carrying the link (`role/link.h`, with payload ops) instead of the log. `python/onemachine` reads it, with the package's own `StreamLink`:

```python
import serial
from onemachine import Tree, StreamLink
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
m = Tree(StreamLink(ser.read, ser.write, ser.flush))   # reads the description
m.temp, m.press                                        # 27.62, 1026.68: scaled as the description says (0.01 C, 0.01 hPa)
m.air.ctrl_meas = 0x27                                 # set by code; read-only and out-of-range are refused here, before anything is sent
m.changes()                                            # [Change('temp', 27.61, 2761), Change('card', 4062320374, ...), Change('air', status='stale')]
m.status('air')                                        # 'alive', 'stale' or 'gone': the part's row; m.temp raises Stale when it is not alive
```

Ops: `d` the description (each code with the status of its row), `v` get by code (the status first, then the value; a part that is not alive answers its last value), `w` set by code (through the node: its limits, its capture, its register), `n` the changes since the sequence number of the last reply (state codes once each with their value now, a row's status, and the events), `f` one
fault key (`x` resets the air sensor behind the host's back, `v` and `p` reset the RFID reader). State and events are kept apart: a state code
(temp, press, a register, a row's status) is one pending bit, so a consumer that does not read for a while gets each code once with its latest value
and nothing is lost; the card is an event and waits in a `fail::Buffer` of 8: when it is full the newest are refused and counted (`m.missed`). A reply
that is lost is seen by its sequence number, and the next one carries every code (`m.resyncs`).
`examples/spi/rig_session.py` is a session on the real board; `test/link/build.sh` runs the same consumer against a simulated one.

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

Type a key in the serial monitor to drive RC522 RST (D4) from the sketch:

| Key | Fault | Log |
| --- | --- | --- |
| `v` | RST low for 3 s: the reader vanishes | `rfid[1] stale`, `card[1]=0` if a card was held, then `rfid[1] alive, init #n` after release |
| `p` | RST low for 1 ms: a silent reset, the ID still answers | `rfid[1] reinit, init #n`, the row stays Alive |

A card that picks a new UID each time its field restarts (random-UID tags, phones; UIDs starting `08`) shows a new
UID after every fault.
