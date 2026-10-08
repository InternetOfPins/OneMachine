# sila

A Nano (or this computer, with simulated endpoints) served as a laboratory device over SiLA 2. The machine declares eight roles,
one of each kind a SiLA client can use, and a Python gateway turns them into SiLA features: any SiLA client can read and command
them without knowing OneMachine, the pins or the board.

[SiLA 2](https://sila-standard.com) is a standard for lab instruments: a device is a server that publishes *features* (a feature has
properties you read or subscribe to, and commands you call, each with typed, constrained parameters) over gRPC, and every
feature carries an XML definition (FDL) a client can fetch. The gateway (`gateway.py`) reads the machine's description over the
serial link, writes one feature per role and serves them. It knows only the kinds in its table: a role of any other kind is
refused, naming the kind.

## Wiring

Arduino Nano (ATmega328P).

| role | where | needs |
|---|---|---|
| `led` | the onboard LED, pin 13 | nothing |
| `lamp` | pin 9 (PWM) | an LED and a 330 ohm resistor to GND |
| `vin` | A0 | optional. Floating, it reads noise anywhere between 0 and 5 V; jumper it to GND, 3V3 or 5V to read about 0, 3.5 and 5 V |
| `step`, `mode`, `note`, `duty`, `ping` | RAM | nothing |

## Build, flash, serve

```
pio run -e nano -t upload
pip install -r requirements.txt           # sila2 and pyserial, Python 3.10 or newer
python3 gateway.py --port /dev/ttyUSB0    # the Nano's port
```

The gateway generates the feature definitions at start (into a temporary directory, or `--fdl-dir DIR`; `--generate` writes
them and stops). It serves on `127.0.0.1:50052` without encryption (`--address`, `--sila-port`). Opening the serial port resets the
Nano, so all outputs start off at each start.

Without the board, the same machine on this computer (from this directory):

```
g++ -std=c++17 -I../../../HAPI/include -I../../include host/main.cpp -o host/machine
python3 gateway.py --sim host/machine
```

`SIM_ADC=511` in the environment sets what the simulated `vin` reads (0 to 1023).

## Use it

Any SiLA 2 client works. With the `sila2` package that the gateway already needs:

```python
import time
from sila2.client import SilaClient

c = SilaClient('127.0.0.1', 50052, insecure=True)
c.Led.SetOn(True)                      # the onboard LED
c.Note.SetText('hello')
c.Duty.SetValue(12.35)
c.Mode.SetValue('high')
time.sleep(0.4)                        # the gateway polls the machine every 0.1 s
print(c.Note.Text.get(), c.Duty.Value.get(), c.Mode.Value.get(), c.Vin.Voltage.get())
c.Duty.SetValue(100.01)                # ValidationError: the server refuses it, nothing reaches the machine
```

## How a OneMachine field becomes a SiLA type

| role (kind) | OneMachine field | SiLA feature: property, command | SiLA type and constraint |
|---|---|---|---|
| `led` (`switch`) | `on` | `Led`: `On`, `SetOn(On)` | Boolean |
| `lamp` (`light`) | `level`, bounded by `param lamp max` | `Lamp`: `Level`, `SetLevel(Level)` | Integer, minimum 0, maximum `max` (200) |
| `step` (`discrete`) | `value`, one of the `value` lines | `Step`: `Value`, `SetValue(Value)` | Integer, Set {0, 10, 50, 100} |
| `ping` (`action`) | `fire` (command), `fired` (report) | `Ping`: `Fired`, `Fire()` | `Fire` has no parameter; `Fired` is an Integer count |
| `mode` (`select`) | `index`, each `value` line has a label | `Mode`: `Value`, `SetValue(Value)` | String, Set {off, low, high} |
| `vin` (`analog`) | `raw`, with a `scale` and a `unit` line | `Vin`: `Voltage` (no command) | Real, Unit V (kg m^2 s^-3 A^-1), Voltage = raw * 5 / 1023 |
| `note` (`text`) | `text`, a `u8[16]` array | `Note`: `Text`, `SetText(Text)` | String, MaximalLength 16, Pattern `[ -~]*` (printable ASCII) |
| `duty` (`scaled`) | `raw`, with `max`, a `scale` and a `unit` line | `Duty`: `Value`, `SetValue(Value)` | Real, Unit % (dimensionless, factor 0.01), minimum 0, maximum 100 = `max` * 1 / 10 |

The kinds `light` and `switch` are OneMachine's (`role/kinds.h`); the other six are in `src/kinds.h`. The rules any such consumer follows, and how to add a kind or another consumer, are in
[docs/consumers.md](../../docs/consumers.md). The lines a kind prints
in the machine description (`value`, `scale`, `unit`) are described in [docs/role.md](../../docs/role.md). One generated
definition (`Mode`, trimmed):

```xml
<Feature ... Category="select" ... Originator="io.github.internetofpins" ...>
  <Identifier>Mode</Identifier>
  <Command>
    <Identifier>SetValue</Identifier>
    <Observable>No</Observable>
    <Parameter>
      <Identifier>Value</Identifier>
      <DataType><Constrained><DataType><Basic>String</Basic></DataType><Constraints><Set><Value>off</Value><Value>low</Value><Value>high</Value></Set></Constraints></Constrained></DataType>
    </Parameter>
  </Command>
  <Property>
    <Identifier>Value</Identifier>
    <Observable>Yes</Observable>
    <DataType><Constrained><DataType><Basic>String</Basic></DataType><Constraints><Set>...</Set></Constraints></Constrained></DataType>
  </Property>
</Feature>
```

The server's UUID is derived from the originator and the machine hash, so the same firmware is always the same server; a different
description (a role, a label, a unit) is a different server.

## The rules you meet

- **Unknown things are refused by name, at start.** A role of a kind the gateway's table (`KINDS` in `gateway.py`) does not have,
  a tuned role, a description line the kind's row does not use, a unit symbol the unit table (`UNITS`: `V` and `%`) does not
  have, a label used twice, or a scale whose bound is not a finite decimal: the gateway exits with a message that names the
  role and the thing. It never guesses.
- **Out of range is refused at the SiLA edge; the device is the backstop.** `SetLevel(250)`, `SetValue(7)` on `step`,
  `SetValue('max')` on `mode`, a 17-character or non-ASCII text, `Duty.SetValue(100.01)` or `-0.1` are validation errors
  from the server's constraint check: no frame goes to the board. A client that bypasses SiLA and sends such a value straight to
  the board gets what the kind does: `lamp` (a `light`) is **clamped** to its bound (200) and its report says `clamped`; `step`,
  `mode`, `note` and `duty` **ignore** the value, keep the previous one and report it.
- **A numeric command in presented units is rounded, and the report is the truth.** `Duty.SetValue(12.35)` becomes the raw value
  124 (the client's decimal, times 10 / 1, to the nearest integer, halves away from zero) and `Duty.Value` then reads 12.4: the value
  the device holds. Enumerated fields (`step`, `mode`) are not rounded: anything not in the set is refused.
- **One frame per command, nothing resent.** A command is sent once; a `Fire()` sends its flag in exactly one frame. The gateway
  holds the serial port under one lock, so concurrent calls do not interleave.

## Expected output

The gateway on the Nano (trimmed):

```
$ python3 gateway.py --port /dev/ttyUSB0
... sila2.server.sila_server Starting SiLA server without encryption
... sila2.server.sila_server Server started
wrote /tmp/sila-fdl-qj4ng57l/Led.sila.xml
wrote /tmp/sila-fdl-qj4ng57l/Lamp.sila.xml
wrote /tmp/sila-fdl-qj4ng57l/Step.sila.xml
wrote /tmp/sila-fdl-qj4ng57l/Mode.sila.xml
wrote /tmp/sila-fdl-qj4ng57l/Vin.sila.xml
wrote /tmp/sila-fdl-qj4ng57l/Note.sila.xml
wrote /tmp/sila-fdl-qj4ng57l/Duty.sila.xml
wrote /tmp/sila-fdl-qj4ng57l/Ping.sila.xml
serving Led, Lamp, Step, Mode, Vin, Note, Duty, Ping on 127.0.0.1:50052, server 5e39722b-8dda-5d1f-9324-689f8ec8b1da, machine hash ba36fc2d
```

A client session on the same run (`Vin` floating, so its value is noise):

```
c.Led.On.get()                  False
c.Led.SetOn(True) ...           True
c.Note.SetText('hello') ...     'hello'
c.Duty.SetValue(12.35) ...      12.4
c.Mode.SetValue('high') ...     high
c.Vin.Voltage.get()             3.758553274682307
c.Duty.SetValue(100.01)         ValidationError Parameter value rejected: Constraint violated: MaximalInclusive(100.0) (100.01)
Duty.SetValue(12.34) -> 12.3
Duty.SetValue(12.35) -> 12.4
Duty.SetValue(99.95) -> 100.0
Ping.Fire() twice: Fired 2
```

## What this example does not cover

No timeout or safe fall (a value that was set stays set), no tuning (a tuned role is refused), no observable commands or command
completion (a command returns when the frame is sent, and its effect shows in the property), and no TLS or discovery beyond the
`sila2` library's defaults (the server runs without encryption and without mDNS). `vin` is read with whatever noise the pin has:
every change of the reading is published.
