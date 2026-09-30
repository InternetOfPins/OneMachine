# python

A Nano whose outputs are driven by name from Python. The machine declares two roles, `led` (a switch) and `lamp` (a light,
at most 200 of 255). `drive.py` knows those names and nothing else: not the pins, not the board. The same script drives the
same roles on this computer with the pins simulated, so it runs without hardware too.

One consumer, but it fans out: a script, a notebook, a test rig, a dashboard are all this same `Machine` object
(`python/onemachine`, docs/role.md).

## Wiring

Arduino Nano (ATmega328P). `led` is the onboard LED (pin 13). `lamp` is pin 9: an LED and a 330 ohm resistor to GND.

## Build, flash, drive

```
pio run -e nano -t upload
pip install pyserial
python3 drive.py /dev/ttyUSB0            # the Nano's port
```

Without the board, the same machine on this computer (from this directory):

```
g++ -std=c++17 -I../../../HAPI/include -I../../include host/main.cpp -o host/machine
python3 drive.py --sim host/machine
```

## Expected output

```
roles:
  led   switch  {'safe': 0}   (at D13)
  lamp  light   {'max': 200, 'safe': 0}   (at D9~)
refs: https://github.com/InternetOfPins/OneMachine/tree/main/examples/python
  ok    led is off after blinking
  lamp asked   0, is   0
  lamp asked  50, is  50
  lamp asked 100, is 100
  lamp asked 150, is 150
  lamp asked 200, is 200
  lamp asked 250, is 200 (clamped)
  ok    lamp clamped to its max
```

The LED blinks three times and the lamp steps up in brightness, stopping at 200. With `--sim`, `at` shows the simulated
pins (`sim.pin(13)`, `sim.pwm(9)`). Stop sending for 2 seconds and the Nano puts both roles to their safe command (off):
`python3 drive.py --sim host/machine --check` checks that too.

## What this proves

- The consumer names roles only. Where a role is lives in the firmware (`src/main.cpp`: `MachineOf<Pin<13>, Pwm<9>>`); move
  the lamp to pin 10 and `drive.py` does not change.
- The description, the frames and their hashes are the only contract between the two sides: the Python side has no C++ in it
  and no generated code.
- Limits are the device's: the lamp's max is enforced on the Nano, and the report tells the consumer when it clamped.
