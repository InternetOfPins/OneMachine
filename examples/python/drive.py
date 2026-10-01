#!/usr/bin/env python3
"""Drive the example's machine by role name: `led` and `lamp`, never a pin.

    python3 drive.py /dev/ttyUSB0          the Nano (needs pyserial: pip install pyserial)
    python3 drive.py --sim host/machine    the same machine on this computer, pins simulated (build it: see README.md)
    --check                                exit 1 unless every step behaves (what CI runs against --sim)"""
import os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'python'))
from onemachine import Machine, StreamLink, OutOfLimits

def open_link(args):
    if args[0] == '--sim': return StreamLink.popen([args[1]])
    import serial                                            # pyserial
    port = serial.Serial(args[0], 115200, timeout=2)
    time.sleep(2); port.reset_input_buffer()                 # opening the port resets the Nano: wait for its bootloader to pass
    return StreamLink(port.read, port.write, port.flush)

def main():
    args = [a for a in sys.argv[1:] if a != '--check']
    check = '--check' in sys.argv
    if not args: sys.exit(__doc__)
    m = Machine(open_link(args))
    fails = []
    def expect(what, cond):
        print(('  ok    ' if cond else '  FAIL  ') + what)
        if not cond: fails.append(what)

    print('roles:')
    for name, role in m.roles.items(): print('  %-5s %-7s %s   (at %s)' % (name, role.kind, role.params, m.where[name]))
    print('refs:', ', '.join(m.refs))

    for i in range(3):                                       # blink the led by name
        m.cmd.led.on = True;  m.push(); time.sleep(0.15)
        m.cmd.led.on = False; m.push(); time.sleep(0.15)
    r = m.poll()
    expect('led is off after blinking', r.led.live and not r.led.on)

    for level in (0, 50, 100, 150, 200, 250):                # the lamp is a light with max 200: 250 is clamped, and the report says so
        m.cmd.lamp.level = level; m.push(); time.sleep(0.1)
        r = m.poll(); print('  lamp asked %3d, is %3d%s' % (level, r.lamp.level, ' (clamped)' if r.lamp.clamped else ''))
    expect('lamp clamped to its max', (r.lamp.level, r.lamp.clamped) == (200, True))

    if m.roles['lamp'].tuned:                                # change a parameter at run time: the lamp's max, inside the firmware's 200
        print('lamp tuning now:', m.tune.lamp, ' firmware limits:', m.roles['lamp'].params)
        m.tune.lamp.max = 120; m.retune(); time.sleep(0.1)  # the command (250) is applied again under the new max
        r = m.poll(); print('  lamp max 120: is %d%s' % (r.lamp.level, ' (clamped)' if r.lamp.clamped else ''))
        expect('lamp clamped to the tuned max', (r.lamp.level, r.lamp.clamped) == (120, True))
        try:
            m.tune.lamp.max = 255; m.retune(); refused = False
        except OutOfLimits as e: refused = True; print('  max 255 refused:', e)
        expect('a max above the firmware limit is refused, nothing changes', refused and m.tune.lamp.max == 120)
        m.tune.lamp.max = 200; m.retune()

    if check:                                                # the supervisor goes quiet: every role to its safe command
        m.cmd.led.on = True; m.push(); time.sleep(0.1)
        time.sleep(2.3); r = m.poll()
        expect('after 2 s of silence: led off, lamp 0', (r.led.on, r.lamp.level) == (False, 0))
    m.link.close() if hasattr(m.link, 'close') else None
    sys.exit(1 if fails else 0)

main()
