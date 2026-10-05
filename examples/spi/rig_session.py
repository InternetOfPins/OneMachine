"""A Python consumer of the spi example's link build (env d1_mini_link), over the serial port: the description, the values, a set by code and its
read-back, a reset behind the host's back, the notifications; with --unplug it waits for the air sensor to be unplugged and plugged in again and follows
its status (stale, then alive) and its last setting, and with --watch N it prints the notifications (the RFID card too) for N seconds.

    ~/.platformio/penv/bin/python examples/spi/rig_session.py /dev/ttyUSB0 [--unplug] [--watch 30]"""
import os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'python'))
import serial
from onemachine import Tree, StreamLink, OutOfRange, ReadOnly, UnknownCode, Stale

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
args = sys.argv[2:]
ser = serial.Serial(port, 115200, timeout=2)
ser.dtr = False; ser.rts = True; time.sleep(0.1); ser.rts = False        # reset the board; the link build says nothing at boot
time.sleep(2.0); ser.reset_input_buffer()
m = Tree(StreamLink(ser.read, ser.write, ser.flush))
t0 = time.time()
def say(s): print('%6.1f  %s' % (time.time() - t0, s), flush=True)

say('>>> m = Tree(StreamLink(ser.read, ser.write, ser.flush))')
for c in m.codes.values():
    say('    %-14s #%d  path %-9s %-6s %s%s%s%s  status %s' % (c.name, c.num, '/'.join(map(str, c.path)), c.kind, 'ro' if c.ro else 'rw',
        '  scaled %d' % c.scaled if c.scaled else '', '  range %d..%d default 0x%02X' % (c.lo, c.hi, c.default) if c.lo is not None else '',
        '  notifies (%s)' % c.notify if c.notify else '', c.status))
say('>>> m.temp, m.press            -> %r, %r' % (m.temp, m.press))
say('>>> m.air.config, m.air.ctrl_meas -> 0x%02X, 0x%02X' % (m.air.config, m.air.ctrl_meas))
say('>>> m.status("air")            -> %r' % m.status('air'))
say('>>> m.changes()                -> %r' % m.changes())
time.sleep(3)
say('>>> m.changes() (3 s later)    -> %r' % m.changes())

say('>>> m.air.ctrl_meas = 0x27')
m.air.ctrl_meas = 0x27
say('>>> m.air.ctrl_meas            -> 0x%02X (read back from the chip)' % m.air.ctrl_meas)
for bad in (300, -1):
    try: m.air.ctrl_meas = bad
    except OutOfRange as e: say('>>> m.air.ctrl_meas = %d      -> OutOfRange: %s' % (bad, e))
try: m.temp = 1
except ReadOnly as e: say('>>> m.temp = 1                -> ReadOnly: %s' % e)
try: m.nothing
except UnknownCode as e: say('>>> m.nothing                -> UnknownCode: %s' % str(e)[:60] + '...')

say('>>> m.fault("x")               # the air sensor is reset behind the host\'s back')
m.fault('x')
time.sleep(2.5)
say('>>> m.air.config, m.air.ctrl_meas -> 0x%02X, 0x%02X (the setting came back: 0x27, not the default 0x57)' % (m.air.config, m.air.ctrl_meas))
m.changes()

if '--unplug' in args:
    say('--- unplug the air sensor (all four wires), wait, plug it in again (any time in the next 60 s)')
    gone = back = False
    t = time.time()
    while time.time() - t < 60 and not back:
        for ch in m.changes():
            if ch.status: say('    change: %r' % (ch,))
            if ch.code == 'air' and ch.status == 'stale' and not gone:
                gone = True
                say('>>> m.status("air")            -> %r' % m.status('air'))
                try: m.temp
                except Stale as e: say('>>> m.temp                      -> Stale: %s' % e)
                say('>>> m.reading("air/ctrl_meas") -> %r (the last setting, not 0xFF)' % (m.reading('air/ctrl_meas'),))
                m.air.ctrl_meas = 0x2B
                say('>>> m.air.ctrl_meas = 0x2B   (while it is gone: captured as the last intent)')
                say('>>> m.reading("air/ctrl_meas") -> %r' % (m.reading('air/ctrl_meas'),))
            if ch.code == 'air' and ch.status == 'alive' and gone:
                back = True
        time.sleep(0.3)
    if back:
        time.sleep(0.5)
        say('>>> m.status("air")            -> %r' % m.status('air'))
        say('>>> m.air.config, m.air.ctrl_meas -> 0x%02X, 0x%02X (0x2B: the last intent came back)' % (m.air.config, m.air.ctrl_meas))
        say('>>> m.temp                     -> %r' % m.temp)
    else: say('    (the sensor was not seen going away and coming back)')

if '--watch' in args:
    secs = int(args[args.index('--watch') + 1])
    say('--- watching the notifications for %d s (move the RFID tag on and off the reader)' % secs)
    t = time.time()
    while time.time() - t < secs:
        for c in m.changes(): say('    %r' % (c,))
        time.sleep(0.5)
    say('    missed (refused by the device): %d' % m.missed)
