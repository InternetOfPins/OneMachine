"""A Python consumer of the spi example's link build (env d1_mini_link), over the serial port: the description, the values, a set by code and its
read-back, a reset behind the host's back, the notifications; with --unplug it waits for the air sensor to be unplugged and plugged in again and follows
its status (stale, then alive) and its last setting, with --reader it does the same for the RFID reader's registers (rfid/gain, rfid/antenna: a set, a
silent reset of the reader, the reader held in reset for 3 s, the antenna off and on), and with --watch N it prints the notifications (the RFID card too)
for N seconds.

    ~/.platformio/penv/bin/python examples/spi/rig_session.py /dev/ttyUSB0 [--unplug] [--reader] [--watch 30] [--descriptions <dir>]

The device describes itself by hash; the text is the build output's (default: .pio/build/d1_mini_link/description, which `pio run -e d1_mini_link` writes)."""
import os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'python'))
import serial
from onemachine import Tree, StreamLink, OutOfRange, ReadOnly, UnknownCode, Stale

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
args = sys.argv[2:]
ser = serial.Serial(port, 115200, timeout=2)
ser.dtr = False; ser.rts = True; time.sleep(0.1); ser.rts = False        # reset the board; the link build says nothing at boot
time.sleep(2.0); ser.reset_input_buffer()
desc = args[args.index('--descriptions') + 1] if '--descriptions' in args else os.path.join(os.path.dirname(os.path.abspath(__file__)), '.pio', 'build', 'd1_mini_link', 'description')
m = Tree(StreamLink(ser.read, ser.write, ser.flush), descriptions=desc)
t0 = time.time()
def say(s): print('%6.1f  %s' % (time.time() - t0, s), flush=True)

say('>>> m = Tree(StreamLink(ser.read, ser.write, ser.flush), descriptions=<the build output>)')
for c in m.codes.values():
    say('    %-14s #%d  path %-9s %-6s %s%s%s%s%s  status %s' % (c.name, c.num, '/'.join(map(str, c.path)), c.kind, 'ro' if c.ro else 'rw',
        '  scaled %d' % c.scaled if c.scaled else '', '  range %d..%d default 0x%02X' % (c.lo, c.hi, c.default) if c.lo is not None else '',
        '  field %d..%d' % c.field if c.field else '',
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

if '--reader' in args:
    say('--- the RFID reader: rfid/gain and rfid/antenna are fields of registers of its machine (RFCfgReg 0x26 bits 6..4, TxControlReg 0x14 bits 1..0)')
    say('>>> m.rfid.gain, m.rfid.antenna -> %d, %d (the defaults: 4, 3; the registers 0x48 and 0x83)' % (m.rfid.gain, m.rfid.antenna))
    say('>>> m.status("rfid/gain")        -> %r' % m.status('rfid/gain', refresh=True))
    m.changes()
    m.rfid.gain = 7
    say('>>> m.rfid.gain = 7; m.rfid.gain -> %d (read back from the chip)' % m.rfid.gain)
    for bad, what in ((8, 'gain'), (4, 'antenna')):
        try: setattr(m.rfid, what, bad)
        except OutOfRange as e: say('>>> m.rfid.%s = %d     -> OutOfRange: %s' % (what, bad, e))
    say('>>> m.fault("p")                 # RST low for 1 ms: the reader is reset behind the host\'s back')
    m.fault('p')
    time.sleep(1.5)
    say('>>> m.rfid.gain, m.rfid.antenna -> %d, %d (the gain came back: 7, not the default 4)' % (m.rfid.gain, m.rfid.antenna))
    say('>>> m.changes()                  -> %r (no status change; the card reads 0: the pulse dropped the RF field)' % (m.changes(),))
    say('>>> m.fault("v")                 # RST low for 3 s: the reader vanishes')
    m.fault('v')
    time.sleep(1.2)
    say('>>> m.status("rfid/gain")        -> %r' % m.status('rfid/gain', refresh=True))
    m.rfid.gain = 5
    say('>>> m.rfid.gain = 5              (while it is gone: the last intent)')
    say('>>> m.reading("rfid/gain")       -> %r' % (m.reading('rfid/gain'),))
    t = time.time()
    while time.time() - t < 20 and m.status('rfid/gain', refresh=True) != 'alive': time.sleep(0.5)
    say('>>> m.status("rfid/gain")        -> %r, after %.1f s' % (m.status('rfid/gain', refresh=True), time.time() - t))
    say('>>> m.rfid.gain                  -> %d (5: the last intent came back)' % m.rfid.gain)
    for ch in m.changes():
        if ch.status: say('    change: %r' % (ch,))
    def cards(secs, until=None):
        t = time.time(); got = []
        while time.time() - t < secs and not (until and until(got)): got += [c for c in m.changes() if c.code == 'card']; time.sleep(0.3)
        return got
    say('--- the card: needs a tag on the reader (without one, the antenna steps below show no card events)')
    got = cards(8, lambda g: any(c.value for c in g))
    say('    waiting up to 8 s for a card: %r' % (got,))
    m.rfid.antenna = 0
    say('>>> m.rfid.antenna = 0; m.rfid.antenna -> %d (both TX drivers off)' % m.rfid.antenna)
    say('    card events in 6 s with the antenna off: %r (a card that was held leaves: 0, and none arrives)' % (cards(6),))
    m.rfid.antenna = 3
    say('>>> m.rfid.antenna = 3           (on again)')
    say('    card events in 6 s with the antenna on: %r' % (cards(6, lambda g: any(c.value for c in g)),))
    m.rfid.gain = 4
    say('>>> m.rfid.gain = 4              (the default again)')

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
