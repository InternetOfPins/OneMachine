"""The wiring spec dry run (python/onemachine/wiring.py): the rig's spec (examples/spi/wiring/rig.toml) is sound, its emitted composition is the
sketch's (examples/spi/src/main.cpp), and the device's own description ('d' from the simulated device, test/link/tree_device.cpp) is what it wires.
A spec with the RC522's IRQ on D4 fails with a sentence, and the C++ rejects the same wiring (irq::Esp8266Line). Specs that differ from the
device by an address, the machines' order or the codes' order are told by the diff.
    python3 check_wiring.py <tree_device> <arduino stub dir>"""
import os, re, struct, subprocess, sys, tempfile
D = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(D, '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'python'))
from onemachine import StreamLink
from onemachine import wiring

failures = 0
def check(cond, what):
    global failures
    if not cond: failures += 1; print('FAIL:', what)

RIG = os.path.join(ROOT, 'examples', 'spi', 'wiring', 'rig.toml')
spec = wiring.load(RIG)

# ---- the rig's spec is sound, and its composition is the one written by hand in the sketch ---------------------------------------------------
check(wiring.check(spec) == [], 'rig.toml: %r' % wiring.check(spec))
board = wiring.Board('wemos-d1-mini')
check(board.gpio('D4') == 2 and board.pins['D4']['strap'] and board.pins['D0']['noirq'] and board.straps() == ['D3', 'D4', 'D8'],
      'the board from OneChip: D4 is GPIO2, a strap; D0 has no interrupt; the straps are D3 D4 D8: %r' % board.straps())
main = re.sub(r'//[^\n]*', '', open(os.path.join(ROOT, 'examples', 'spi', 'src', 'main.cpp')).read())
flat = re.sub(r'\s+', '', main)
lines = wiring.emit(spec)[1:]
missing = []
for l in lines:
    frag = l.split('=', 1)[1].rstrip(';') if l.startswith('using ') else l      # a `using X = ...`: what it is, whatever the sketch calls it
    if re.sub(r'\s+', '', frag) not in flat: missing.append(l)
check(missing == [], 'every emitted line is in the sketch: missing %r' % missing)
print('emitted %d lines, all in examples/spi/src/main.cpp' % len(lines))

# ---- the device describes what the spec wires --------------------------------------------------------------------------------------------
link = StreamLink.popen([sys.argv[1]])
st, data = link.call('d', b'')
desc = data.decode()
check(st == 0 and wiring.diff(spec, desc) == [], 'diff against the device: %r' % wiring.diff(spec, desc))

def variant(text):
    f = tempfile.NamedTemporaryFile('w', suffix='.toml', delete=False); f.write(text); f.close()
    s = wiring.load(f.name); os.unlink(f.name); return s
raw = open(RIG).read()

# another address: a sound wiring, but not this device's
s77 = variant(raw.replace('addr = 0x76', 'addr = 0x77'))
check(wiring.check(s77) == [], '0x77 is an address a BMP280 can have')
d = wiring.diff(s77, desc)
check(len(d) == 5 and all('1/118/' in m and '1/119/' in m for m in d), 'the device is at 0x76: every air code differs, by its path: %r' % d)

# the machines in another order (air first): the path's first number is the machine's position in the App
a, b = raw.index('[parts.rfid]'), raw.index('[parts.air]'); c = raw.index('# What the App publishes')
order = variant(raw[:a] + raw[b:c] + raw[a:b] + raw[c:])
d = wiring.diff(order, desc)
check(any(m.startswith('temp: the device has it at 1/118/0, the spec at 0/118/0') for m in d) and any(m.startswith('card: the device has it at 0/1, the spec at 1/1') for m in d),
      'machines in another order: the paths differ: %r' % d)

# the codes in another order: a code's number is its position
swapped = variant(raw.replace('temp = "air/temp"\npress = "air/press"', 'press = "air/press"\ntemp = "air/temp"'))
d = wiring.diff(swapped, desc)
check(len(d) == 1 and d[0].startswith("codes:"), 'codes in another order: %r' % d)

# a pin used twice, a driver that cannot be there, a node that does not exist
bad = variant(raw.replace('scl = "D1"', 'scl = "D2"').replace('addr = 0x76', 'addr = 0x40').replace('"air/ctrl/config"', '"air/ctrl/cfg"'))
e = wiring.check(bad)
check(any('i2c.scl on D2: i2c.sda is on it already' in x for x in e) and any('answers at 0x76 or 0x77, not 0x40' in x for x in e)
      and any("has no node 'ctrl/cfg'" in x for x in e), 'three faults, three sentences: %r' % e)

# ---- the RC522's IRQ on D4: the spec is refused with the reason, and the C++ refuses the same wiring ------------------------------------------
d4 = wiring.load(os.path.join(ROOT, 'examples', 'spi', 'wiring', 'rig_irq_d4.toml'))
e = wiring.check(d4)
check(len(e) == 1 and e[0].startswith('rfid.irq on D4 (GPIO2): a boot strap pin') and 'ROM (download) mode' in e[0], 'D4: %r' % e)
print('rig_irq_d4.toml: error: ' + (e[0] if e else '(none)'))
try: wiring.emit(d4); check(False, 'emit refuses what check refuses')
except wiring.WiringError as x: check(x.errors == e, 'emit says the same')
irqline = [l for l in wiring.emit(d4, checked=False) if l.startswith('using RfidLine')]
check(irqline == ['using RfidLine = irq::Sampled<2>;'], 'emitted anyway: %r' % irqline)
with tempfile.TemporaryDirectory() as t:
    src = os.path.join(t, 'irq_d4.cpp')
    open(src, 'w').write('#include <Arduino.h>\n#include "irq_esp8266.h"\n%s\nint main() { RfidLine::begin(); return RfidLine::ready(); }\n' % irqline[0])
    r = subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', '-fsyntax-only', '-I', sys.argv[2], '-I', os.path.join(ROOT, 'examples', 'spi', 'src'), src],
                       capture_output=True, text=True)
    msg = 'an interrupt line on GPIO0, GPIO2 or GPIO15 (D3, D4, D8)'
    check(r.returncode != 0 and msg in r.stderr, 'the C++ rejects it with irq::Esp8266Line\'s message: %s' % r.stderr[-300:])
    print('and the C++ build: %s' % (msg if msg in r.stderr else '(not rejected)'))

print('FAILED: %d' % failures if failures else 'OK: wiring spec dry run')
sys.exit(1 if failures else 0)
