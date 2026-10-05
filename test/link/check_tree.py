"""The Python consumer of a machine tree (python/onemachine/tree.py) against the spi example's air sensor on a simulated chip (tree_device.cpp),
over a pipe (StreamLink) and in-process (CtypesLink): the description, values, a set by code and its read-back from the chip, the refusals,
a reset behind the host's back, an unplug with a set while it is gone, the notifications, and a full queue."""
import os, struct, sys
D = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(D, '..', '..', 'python'))
from onemachine import Tree, StreamLink, CtypesLink, OutOfRange, ReadOnly, UnknownCode, Stale, Reading, Change, LinkError

CTYPES = len(sys.argv) > 1 and sys.argv[1] == 'ctypes'
DESC = sys.argv[2] if len(sys.argv) > 2 else None        # where the build wrote the description by hash (examples/spi/describe.cpp)
failures = 0
def check(cond, what):
    global failures
    if not cond: failures += 1; print('FAIL:', what)

link = CtypesLink(os.path.join(D, 'tree_device.so'), autocycle=False) if CTYPES else StreamLink.popen([os.path.join(D, 'tree_device')])
def op(c, payload=b''):
    st, data = link.call(c, payload); return st, data
def advance(ms): check(op('t', struct.pack('<I', ms))[0] == 0, 'advance')
def reg(a): return op('R', bytes([a]))[1][0]

st, d = op('d')
HASHED = d.startswith(b'hash ')
if HASHED:
    check(len(d) == 5 + 8 + 1 + 6 + 1 and d.endswith(b'000000\n'), 'd by hash: the hash and six statuses, %d bytes: %r' % (len(d), d))
    try: Tree(link); check(False, 'a device described by hash, and no description directory')
    except LinkError as e: check('-DONEMACHINE_DESC_TEXT' in str(e) and d[5:13].decode() in str(e), 'a readable error: %s' % e)
m = Tree(link, descriptions=DESC)
def key(name):                                           # a code in a raw request: its number under the hash, or its name (the text build)
    return struct.pack('<IB', m.hash, m.codes[name].num) if HASHED else name.encode()
if HASHED: check(m.description == open(os.path.join(DESC, d[5:13].decode() + '.txt')).read(), 'the text is the build output\'s')
if not HASHED and DESC:                                  # the two walks: the text the device sends, without its statuses, is the build output's
    import re
    files = [f for f in os.listdir(DESC) if re.fullmatch(r'[0-9a-f]{8}\.txt', f)]      # <hash>.txt (facts.txt is beside it)
    static = '\n'.join(re.sub(r' status (alive|stale|gone)$', '', l) for l in m.description.split('\n'))
    built = open(os.path.join(DESC, files[0])).read()
    built = built[:built.index('wiring ')] if 'wiring ' in built else built      # the wiring lines are the build output's only (the text build is Round 6's)
    check(len(files) == 1 and static == built, 'the text walk and the hash walk write the same description')
check(list(m.codes) == ['temp', 'press', 'air', 'air/config', 'air/ctrl_meas', 'card'], 'the codes, in the device\'s order: %s' % list(m.codes))
check(m.codes['temp'].scaled == 2 and m.codes['temp'].notify == 'sync' and m.codes['temp'].ro, 'temp: scaled 2, notifies, read-only')
check(m.codes['air'].kind == 'group' and m.codes['air'].group_size == 2, 'air is a group of 2')
c = m.codes['air/ctrl_meas']
check(c.kind == 'reg' and not c.ro and (c.lo, c.hi, c.default) == (0, 255, 0x57) and c.notify is None, 'air/ctrl_meas: a register, rw, 0..255, default 0x57, silent')
check(m.codes['card'].notify == 'event' and m.codes['card'].unit == 'u32', 'card: an event, u32')
check([c.num for c in m._codes] == list(range(6)), 'numbered in order')
check(all(c.status == 'alive' for c in m._codes) and m.status('temp') == 'alive' and m.status('card') == 'alive', 'every code is alive in the description')

# ---- values, scaled by the description -----------------------------------------------------------------------------------------
advance(500)
check(abs(m.temp - 25.08) < 1e-9 and abs(m.press - 1006.53) < 1e-9, 'temp 25.08, press 1006.53: %r %r' % (m.temp, m.press))
check(m.raw('temp') == 2508 and m.raw('press') == 100653, 'the raw integers')
check(m.air.ctrl_meas == 0x57 and m.air.config == 0x90, 'the registers read from the chip: the defaults')
try: m.raw('air'); check(False, 'a group has no value')
except AttributeError: pass

# ---- notifications: on change only -------------------------------------------------------------------------------------------
ch = m.changes()
check(sorted(x.code for x in ch) == ['press', 'temp'] and {x.code: x.raw for x in ch} == {'temp': 2508, 'press': 100653}, 'the first values arrive once: %r' % ch)
check(m.changes() == [], 'nothing since')
advance(1000)
check(m.changes() == [], 'a value that did not move says nothing')
op('S', struct.pack('<II', 415148, 519888 + 1600)); advance(300)
ch = m.changes()
check([x.code for x in ch] == ['temp', 'press'] and ch[0].value > 25.08, 'a warmer sample: temp and press, once: %r' % ch)
check(m.changes() == [], 'and no more')

# ---- set by code: through the node, to the register ---------------------------------------------------------------------------
m.air.ctrl_meas = 0x27
check(reg(0xF4) == 0x27 and m.air.ctrl_meas == 0x27, 'set air/ctrl_meas: the chip has it, and it reads back')
check(m.changes() == [], 'a silent code does not notify')
for bad, exc in ((300, OutOfRange), (-1, OutOfRange)):
    try: m.air.ctrl_meas = bad; check(False, 'range %r' % bad)
    except exc: pass
check(reg(0xF4) == 0x27, 'refused here: nothing was sent')
st, _ = op('w', struct.pack('<i', 300) + key('air/ctrl_meas')); check(st == 3 and reg(0xF4) == 0x27, 'the device refuses it too (BadValue) and keeps the value')
try: m.temp = 1; check(False, 'temp is read-only')
except ReadOnly: pass
try: m.air = 1; check(False, 'air is a group')
except AttributeError: pass
try: m.nothing; check(False, 'unknown code')
except UnknownCode: pass
st, _ = op('w', struct.pack('<i', 1) + key('temp')); check(st == 0x82, 'a set of a read-only code: the device answers ReadOnly')
st, _ = op('v', struct.pack('<IB', m.hash, 6) if HASHED else b'nope'); check(st == 0x80, 'an unknown code: Unknown')
if HASHED:                                               # a number is valid only under the hash it was read with; names are not codes here
    st, _ = op('v', struct.pack('<IB', m.hash ^ 1, 0)); check(st == 1, 'another build\'s hash: BadHash (%d)' % st)
    st, _ = op('w', struct.pack('<i', 0x27) + struct.pack('<IB', m.hash ^ 1, 4)); check(st == 1 and reg(0xF4) == 0x27, 'a set under another hash: BadHash, nothing written')
    st, _ = op('v', b'temp'); check(st == 2, 'a name, by hash: BadLength (%d)' % st)
    st, d = op('v', key('temp')); check(st == 0 and len(d) == 5, 'temp by its number: status and value')

# ---- a reset behind the host's back: the last setting comes back ---------------------------------------------------------------
op('x'); check(reg(0xF4) == 0x00, 'the chip lost its settings')
advance(300)
check(reg(0xF4) == 0x27 and reg(0xF5) == 0x90 and m.air.ctrl_meas == 0x27, 'restored, not the defaults')

# ---- unplugged: Stale, not a register that reads 0xFF; a set while it is gone is the last intent ----------------------------------
m.changes()
check(m.status('air') == 'alive', 'alive before')
t_before = m.temp
op('u'); advance(1500)
ch = m.changes()
check(sorted(x.code for x in ch if x.status == 'stale') == ['air', 'air/config', 'air/ctrl_meas', 'press', 'temp'] and all(x.value is None for x in ch if x.status),
      'the unplug is announced once, for every code of the part: %r' % ch)
check(len([x for x in ch if x.status]) == 5, 'and only once')
check(m.status('air') == 'stale' and m.status('temp') == 'stale' and m.status('card') == 'alive', 'stale for the part, alive for the card')
check(m.status('air', refresh=True) == 'stale', 'and the device says so')
m.refresh(); check(m.codes['temp'].status == 'stale' and m.codes['card'].status == 'alive', 'the description read again carries the statuses now (by hash too)')
try: m.temp; check(False, 'temp of a stale part raises')
except Stale as e: check(e.status == 'stale' and e.last == t_before, 'Stale carries the status and the last value: %r' % e.last)
try: m.air.ctrl_meas; check(False, 'a register of a stale part raises')
except Stale as e: check(e.last == 0x27, 'a register answers its last set, not 0xFF: %r' % e.last)
r = m.reading('air/ctrl_meas'); check(isinstance(r, Reading) and r.status == 'stale' and r.value == 0x27, 'reading() marks it instead: %r' % (r,))
m.air.ctrl_meas = 0x2B                                           # the write cannot reach it: it is the last intent
check(m.reading('air/ctrl_meas') == (0x2B, 'stale'), 'the intent is what it answers: %r' % (m.reading('air/ctrl_meas'),))
check(m.changes() == [], 'a stale part says nothing more')
op('p'); check(reg(0xF4) == 0x00, 'plugged in again: reset values')
advance(3000)
check(reg(0xF4) == 0x2B and m.air.ctrl_meas == 0x2B, 'the last intent came back')
ch = m.changes()
check(sorted(x.code for x in ch if x.status == 'alive') == ['air', 'air/config', 'air/ctrl_meas', 'press', 'temp'], 'and the return is announced: %r' % [x for x in ch if x.status])
check(m.status('air') == 'alive' and m.status('temp') == 'alive', 'alive again')
check(m.temp == t_before, 'and reads again')

# ---- the card's row has a status of its own -------------------------------------------------------------------------------------
op('z', bytes([2])); advance(20)
ch = m.changes()
check([x for x in ch if x.status] == [Change('card', status='gone')] and m.status('card') == 'gone' and m.status('air') == 'alive', 'the card\'s row gone: %r' % ch)
op('z', bytes([0])); advance(20); m.changes()
st, d = op('v', key('card')); check(st == 4 and d == bytes([0]), 'an event has no value: NoValue and the status alone')

# ---- the card: an event with a value ------------------------------------------------------------------------------------------
m.changes()
op('k', struct.pack('<I', 0xF22216F6)); op('k', struct.pack('<I', 0))
ch = m.changes()
check([(x.code, x.value) for x in ch] == [('card', 0xF22216F6), ('card', 0)], 'the card arrives and leaves, in order: %r' % ch)

# ---- a full queue: the newest are refused and counted ------------------------------------------------------------------------
before = m.missed
for i in range(12): op('k', struct.pack('<I', i + 1))
ch = m.changes()
check(len(ch) == 8 and [x.value for x in ch] == list(range(1, 9)), 'the oldest 8 are kept: %r' % [x.value for x in ch])
check(m.missed - before == 4, 'and the 4 refused are counted: %d' % (m.missed - before))
check(m.changes() == [] and m.missed - before == 4, 'the count is told once')

print('FAILED: %d' % failures if failures else 'OK: python Tree over %s, description %s' % ('ctypes' if CTYPES else 'a pipe', 'by hash' if HASHED else 'as text'))
sys.exit(1 if failures else 0)
