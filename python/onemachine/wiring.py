"""A wiring spec in, the App's wiring out, and the device's own description as the test that the build matched the wiring
(OneMachine Redrawn, experiments 4 and 6: a dry run).

    python3 -m onemachine.wiring check examples/spi/wiring/rig.toml [--facts <dir>]            the wiring's rules; exit 1 with the reasons
    python3 -m onemachine.wiring emit  examples/spi/wiring/rig.toml [--facts <dir>]            the App's Wiring (C++), after check
    python3 -m onemachine.wiring diff  examples/spi/wiring/rig.toml <description> [--facts <dir>]   the spec against a description (the build
                                                                                               output's <hash>.txt: its codes, paths and wiring lines)

The spec is TOML (tomllib, no dependency): board, [i2c], [spi], [parts.<name>] in the App's machine order (the canonical order, D45), and [publish]
code = "part/node".

What the rules know comes from the C++, through facts.txt, which examples/spi/describe.cpp writes beside the description (describe.py does it on
every PlatformIO build; without --facts this module builds it with the host's g++):
  board   each D pin's GPIO, whether the chip samples it at reset (strap, and the level it needs) and whether it has an interrupt: OneChip's Esp8266Pins
  bus     who drives each bus line (I2C SDA shared, SCL the MCU; SPI MISO the part, the rest the MCU): discover::I2cLines, SpiLines
  driver  each driver's manifest beside the driver (bmpm::Manifest, rc522::Manifest): its bus, addresses, ids, its own lines and who drives each one,
          its nodes (walked from its machine: a group's children are `group/child`)
The rules, each with the sentence a person needs:
  - every pin is on the board and used once
  - a line the part drives or shares is not on a boot strap pin; the MCU's own lines may be (a chip select on D3 or D8 idles high and is fine)
  - an ISR delivery is not on a pin without an interrupt (GPIO16): it is sampled there
  - a part is on the bus its driver speaks, at an address and with ids its driver knows
  - every published code names a node its part has
"""
import os, re, subprocess, sys, tempfile, tomllib
from .tree import _parse, wiring as _wiring

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))
LIBS = ['OneMachine', 'HAPI', 'OneBus', 'OneData', 'OneMenu', 'OneItem', 'OneOutput', 'OneBit', 'OnePin', 'OneChip', 'OneParse', 'OneInput', 'OneIO']

class WiringError(Exception):
    def __init__(self, errors): super().__init__('\n'.join(errors)); self.errors = errors

class Facts:
    """facts.txt: the board's pins, the buses' lines and the drivers' manifests, as the C++ declares them."""
    def __init__(self, text):
        self.board, self.pins, self.buses, self.drivers = None, {}, {}, {}
        for line in text.split('\n'):
            w = line.split()
            if not w: continue
            if w[0] == 'board': self.board = w[1]
            elif w[0] == 'pin':
                self.pins[w[1]] = dict(gpio=int(w[2]), strap='strap' in w, level=('high' if 'high' in w else 'low') if 'strap' in w else None, noirq='noirq' in w)
            elif w[0] == 'bus': self.buses[w[1]] = dict(zip(w[2::2], w[3::2]))
            elif w[0] == 'driver':
                d = self.drivers.setdefault(w[1], dict(bus=w[2], addrs=(), ids=(), lines={}, nodes={}, event=None))
                key = None
                for t in w[3:]:
                    if t in ('addrs', 'ids'): key = t; d[key] = ()
                    else: d[key] = d[key] + (int(t, 16),)
            elif w[0] == 'line': self.drivers[w[1]]['lines'][w[2]] = w[3]
            elif w[0] == 'node':
                kind = w[4]; dec = int(w[5]) if kind == 'value' else None
                self.drivers[w[1]]['nodes'][w[2]] = (tuple(int(x) for x in w[3].split('/')), kind, dec)
            elif w[0] == 'event': self.drivers[w[1]]['event'] = w[2]; self.drivers[w[1]]['nodes'][w[2]] = (None, 'event', None)

    @classmethod
    def load(cls, where=None):
        """From a facts.txt, or a directory holding one; None: build describe.cpp with the host's g++ and run it."""
        if where:
            return cls(open(os.path.join(where, 'facts.txt') if os.path.isdir(where) else where).read())
        with tempfile.TemporaryDirectory() as t:
            build(t)
            return cls(open(os.path.join(t, 'facts.txt')).read())
    def gpio(self, pin): return self.pins[pin]['gpio']
    def straps(self): return [p for p, v in self.pins.items() if v['strap']]

def build(out, wiring_header=None, cxx=None):
    """Build examples/spi/describe.cpp on the host (with an emitted Wiring when given) and run it into out: <hash>.txt and facts.txt.
    Returns (returncode, the compiler's or the program's output); raises when the build fails and wiring_header is None."""
    parent = os.path.dirname(ROOT)
    inc = [f for n in LIBS for f in ('-I', os.path.join(parent, n, 'include'))]
    exe = os.path.join(out, 'describe')
    cmd = [cxx or os.environ.get('CXX', 'g++'), '-std=c++17', '-O1'] + inc
    if wiring_header: cmd.append('-DAIRTREE_WIRING="%s"' % os.path.abspath(wiring_header))
    r = subprocess.run(cmd + [os.path.join(ROOT, 'examples', 'spi', 'describe.cpp'), '-o', exe], capture_output=True, text=True)
    if r.returncode:
        if wiring_header is None: raise RuntimeError('describe.cpp does not build:\n' + r.stderr[-2000:])
        return r.returncode, r.stderr
    r = subprocess.run([exe, out], capture_output=True, text=True)
    return r.returncode, r.stdout.strip()

def load(path):
    with open(path, 'rb') as f: spec = tomllib.load(f)
    spec['_path'] = path
    return spec

def _pin_of(v): return v['pin'] if isinstance(v, dict) else v

def _lines(spec, facts):
    """Every line the spec wires: (owner, pin, who drives it, the part's driver or None, its role)."""
    out = []
    for bus, roles in (('i2c', ('sda', 'scl')), ('spi', ('sck', 'miso', 'mosi'))):
        for r in roles:
            if r in spec.get(bus, {}): out.append(('%s.%s' % (bus, r), spec[bus][r], facts.buses.get(bus, {}).get(r, 'mcu'), None, r))
    for p in spec.get('spi', {}).get('empty_cs', []): out.append(('spi.empty_cs', p, facts.buses.get('spi', {}).get('cs', 'mcu'), None, 'cs'))
    for name, part in spec.get('parts', {}).items():
        drv = facts.drivers.get(part.get('driver'))
        if drv is None: continue
        for role, drive in drv['lines'].items():
            if part.get(role) is not None: out.append(('%s.%s' % (name, role), _pin_of(part[role]), drive, part['driver'], role))
    return out

def check(spec, facts):
    """The wiring's rules: a list of sentences, empty when it is sound."""
    errors, used = [], {}
    if spec.get('board') != facts.board: errors.append('board %r: the facts are for %r' % (spec.get('board'), facts.board))
    for name, part in spec.get('parts', {}).items():
        drv = facts.drivers.get(part.get('driver'))
        if drv is None: errors.append('%s: no driver %r (known: %s)' % (name, part.get('driver'), ', '.join(facts.drivers))); continue
        if part.get('bus') != drv['bus']: errors.append('%s: a %s is on %s, not %s' % (name, part['driver'], drv['bus'], part.get('bus')))
        if 'addr' in part and part['addr'] not in drv['addrs']:
            errors.append('%s: a %s answers at %s, not 0x%02X' % (name, part['driver'], ' or '.join('0x%02X' % a for a in drv['addrs']), part['addr']))
        for i in part.get('accept', []):
            if i not in drv['ids']: errors.append('%s: accepts id 0x%02X, which its driver does not know (it knows %s)' % (name, i, ', '.join('0x%02X' % x for x in drv['ids'])))
        for role in drv['lines']:
            if role not in part: errors.append('%s: its %s pin is not given' % (name, role))
    for owner, pin, drive, driver, role in _lines(spec, facts):
        info = facts.pins.get(pin)
        if info is None: errors.append('%s on %s: the board has no such pin (it has %s)' % (owner, pin, ', '.join(facts.pins))); continue
        if pin in used: errors.append('%s on %s: %s is on it already' % (owner, pin, used[pin])); continue
        used[pin] = owner
        if drive != 'mcu' and info['strap']:
            st = ['%s (GPIO%d)' % (p, facts.gpio(p)) for p in facts.straps()]
            why = ('The %s keeps a pending IRQ across a reset of the board and holds the pin low' % driver.upper() if role == 'irq' and driver
                   else 'A part can hold an open-drain line low across a reset of the MCU' if drive == 'shared'
                   else 'The part can drive it while the MCU is held in reset')
            errors.append('%s on %s (GPIO%d): a boot strap pin, and this line is driven by the %s, not the MCU alone. %s, and the ESP8266 then boots into '
                          'ROM (download) mode instead of the sketch (it needs GPIO%d %s at reset). Put it on a pin that is not %s; a line the MCU drives '
                          '(a chip select, a reset) may stay there.' % (owner, pin, info['gpio'], 'part' if drive == 'device' else 'part and the MCU (shared)',
                          why, info['gpio'], info['level'], ', '.join(st[:-1]) + ' or ' + st[-1]))
        v = spec.get('parts', {}).get(owner.split('.')[0], {}).get(role) if driver else None
        if isinstance(v, dict) and v.get('delivery') == 'isr' and info['noirq']:
            errors.append('%s on %s (GPIO%d): this pin has no interrupt; use delivery = "sampled"' % (owner, pin, info['gpio']))
    for code, where in spec.get('publish', {}).items():
        part, _, node = where.partition('/')
        p = spec.get('parts', {}).get(part)
        if p is None: errors.append('publish %s = %r: no part %r' % (code, where, part)); continue
        drv = facts.drivers.get(p.get('driver'), {})
        if node not in drv.get('nodes', {}): errors.append('publish %s = %r: a %s has no node %r (it has %s)' % (code, where, p.get('driver'), node, ', '.join(drv.get('nodes', {}))))
    return errors

_NAMESPACE = {'bmp280': 'bmpm', 'rc522': 'rc522m'}   # the namespace of each driver's machine header

def _code_tag(code): return 'Code' + ''.join(w[:1].upper() + w[1:] for w in re.split(r'_', code.split('/')[-1]))   # air/ctrl_meas: CodeCtrlMeas

def emit(spec, facts, checked=True):
    """The App's Wiring for examples/spi (a header air_tree.h includes with -DAIRTREE_WIRING), then its codes and published nodes, as C++ lines.
    Raises WiringError when check() fails; checked=False emits anyway (to show that the C++ rules reject the same wiring). The Wiring ends at the
    line `};`: wiring_header() is that part alone."""
    errors = check(spec, facts) if checked else []
    if errors: raise WiringError(errors)
    parts = spec['parts']
    def cam(part, role): return part + role[:1].upper() + role[1:]
    out = ['// generated from %s by python3 -m onemachine.wiring emit: the App\'s Wiring (air_tree.h includes it with -DAIRTREE_WIRING)' % os.path.basename(spec['_path']),
           'struct Wiring {',
           '  using Board = hw::esp8266::Esp8266Pins;',
           '  static constexpr const char* board = "%s";' % spec['board'],
           '  static constexpr uint8_t sda = Board::%s, scl = Board::%s;' % (spec['i2c']['sda'], spec['i2c']['scl']),
           '  static constexpr uint8_t sck = Board::%s, miso = Board::%s, mosi = Board::%s;' % (spec['spi']['sck'], spec['spi']['miso'], spec['spi']['mosi'])]
    cs = ['%s = Board::%s' % (cam(n, 'cs'), p['cs']) for n, p in parts.items() if p['bus'] == 'spi'] + ['emptyCs = Board::%s' % c for c in spec['spi'].get('empty_cs', [])]
    out.append('  static constexpr uint8_t %s;' % ', '.join(cs))
    for name, p in parts.items():
        lines = [r for r in facts.drivers[p['driver']]['lines'] if r != 'cs']
        if lines: out.append('  static constexpr uint8_t %s;' % ', '.join('%s = Board::%s' % (cam(name, r), _pin_of(p[r])) for r in lines))
        if 'addr' in p: out.append('  static constexpr uint8_t %s = 0x%02X;' % (cam(name, 'addr'), p['addr']))
    out.append('};')
    for num, code in enumerate(spec['publish']):
        out.append('struct %s { static constexpr uint8_t num = %d; ONEMACHINE_STATE_NAME(name, "%s"); };' % (_code_tag(code), num, code))
    for code, where in spec['publish'].items():
        part, _, node = where.partition('/')
        driver = parts[part]['driver']
        path, kind, dec = facts.drivers[driver]['nodes'][node]
        notify = ', oneData::OnSync<Note<%s>::fn>' % _code_tag(code) if kind == 'value' else ''
        ns = _NAMESPACE[driver]                                                      # a machine's published nodes are its header's: bmpm::, rc522m::
        out.append('%s::PublishedAt<%s, %s::PathRef<M, %s>%s>' % (ns, _code_tag(code), ns, ', '.join(map(str, path)), notify))
    return out

def wiring_header(lines): return '\n'.join(lines[:lines.index('};') + 1]) + '\n'

def diff(spec, facts, description):
    """The spec against a description (the build output's text): its codes, paths, kinds and scales, and its wiring lines. A list of mismatches."""
    errors = check(spec, facts)
    if errors: raise WiringError(errors)
    parts, names = spec['parts'], list(spec['parts'])
    got, wired = _parse(description), _wiring(description)
    want = list(spec['publish'].items())
    out = []
    if [c.name for c in got] != [c for c, _ in want]:
        out.append('codes: the spec publishes %s, the device %s (a code\'s number is its position)' % ([c for c, _ in want], [c.name for c in got]))
    dev = {c.name: c for c in got}
    spi_parts = [q['cs'] for q in parts.values() if q['bus'] == 'spi']
    for code, where in want:
        c = dev.get(code)
        if c is None: continue
        part, _, node = where.partition('/')
        p = parts[part]
        path, kind, dec = facts.drivers[p['driver']]['nodes'][node]
        machine = names.index(part)
        ident = p['addr'] if 'addr' in p else spi_parts.index(p['cs'])                  # the device's identity in a path: its address, or its slot
        expect = [machine, ident] + list(path)
        if c.path != expect:
            out.append('%s: the device has it at %s, the spec at %s (%s)' % (code, '/'.join(map(str, c.path)), '/'.join(map(str, expect)),
                       'machine/address/node' if 'addr' in p else 'machine/slot/node'))
        dkind = 'event' if c.notify == 'event' else c.kind
        if dkind != kind: out.append('%s: the device has a %s, the spec\'s %s/%s is a %s' % (code, dkind, p['driver'], node, kind))
        if kind == 'value' and dec is not None and c.scaled != dec: out.append('%s: scaled %d on the device, %d in the spec' % (code, c.scaled, dec))
    if not wired:
        out.append('wiring: the description has no wiring lines (the build output\'s <hash>.txt has them; the text a device sends does not)')
        return out
    def same(where, role, pin, drive):
        got_, want_ = wired.get(where, {}).get(role), (facts.gpio(pin), drive)
        if got_ != want_:
            out.append('wiring %s %s: the device has %s, the spec %s (GPIO%d, %s)' % (where, role, 'GPIO%d %s' % got_ if got_ else 'nothing', pin, want_[0], want_[1]))
    for bus, roles in (('i2c', ('sda', 'scl')), ('spi', ('sck', 'miso', 'mosi'))):
        for r in roles: same(bus, r, spec[bus][r], facts.buses[bus][r])
    for c in spec['spi'].get('empty_cs', []): same('empty', 'cs', c, facts.buses['spi']['cs'])
    for name, p in parts.items():
        drv = facts.drivers[p['driver']]
        if wired.get(name, {}).get('driver') != p['driver']: out.append('wiring %s: the device has a %s, the spec a %s' % (name, wired.get(name, {}).get('driver'), p['driver']))
        for role, drive in drv['lines'].items(): same(name, role, _pin_of(p[role]), drive)
        if 'addr' in p and wired.get(name, {}).get('at') != p['addr']:
            at = wired.get(name, {}).get('at')
            out.append('wiring %s: the device is at %s, the spec at 0x%02X' % (name, '0x%02X' % at if at is not None else 'no address', p['addr']))
    return out

def main(argv):
    args = argv[1:]
    facts_dir = None
    if '--facts' in args: i = args.index('--facts'); facts_dir = args[i + 1]; del args[i:i + 2]
    if len(args) < 2 or args[0] not in ('check', 'emit', 'diff'): print(__doc__); return 2
    spec, facts = load(args[1]), Facts.load(facts_dir)
    try:
        if args[0] == 'check':
            errors = check(spec, facts)
            for e in errors: print('error: ' + e)
            if not errors: print('ok: %s' % args[1])
            return 1 if errors else 0
        if args[0] == 'emit': print('\n'.join(emit(spec, facts))); return 0
        mism = diff(spec, facts, open(args[2]).read())
        for m in mism: print('mismatch: ' + m)
        if not mism: print('ok: the device is what %s wires' % args[1])
        return 1 if mism else 0
    except WiringError as e:
        for x in e.errors: print('error: ' + x)
        return 1

if __name__ == '__main__': sys.exit(main(sys.argv))
