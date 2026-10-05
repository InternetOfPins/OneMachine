"""A wiring spec in, the App's composition out, and the device's own description as the test that the build matched the wiring
(OneMachine Redrawn, experiment 4: a dry run).

    python3 -m onemachine.wiring check examples/spi/wiring/rig.toml            the wiring's rules; exit 1 with the reasons
    python3 -m onemachine.wiring emit  examples/spi/wiring/rig.toml            the App composition (C++), after check
    python3 -m onemachine.wiring diff  examples/spi/wiring/rig.toml desc.txt   the spec against a device's description ('d' as text)

The spec is TOML (tomllib, no dependency): board, [i2c], [spi], [parts.<name>] in the App's machine order, [publish] code = "part/node".
The board's pins come from OneChip (chips/esp8266/esp8266Device.h): each D pin's GPIO, and what its comment says of it ("boot strapping",
"no interrupt"). What a driver needs and publishes (its manifest) is DRIVERS below: a stand-in for a declaration that belongs beside each driver.

The rules, each with the sentence a person needs:
  - every pin is on the board and used once
  - a line the device drives at any time (an IRQ) is not on a boot strap pin: the RC522 keeps a pending IRQ across a reset of the board, holds the
    pin low, and the ESP8266 boots into ROM (download) mode. The C++ rejects it too (irq::Esp8266Line's static_assert), at the next build.
  - an ISR delivery is not on a pin without an interrupt (GPIO16): it is sampled there
  - a part is on the bus its driver speaks; at an address it can have; the ids it accepts are ids its driver knows
  - every published code names a node its part has
"""
import os, re, sys, tomllib
from .tree import _parse

HERE = os.path.dirname(os.path.abspath(__file__))
ONECHIP = os.environ.get('ONECHIP', os.path.normpath(os.path.join(HERE, '..', '..', '..', 'OneChip')))

BOARDS = {'wemos-d1-mini': ('chips/esp8266/esp8266Device.h', 'Esp8266Dev')}

class WiringError(Exception):
    def __init__(self, errors): super().__init__('\n'.join(errors)); self.errors = errors

class Board:
    """D pins of a board, from OneChip: gpio, and the notes its comment gives (strap: sampled at reset; noirq: no interrupt)."""
    def __init__(self, name):
        if name not in BOARDS: raise WiringError(['board %r: not known (known: %s)' % (name, ', '.join(BOARDS))])
        path = os.path.join(ONECHIP, 'include', BOARDS[name][0])
        self.pins = {}
        for m in re.finditer(r'static constexpr uint8_t (D\d+)\s*=\s*(\d+);\s*//(.*)', open(path).read()):
            note = m.group(3).lower()
            self.pins[m.group(1)] = dict(gpio=int(m.group(2)), strap='boot strapping' in note, noirq='no interrupt' in note)
        self.source = path
    def gpio(self, pin): return self.pins[pin]['gpio']
    def straps(self): return [p for p, v in self.pins.items() if v['strap']]

# What each driver needs and publishes. node: its path in the machine (positions, as PathRef<M, ...>) and its kind, and for a value its decimals and
# whether it notifies. A stand-in: the declaration belongs beside the driver (the identify entries, OneChip's pin roles, the machine's Nodes).
DRIVERS = {
    'bmp280': dict(bus='i2c', addrs=(0x76, 0x77), ids=(0x58, 0x60),
                   nodes={'temp': ((0,), 'value', 2), 'press': ((1,), 'value', 2), 'ctrl': ((3,), 'group', None),
                          'ctrl/config': ((3, 0), 'reg', None), 'ctrl/ctrl_meas': ((3, 1), 'reg', None)}),
    'rc522':  dict(bus='spi', pins={'cs': 'out', 'rst': 'out', 'irq': 'driven'},
                   nodes={'card': (None, 'event', None)}),
}

def load(path):
    with open(path, 'rb') as f: spec = tomllib.load(f)
    spec['_path'] = path
    return spec

def _pin(errors, board, owner, pin):
    if pin not in board.pins: errors.append('%s on %s: the %s has no such pin (it has %s)' % (owner, pin, 'board', ', '.join(board.pins))); return None
    return board.pins[pin]

def check(spec):
    """The wiring's rules: a list of sentences, empty when it is sound."""
    errors, used = [], {}
    board = Board(spec.get('board', '?'))
    def use(owner, pin):
        if _pin(errors, board, owner, pin) is None: return
        if pin in used: errors.append('%s on %s: %s is on it already' % (owner, pin, used[pin]))
        else: used[pin] = owner
    for bus, roles in (('i2c', ('sda', 'scl')), ('spi', ('sck', 'miso', 'mosi'))):
        for r in roles:
            if r in spec.get(bus, {}): use('%s.%s' % (bus, r), spec[bus][r])
    for p in spec.get('spi', {}).get('empty_cs', []): use('spi.empty_cs', p)
    for name, part in spec.get('parts', {}).items():
        drv = DRIVERS.get(part.get('driver'))
        if drv is None: errors.append('%s: no driver %r (known: %s)' % (name, part.get('driver'), ', '.join(DRIVERS))); continue
        if part.get('bus') != drv['bus']: errors.append('%s: a %s is on %s, not %s' % (name, part['driver'], drv['bus'], part.get('bus')))
        if 'addr' in part and part['addr'] not in drv.get('addrs', ()):
            errors.append('%s: a %s answers at %s, not 0x%02X' % (name, part['driver'], ' or '.join('0x%02X' % a for a in drv['addrs']), part['addr']))
        for i in part.get('accept', []):
            if i not in drv.get('ids', ()): errors.append('%s: accepts id 0x%02X, which its driver does not know (it knows %s)' % (name, i, ', '.join('0x%02X' % x for x in drv['ids'])))
        for role, kind in drv.get('pins', {}).items():
            v = part.get(role)
            if v is None: errors.append('%s: its %s pin is not given' % (name, role)); continue
            pin = v['pin'] if isinstance(v, dict) else v
            use('%s.%s' % (name, role), pin)
            info = board.pins.get(pin)
            if info is None: continue
            if kind == 'driven' and info['strap']:
                st = ['%s (GPIO%d)' % (p, board.gpio(p)) for p in board.straps()]
                straps = ', '.join(st[:-1]) + ' or ' + st[-1] if len(st) > 1 else ''.join(st)
                errors.append('%s.%s on %s (GPIO%d): a boot strap pin. The %s keeps a pending IRQ across a reset of the board and holds the pin low, '
                              'and the ESP8266 then boots into ROM (download) mode instead of the sketch. Put the IRQ on a pin that is not %s; '
                              'the rig uses D0 with delivery = "sampled".' % (name, role, pin, info['gpio'], part['driver'].upper(), straps))
            if kind == 'driven' and isinstance(v, dict) and v.get('delivery') == 'isr' and info['noirq']:
                errors.append('%s.%s on %s (GPIO%d): this pin has no interrupt; use delivery = "sampled"' % (name, role, pin, info['gpio']))
    for code, where in spec.get('publish', {}).items():
        part, _, node = where.partition('/')
        p = spec.get('parts', {}).get(part)
        if p is None: errors.append('publish %s = %r: no part %r' % (code, where, part)); continue
        drv = DRIVERS.get(p.get('driver'), {})
        if node not in drv.get('nodes', {}): errors.append('publish %s = %r: a %s has no node %r (it has %s)' % (code, where, p.get('driver'), node, ', '.join(drv.get('nodes', {}))))
    return errors

def _code_tag(code): return 'Code' + ''.join(w[:1].upper() + w[1:] for w in re.split(r'_', code.split('/')[-1]))   # air/ctrl_meas: CodeCtrlMeas

def emit(spec, checked=True):
    """The App's composition for examples/spi, as C++ lines (a dry run: the sketch is not built from it). Raises WiringError when check() fails;
    checked=False emits anyway (to show that the C++ rules reject the same wiring)."""
    errors = check(spec) if checked else []
    if errors: raise WiringError(errors)
    board, parts = Board(spec['board']), spec['parts']
    g = board.gpio
    out = ['// generated from %s by python3 -m onemachine.wiring emit: the App composition of examples/spi (dry run)' % os.path.basename(spec['_path'])]
    i2c, spi = spec['i2c'], spec['spi']
    out.append('using Twi = esp::Esp8266TwiMaster<%d, %d, %d>;' % (g(i2c['sda']), g(i2c['scl']), i2c.get('hz', 100000)))
    cs = [p['cs'] for p in parts.values() if p['bus'] == 'spi'] + spi.get('empty_cs', [])
    out.append('using Spi = hapi::APIOf<oneBus::SpiAPI, oneBus::SpiSlots<%s>, oneBus::SpiMaster<%d>, esp::Esp8266SpiCore>;' %
               (', '.join('esp::OutPin<%d>' % g(p) for p in cs), spi.get('hz', 4000000)))
    for name, p in parts.items():
        if p['driver'] == 'rc522':
            out.append('constexpr uint8_t rstPin = %d;' % g(p['rst']))
            irq = p['irq']
            out.append('using RfidLine = irq::%s<%d>;' % ('Sampled' if irq.get('delivery', 'sampled') == 'sampled' else 'IsrFlag', g(irq['pin'])))
        if p['driver'] == 'bmp280':
            out.append('using Bmp = bmpm::Machine<AirApp, bmpm::Addr<0x%02X>, AirMode>;' % p['addr'])
    for num, (code, where) in enumerate(spec['publish'].items()):
        out.append('struct %s { static constexpr uint8_t num = %d; ONEMACHINE_STATE_NAME(name, "%s"); };' % (_code_tag(code), num, code))
    for code, where in spec['publish'].items():
        part, _, node = where.partition('/')
        path, kind, dec = DRIVERS[parts[part]['driver']]['nodes'][node]
        if path is None: continue                                  # an event: the App's Extra, not a published node
        notify = ', oneData::OnSync<&say<%s, %d>>' % (_code_tag(code), dec) if kind == 'value' else ''
        out.append('bmpm::PublishedAt<%s, bmpm::PathRef<Bmp, %s>%s>' % (_code_tag(code), ', '.join(map(str, path)), notify))
    return out

def diff(spec, description):
    """The spec against the device's description ('d' as text): a list of mismatches, empty when the device is what was wired."""
    errors = check(spec)
    if errors: raise WiringError(errors)
    parts, names = spec['parts'], list(spec['parts'])
    got = _parse(description)
    want = list(spec['publish'].items())
    out = []
    if [c.name for c in got] != [c for c, _ in want]:
        out.append('codes: the spec publishes %s, the device %s (a code\'s number is its position)' % ([c for c, _ in want], [c.name for c in got]))
    dev = {c.name: c for c in got}
    for code, where in want:
        c = dev.get(code)
        if c is None: continue
        part, _, node = where.partition('/')
        p = parts[part]; drv = DRIVERS[p['driver']]
        path, kind, dec = drv['nodes'][node]
        machine = names.index(part)
        if path is None: expect = [machine, 1 + [q['cs'] for q in parts.values() if q['bus'] == 'spi'].index(p['cs'])]     # an event: <machine>/<row>
        else: expect = [machine, p['addr']] + list(path)
        if c.path != expect:
            out.append('%s: the device has it at %s, the spec at %s (%s)' % (code, '/'.join(map(str, c.path)), '/'.join(map(str, expect)),
                       'machine/address/node' if path is not None else 'machine/row'))
        dkind = 'event' if c.notify == 'event' else c.kind
        if dkind != kind: out.append('%s: the device has a %s, the spec\'s %s/%s is a %s' % (code, dkind, p['driver'], node, kind))
        if kind == 'value' and dec is not None and c.scaled != dec: out.append('%s: scaled %d on the device, %d in the spec' % (code, c.scaled, dec))
    return out

def main(argv):
    if len(argv) < 3 or argv[1] not in ('check', 'emit', 'diff'): print(__doc__); return 2
    spec = load(argv[2])
    try:
        if argv[1] == 'check':
            errors = check(spec)
            for e in errors: print('error: ' + e)
            if not errors: print('ok: %s' % argv[2])
            return 1 if errors else 0
        if argv[1] == 'emit': print('\n'.join(emit(spec))); return 0
        mism = diff(spec, open(argv[3]).read())
        for m in mism: print('mismatch: ' + m)
        if not mism: print('ok: the device is what %s wires' % argv[2])
        return 1 if mism else 0
    except WiringError as e:
        for x in e.errors: print('error: ' + x)
        return 1

if __name__ == '__main__': sys.exit(main(sys.argv))
