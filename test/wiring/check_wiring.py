"""The wiring spec dry run (python/onemachine/wiring.py, experiments 4 and 6), on facts the C++ declares: OneChip's Esp8266Pins, the buses' lines,
the BMP280's and RC522's manifests (examples/spi/describe.cpp writes them to facts.txt beside the description).
  - the rig's spec (examples/spi/wiring/rig.toml) is sound; the Wiring it emits, built in place of the hand-written one (air_tree.h), gives the same
    description, to its hash; its codes and published nodes are air_tree.h's; the device built from these types describes itself by that hash
  - the description's wiring lines are what the spec wires (diff); another address, another order of machines or codes, two MCU lines swapped
    between strap pins are each told
  - the strap rule goes by who drives the line: the MCU's lines may sit on D3, D4, D8; SDA (shared) may not; the RC522's IRQ on D4 is refused with
    the reason, and the same wiring emitted anyway does not build (air_tree.h's rule, from the manifests' roles and OneChip's facts)
    python3 check_wiring.py <tree_device>"""
import os, re, struct, sys, tempfile
D = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(D, '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'python'))
from onemachine import StreamLink
from onemachine import wiring
from onemachine.tree import wiring as wiring_lines

failures = 0
def check(cond, what):
    global failures
    if not cond: failures += 1; print('FAIL:', what)

RIG = os.path.join(ROOT, 'examples', 'spi', 'wiring', 'rig.toml')
spec, raw = wiring.load(RIG), open(os.path.join(ROOT, 'examples', 'spi', 'wiring', 'rig.toml')).read()
def variant(text):
    f = tempfile.NamedTemporaryFile('w', suffix='.toml', delete=False); f.write(text); f.close()
    s = wiring.load(f.name); os.unlink(f.name); return s

with tempfile.TemporaryDirectory() as hand, tempfile.TemporaryDirectory() as gen:
    # ---- the facts and the description, from the hand-written Wiring ----------------------------------------------------------------------
    rc, path = wiring.build(hand)
    check(rc == 0 and path.endswith('.txt'), 'describe.cpp: %r' % path)
    facts = wiring.Facts.load(hand)
    desc = open(path).read()
    h = os.path.basename(path)[:8]
    check(facts.pins['D4'] == dict(gpio=2, strap=True, level='high', noirq=False) and facts.pins['D8']['level'] == 'low' and facts.pins['D0']['noirq']
          and facts.straps() == ['D3', 'D4', 'D8'], 'the board, from OneChip\'s Esp8266Pins: %r' % facts.pins)
    check(facts.drivers['rc522']['lines'] == {'cs': 'mcu', 'rst': 'mcu', 'irq': 'device'} and facts.drivers['rc522']['ids'][:2] == (0x91, 0x92)
          and facts.drivers['bmp280']['addrs'] == (0x76, 0x77) and facts.drivers['bmp280']['ids'] == (0x58, 0x60)
          and facts.drivers['bmp280']['nodes']['ctrl/ctrl_meas'] == ((3, 1), 'reg', None) and facts.buses['i2c'] == {'sda': 'shared', 'scl': 'mcu'},
          'the manifests, from the drivers: %r' % facts.drivers)

    # ---- the rig's spec: sound; its Wiring is the hand-written one, to the description's hash ------------------------------------------------
    check(wiring.check(spec, facts) == [], 'rig.toml: %r' % wiring.check(spec, facts))
    lines = wiring.emit(spec, facts)
    hdr = os.path.join(gen, 'wiring.h'); open(hdr, 'w').write(wiring.wiring_header(lines))
    rc, path2 = wiring.build(gen, hdr)
    check(rc == 0 and os.path.basename(path2) == os.path.basename(path) and open(path2).read() == desc,
          'the emitted Wiring builds the same description: %s against %s' % (os.path.basename(path2), os.path.basename(path)))
    tree = re.sub(r'\s+', '', re.sub(r'//[^\n]*', '', open(os.path.join(ROOT, 'examples', 'spi', 'src', 'air_tree.h')).read()))
    rest = lines[lines.index('};') + 1:]
    missing = [l for l in rest if re.sub(r'\s+', '', l) not in tree]
    check(missing == [], 'the emitted codes and published nodes are air_tree.h\'s: missing %r' % missing)
    print('emitted Wiring builds description %s, the hand-written one\'s; %d code and node lines, all in air_tree.h' % (os.path.basename(path2), len(rest)))

    # ---- the device built from the same types describes itself by that hash; the build output's description is what the spec wires ----------
    link = StreamLink.popen([sys.argv[1]])
    st, data = link.call('d', b'')
    check(st == 0 and data.startswith(('hash ' + h).encode()), 'the device\'s hash is the build output\'s: %r, %s' % (data[:13], h))
    check(wiring.diff(spec, facts, desc) == [], 'diff: %r' % wiring.diff(spec, facts, desc))
    w = wiring_lines(desc)
    check(w['rfid'] == {'driver': 'rc522', 'cs': (15, 'mcu'), 'rst': (2, 'mcu'), 'irq': (16, 'device')} and w['air'] == {'driver': 'bmp280', 'at': 0x76}
          and w['empty'] == {'cs': (0, 'mcu')}, 'the wiring lines: %r' % w)

    # ---- differences the diff tells -----------------------------------------------------------------------------------------------------
    d = wiring.diff(variant(raw.replace('addr = 0x76', 'addr = 0x77')), facts, desc)
    check(len(d) == 6 and sum('1/118/' in m for m in d) == 5 and any(m.startswith('wiring air: the device is at 0x76, the spec at 0x77') for m in d),
          'another address: 5 paths and the wiring line: %r' % d)
    a, b, c = raw.index('[parts.rfid]'), raw.index('[parts.air]'), raw.index('# What the App publishes')
    d = wiring.diff(variant(raw[:a] + raw[b:c] + raw[a:b] + raw[c:]), facts, desc)
    check(any(m.startswith('temp: the device has it at 1/118/0, the spec at 0/118/0') for m in d), 'machines in another order: %r' % d)
    d = wiring.diff(variant(raw.replace('temp = "air/temp"\npress = "air/press"', 'press = "air/press"\ntemp = "air/temp"')), facts, desc)
    check(len(d) == 1 and d[0].startswith('codes:'), 'codes in another order: %r' % d)
    swapped = variant(raw.replace('rst = "D4"', 'rst = "D3"').replace('empty_cs = ["D3"]', 'empty_cs = ["D4"]'))
    check(wiring.check(swapped, facts) == [], 'RST on D3 and the empty slot on D4: both the MCU\'s lines, on strap pins: allowed')
    d = wiring.diff(swapped, facts, desc)
    check(sorted(m.split(':')[0] for m in d) == ['wiring empty cs', 'wiring rfid rst'], 'and the diff tells the two pins: %r' % d)

    # ---- the strap rule by role ---------------------------------------------------------------------------------------------------------
    e = wiring.check(variant(raw.replace('sda = "D2"', 'sda = "D3"').replace('empty_cs = ["D3"]', 'empty_cs = []')), facts)
    check(len(e) == 1 and e[0].startswith('i2c.sda on D3 (GPIO0): a boot strap pin, and this line is driven by the part and the MCU (shared)'),
          'SDA (shared) on D3: refused: %r' % e)
    bad = variant(raw.replace('scl = "D1"', 'scl = "D2"').replace('addr = 0x76', 'addr = 0x40').replace('"air/ctrl/config"', '"air/ctrl/cfg"'))
    e = wiring.check(bad, facts)
    check(any('i2c.scl on D2: i2c.sda is on it already' in x for x in e) and any('answers at 0x76 or 0x77, not 0x40' in x for x in e)
          and any("has no node 'ctrl/cfg'" in x for x in e), 'three faults, three sentences: %r' % e)

    d4 = wiring.load(os.path.join(ROOT, 'examples', 'spi', 'wiring', 'rig_irq_d4.toml'))
    e = wiring.check(d4, facts)
    check(len(e) == 1 and e[0].startswith('rfid.irq on D4 (GPIO2): a boot strap pin, and this line is driven by the part') and 'ROM (download) mode' in e[0]
          and 'needs GPIO2 high at reset' in e[0], 'D4: %r' % e)
    print('rig_irq_d4.toml: error: ' + (e[0] if e else '(none)'))
    try: wiring.emit(d4, facts); check(False, 'emit refuses what check refuses')
    except wiring.WiringError as x: check(x.errors == e, 'emit says the same')
    with tempfile.TemporaryDirectory() as bad:
        hdr = os.path.join(bad, 'wiring.h'); open(hdr, 'w').write(wiring.wiring_header(wiring.emit(d4, facts, checked=False)))
        rc, err = wiring.build(bad, hdr)
        msg = 'wiring: a line the part drives is on a boot strap pin'
        check(rc != 0 and msg in err, 'the emitted Wiring does not build: %s' % err[-400:])
        print('and the C++ build: %s' % (msg if msg in err else '(not rejected)'))

print('FAILED: %d' % failures if failures else 'OK: wiring spec dry run')
sys.exit(1 if failures else 0)
