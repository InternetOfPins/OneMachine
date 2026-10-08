#!/usr/bin/env python3
"""python/onemachine's facts for a kind (role_facts): facts_check.py <host build of examples/sila>
What the description of the examples/sila machine says through each of its eight kinds; the arithmetic between raw and presented values; and each refusal, by name."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../python'))
from fractions import Fraction
from onemachine import Machine, StreamLink, SchemaError, KindError, LinkError, role_facts, KINDS
from onemachine.machine import _fnv, OK
from onemachine.schema import Field, Layer

fails = 0
def check(what, cond, extra=''):
    global fails
    print('  %-5s %s%s' % ('ok' if cond else 'FAIL', what, ('   ' + str(extra)) if extra and not cond else '')); fails += 0 if cond else 1

m = Machine(StreamLink.popen([sys.argv[1]]))
F = {name: role_facts(m, name) for name in m.roles}
check('the eight kinds of the table are the eight roles of the machine', sorted(KINDS) == sorted(r.kind for r in m.roles.values()) and len(F) == 8)

# ---- the facts of each role
f = F['led'];  check('switch: reports and commands `on`, nothing else', (f.report, f.command, f.low, f.high, f.values, f.scale, f.unit, f.size) == ('on', 'on', None, None, (), None, None, None))
f = F['lamp']; check('light: `level`, bounded 0..max (param lamp max 200)', (f.report, f.command, f.low, f.high, f.command_type) == ('level', 'level', 0, 200, 'u16'))
check('light: its report has `clamped` and `live`', {'level', 'clamped', 'live'} <= set(f.report_fields), f.report_fields)
f = F['step']; check('discrete: `value`, the allowed values in order', (f.command, f.values, f.labels) == ('value', (0, 10, 50, 100), ()))
f = F['mode']; check('select: `index`, values and labels in order', (f.report, f.command, f.values, f.labels) == ('index', 'index', (0, 1, 2), ('off', 'low', 'high')))
f = F['vin'];  check('analog: reports `raw`, no command, scale 5/1023, unit V', (f.report, f.command, f.scale, f.unit, f.low, f.high) == ('raw', None, (5, 1023), 'V', None, None))
f = F['note']; check('text: `text`, 16 bytes', (f.report, f.command, f.size) == ('text', 'text', 16))
f = F['duty']; check('scaled: `raw` 0..1000, scale 1/10, unit %', (f.report, f.command, f.low, f.high, f.scale, f.unit) == ('raw', 'raw', 0, 1000, (1, 10), '%'))
f = F['ping']; check('action: reports `fired`, commands `fire`', (f.report, f.command) == ('fired', 'fire'))
check('only the commandable kinds have a command field', sorted(n for n, x in F.items() if x.command is None) == ['vin'])

# ---- raw <-> presented
d = F['duty']
check('presented: raw 123 is exactly 12.3', d.presented(123) == Fraction(123, 10))
edge = [(12.34, 123), (12.35, 124), (12.36, 124), (0.04, 0), (0.05, 1), (99.95, 1000), (100, 1000), (-0.05, -1), (-0.04, 0), (0.15, 2), (0.25, 3)]
check('to_raw: nearest integer of the decimal as written, halves away from zero', all(d.to_raw(v) == r for v, r in edge), [(v, d.to_raw(v), r) for v, r in edge if d.to_raw(v) != r])
check('to_raw takes the decimal the client wrote (12.35), not the binary double below it', Fraction(12.35) < Fraction(1235, 100) and d.to_raw(12.35) == 124)
v = F['vin']
check('analog presented: float(presented) is float(raw) * 5 / 1023 for every u16 raw', all(float(v.presented(r)) == float(r) * 5 / 1023 for r in range(65536)))
s = F['mode']
check('select: label_of / raw_of', (s.label_of(2), s.raw_of('low')) == ('high', 1))
try: s.label_of(9); check('select: a raw no `value` line names is a LinkError', False)
except LinkError as e: check('select: a raw no `value` line names is a LinkError naming the role', 'mode' in str(e) and '9' in str(e))
try: s.raw_of('max'); check('select: an unknown label is a KeyError', False)
except KeyError: check('select: an unknown label is a KeyError', True)
n = F['note']
check('text: packed ASCII, NUL-padded to the buffer, unpacked up to the first NUL', n.pack_text('hi') == [104, 105] + [0] * 14 and n.unpack_text(n.pack_text('hi Rui')) == 'hi Rui' and n.unpack_text([0] * 16) == '')
for what, text in (('17 characters', 'x' * 17), ('non-ASCII', 'hé'), ('a control character', 'a\tb')):
    try: n.pack_text(text); check('text: %s is refused' % what, False)
    except ValueError: check('text: %s is refused' % what, True)

# ---- refusals, by name: the machine's own description with one line changed (the hash recomputed), over the same command and report descriptions
base = {op: m.link.call(op)[1].decode() for op in 'mcr'}
lines = [l for l in base['m'].split('\n') if l][2:]
def rehash(ls):
    h = 2166136261
    for l in ls:
        if not l.startswith('at '): h = _fnv(h, (l + '\n').encode())
    return 'machine 1\nhash %08x\n%s\n' % (h, '\n'.join(ls))
class Fake:
    def __init__(self, text): self.text = text
    def call(self, op, payload=b''):
        op = op if isinstance(op, str) else chr(op)
        return OK, (self.text if op == 'm' else base[op]).encode()
def refused(what, edit, role, *words):
    ls = list(lines); edit(ls)
    try:
        mm = Machine(Fake(rehash(ls))); role_facts(mm, role); check('refused: %s' % what, False)
    except KindError as e: check('refused: %s -> %s' % (what, e), role in str(e) and all(w in str(e) for w in words), e)
def sub(ls, old, new): ls[ls.index(old)] = new
def drop(ls, *olds):
    for o in olds: ls.remove(o)
def after(ls, line, *new): i = ls.index(line); ls[i + 1:i + 1] = list(new)
refused('a light with no `max` param', lambda ls: drop(ls, 'param lamp max 200'), 'lamp', "'max'", 'level')
refused('a light whose max does not fit its field (70000 in a u16)', lambda ls: sub(ls, 'param lamp max 200', 'param lamp max 70000'), 'lamp', '70000', 'u16')
refused('a discrete with no `value` lines', lambda ls: drop(ls, *[l for l in ls if l.startswith('value step')]), 'step', 'value')
refused('a discrete with a repeated value', lambda ls: sub(ls, 'value step 50', 'value step 10'), 'step', 'not distinct')
refused('a discrete value with a label', lambda ls: sub(ls, 'value step 50', 'value step 50 fifty'), 'step', 'label')
refused('a select value with no label', lambda ls: sub(ls, 'value mode 2 high', 'value mode 2'), 'mode', 'value 2 has no label')
refused('a select with a repeated label', lambda ls: sub(ls, 'value mode 2 high', 'value mode 2 low'), 'mode', 'same label', 'low')
refused('a `unit` line on a switch', lambda ls: after(ls, 'role led switch', 'unit led on V'), 'led', 'unit lines')
refused('a `value` line on a light', lambda ls: after(ls, 'role lamp light', 'value lamp 5 five'), 'lamp', 'value lines')
refused('an analog with no `scale` line', lambda ls: drop(ls, 'scale vin raw 5 1023'), 'vin', 'scale')
refused('an analog whose `unit` is for another field', lambda ls: sub(ls, 'unit vin raw V', 'unit vin other V'), 'vin', 'unit')
refused('a scaled with a scale of 0/10', lambda ls: sub(ls, 'scale duty raw 1 10', 'scale duty raw 0 10'), 'duty', 'not positive')
refused('an analog with a scale whose numerator is 0 (0/1023)', lambda ls: sub(ls, 'scale vin raw 5 1023', 'scale vin raw 0 1023'), 'vin', 'scale 0/1023', 'not positive')
refused('an analog with a negative numerator (-5/1023)', lambda ls: sub(ls, 'scale vin raw 5 1023', 'scale vin raw -5 1023'), 'vin', 'scale -5/1023', 'not positive')
refused('an analog with a negative denominator (5/-1023)', lambda ls: sub(ls, 'scale vin raw 5 1023', 'scale vin raw 5 -1023'), 'vin', 'scale 5/-1023', 'not positive')
refused('a scaled with a negative numerator (-1/10)', lambda ls: sub(ls, 'scale duty raw 1 10', 'scale duty raw -1 10'), 'duty', 'scale -1/10', 'not positive')
refused('a scaled with a negative denominator (1/-10)', lambda ls: sub(ls, 'scale duty raw 1 10', 'scale duty raw 1 -10'), 'duty', 'scale 1/-10', 'not positive')
try: Machine(Fake(rehash([('scale vin raw 5 0' if l == 'scale vin raw 5 1023' else l) for l in lines]))); check('refused: a scale with denominator 0, by the description itself', False)
except SchemaError as e: check('refused: a scale with denominator 0, by the description itself -> %s' % e, 'denominator 0' in str(e))
refused('a scaled with no `max` param', lambda ls: drop(ls, 'param duty max 1000'), 'duty', "'max'")
refused('a scaled whose max does not fit its field', lambda ls: sub(ls, 'param duty max 1000', 'param duty max 70000'), 'duty', '70000')
check('the machine as it is is accepted by every refusal harness', all(role_facts(Machine(Fake(rehash(list(lines)))), n) for n in m.roles))

# ---- the field-shape refusals: stub schemas with one field changed
class S:
    def __init__(self, **layers): self.l = layers
    def layer(self, name): return self.l.get(name)
def layer(*fields):
    l = Layer('x'); l.fields = [Field(*x) for x in fields]; return l
class M:
    def __init__(self, kind, cmd, rep, **params):
        from onemachine.machine import RoleInfo
        r = RoleInfo('x', kind); r.params = params
        if kind in ('discrete', 'select'): r.values = [(0, 'a' if kind == 'select' else None), (1, 'b' if kind == 'select' else None)]
        if kind in ('analog', 'scaled'): r.scales, r.units = {'raw': (1, 1)}, {'raw': 'V'}
        self.roles, self.command_schema, self.report_schema = {'x': r}, S(x=cmd), S(x=rep)
live = ('live', 'bool', None)
def shape(what, kind, cmd, rep, *words, **params):
    try: role_facts(M(kind, cmd, rep, **params), 'x'); check('refused: %s' % what, False)
    except KindError as e: check('refused: %s -> %s' % (what, e), 'role x' in str(e) and all(w in str(e) for w in words), e)
shape('a switch whose command is not a bool', 'switch', layer(('on', 'u8', None)), layer(('on', 'bool', None), live), 'command', 'not bool')
shape('a light with no `level` in its report', 'light', layer(('level', 'u16', None)), layer(('lvl', 'u16', None), live), 'report', "'level'", max=10)
shape('a light whose level is an array', 'light', layer(('level', 'u8', 4)), layer(('level', 'u16', None), live), 'command', 'scalar', max=10)
shape('a light whose level is a bool', 'light', layer(('level', 'bool', None)), layer(('level', 'u16', None), live), 'command', 'bool', 'no integer', max=10)
shape('a text whose command is a u16 array', 'text', layer(('text', 'u16', 4)), layer(('text', 'u8', 4), live), 'command', 'u8 array')
shape('a text whose command and report buffers differ', 'text', layer(('text', 'u8', 4)), layer(('text', 'u8', 8), live), '4 bytes', '8')
shape('an action whose command flag is signed', 'action', layer(('fire', 'i8', None)), layer(('fired', 'u16', None), live), 'signed')
ok = role_facts(M('action', layer(('fire', 'u8', None)), layer(('fired', 'u16', None), live)), 'x')
check('the same action with an unsigned flag is accepted', ok.command_type == 'u8')
try: role_facts(M('axis', layer(), layer()), 'x'); check('refused: a kind the table does not have', False)
except KindError as e: check('refused: a kind the table does not have -> %s' % e, 'axis' in str(e) and 'role x' in str(e))

print('FAILED' if fails else 'ok'); sys.exit(1 if fails else 0)
