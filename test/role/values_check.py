#!/usr/bin/env python3
"""python/onemachine's reading of the value, scale and unit lines: values_check.py <description file written by values_check.cpp>"""
import sys, re
sys.path.insert(0, '../../python')
from onemachine.machine import Description, _fnv
from onemachine.schema import SchemaError

fails = 0
def check(what, cond):
    global fails
    print('  %-5s %s' % ('ok' if cond else 'FAIL', what)); fails += 0 if cond else 1

def rehash(body):                                    # a description with a correct hash line around `body` (lines, no `at`)
    h = 2166136261
    for l in body: h = _fnv(h, (l + '\n').encode())
    return 'machine 1\nhash %08x\n%s\n' % (h, '\n'.join(body))

def refused(body, why):
    try: Description(rehash(body))
    except SchemaError as e: check('refused: %s -> %s' % (why, e), True); return
    check('refused: %s' % why, False)

d = Description(open(sys.argv[1]).read())
m, v = d.roles['mode'], d.roles['vin']
check('values of mode, in order, with and without label', m.values == [(0, 'off'), (2, None), (7, 'high')])
check('scale of vin raw', v.scales == {'raw': (5, 1023)})
check('unit of vin raw', v.units == {'raw': 'V'})
check('a role that prints none has none', d.roles['led'].values == [] and d.roles['led'].scales == {} and d.roles['led'].units == {})
base = ['role mode select']
refused(base + ['value mode 1 two words'], 'a label with a space')
refused(base + ['value mode 1 bad"quote'], 'a label with a quote')
refused(base + ['value mode 1 ' + 'x' * 33], 'a label of 33 characters')
refused(base + ['value nobody 1 x'], 'a value line for a role not declared above it')
refused(base + ['scale mode index 1 0'], 'a scale with denominator 0')
refused(base + ['frobnicate mode 1'], 'an unknown line type')
try: Description(open(sys.argv[1]).read().replace('value mode 7 high', 'value mode 7 higher')); check('changed label with the old hash is refused', False)
except SchemaError as e: check('changed label with the old hash is refused -> %s' % e, 'hash' in str(e))
print('FAILED' if fails else 'ok'); sys.exit(1 if fails else 0)
