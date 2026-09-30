#!/usr/bin/env python3
"""python/onemachine's Schema against the native state peers of test/state (host_peer.cpp, array_check.cpp): every scalar width and array
kind decodes to the native values and re-encodes byte-equal, state::json matches, refusals refuse. usage: check_schema.py HOST_PEER ARRAY_PEER"""
import os, subprocess, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'python'))
from onemachine import Schema, SchemaError
run = lambda *a: subprocess.run(a, capture_output=True, text=True, check=True).stdout.strip()
def values(s, st):   # the same layer/field=value lines the native peer prints (bool as 0/1)
    out = []
    for l in s.layers:
        for f in l.fields:
            v = getattr(getattr(st, l.name), f.name)
            out.append('%s/%s=%s' % (l.name, f.name, ','.join(str(int(x)) for x in (v if f.n else [v]))))
    return '\n'.join(out)
ok = True
def check(what, cond):
    global ok; ok &= bool(cond); print(('ok   ' if cond else 'FAIL ') + what)
for peer, steps in ((sys.argv[1], (0, 5)), (sys.argv[2], (0, 3))):
    s = Schema.parse(run(peer, 'describe'))
    check('%s: description hash %08x size %d recomputed' % (peer.split('/')[-1], s.hash, s.size), True)
    for n in steps:
        fr = bytes.fromhex(run(peer, 'frame', str(n)))
        st = s.decode(fr)
        check('  frame %d decodes to the native values' % n, values(s, st) == run(peer, 'values', str(n)))
        check('  frame %d re-encodes byte-equal' % n, s.encode(st) == fr)
    if 'host' in peer:
        check('  json of frame 0 equals state::json', s.json(s.decode(bytes.fromhex(run(peer, 'frame', '0')))) == run(peer, 'json'))
        check('  zero state encodes to the native zero frame', s.encode(s.zero()).hex() == run(peer, 'zero'))
    try: s.decode(b'\0' + fr[1:]); check('  another hash refused', False)
    except SchemaError: check('  another hash refused', True)
    try: s.decode(fr[:-1]); check('  short frame refused', False)
    except SchemaError: check('  short frame refused', True)
    st = s.zero(); l = s.layers[0]; f = l.fields[0]
    try: setattr(getattr(st, l.name), f.name, [10**20] * f.n if f.n else 10**20); check('  out-of-range assignment refused', False)
    except ValueError: check('  out-of-range assignment refused', True)
sys.exit(0 if ok else 1)
