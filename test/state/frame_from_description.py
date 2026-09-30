#!/usr/bin/env python3
"""Build the frame of a state from a self-description and layer/field=value lines: usage frame_from_description.py DESCRIPTION VALUES.
An independent implementation of the wire format (little-endian, fixed widths, the schema hash first) with Python's struct module.
An array field takes its values comma-separated."""
import struct, sys
FMT = {'bool': '?'}
for bits, (u, i) in {8: ('B', 'b'), 16: ('H', 'h'), 32: ('I', 'i'), 64: ('Q', 'q')}.items():
    FMT['u%d' % bits] = u; FMT['i%d' % bits] = i
desc = open(sys.argv[1]).read().split('\n')
vals = dict(l.split('=') for l in open(sys.argv[2]).read().split('\n') if l)
h = int([l for l in desc if l.startswith('hash ')][0][5:], 16)
out = struct.pack('<I', h)
for l in desc:
    if '/' in l:
        path, typ = l.split(' ')
        base, _, count = typ.partition('[')
        items = vals[path].split(',') if count else [vals[path]]
        for x in items:
            out += struct.pack('<' + FMT[base], bool(int(x)) if base == 'bool' else int(x))
print(out.hex())
