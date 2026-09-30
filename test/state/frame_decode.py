#!/usr/bin/env python3
"""Decode a state frame with nothing but a self-description: usage frame_decode.py DESCRIPTION HEXFRAME.
Refuses a frame whose header is not the description's hash (the cache key) or whose length is not its size.
Prints layer/field=value lines in frame order (bool as 0/1)."""
import struct, sys
FMT = {'bool': '?'}
for bits, (u, i) in {8: ('B', 'b'), 16: ('H', 'h'), 32: ('I', 'i'), 64: ('Q', 'q')}.items():
    FMT['u%d' % bits] = u; FMT['i%d' % bits] = i
desc = open(sys.argv[1]).read().split('\n')
frame = bytes.fromhex(sys.argv[2])
h = int([l for l in desc if l.startswith('hash ')][0][5:], 16)
size = int([l for l in desc if l.startswith('size ')][0][5:])
if len(frame) != size: sys.exit('frame is %d bytes, the description says %d' % (len(frame), size))
if struct.unpack_from('<I', frame)[0] != h: sys.exit('frame header %08x is not the description hash %08x' % (struct.unpack_from('<I', frame)[0], h))
off = 4
for l in desc:
    if '/' in l:
        path, typ = l.split(' ')
        base, _, count = typ.partition('[')
        n = int(count.rstrip(']')) if count else 1
        vs = []
        for _ in range(n):
            (v,) = struct.unpack_from('<' + FMT[base], frame, off); off += struct.calcsize('<' + FMT[base]); vs.append(str(int(v)))
        print('%s=%s' % (path, ','.join(vs)))
