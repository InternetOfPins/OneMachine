#!/usr/bin/env python3
"""Compute the schema hash and the frame size from a state self-description (stdin), knowing nothing of the C++.
Checks the description against itself (fields sit under the layer above them) and against its own `hash` and `size` lines.
Prints `<hash> <size>`; exit 1 on any disagreement."""
import sys
CODE = {'bool': (1, 1)}                                    # type -> (schema byte, width in bytes)
for bits in (8, 16, 32, 64):
    CODE['u%d' % bits] = (bits // 8 * 2, bits // 8)
    CODE['i%d' % bits] = (bits // 8 * 2 + 1, bits // 8)

def fnv(h, data):
    for b in data:
        h = ((h ^ b) * 16777619) & 0xFFFFFFFF
    return h

def main():
    lines = sys.stdin.read().split('\n')
    if lines and lines[-1] == '': lines.pop()
    if lines[0] != 'state 1': sys.exit('not a state 1 description')
    want_hash = want_size = None; body = []
    for l in lines[1:]:
        if l.startswith('hash '): want_hash = int(l[5:], 16)
        elif l.startswith('size '): want_size = int(l[5:])
        else: body.append(l)
    h, size, layer = 2166136261, 4, None
    for l in body:
        if l.startswith('layer '):
            layer = l[6:]; h = fnv(fnv(h, layer.encode()), b'\xff')
        else:
            path, typ = l.split(' ')
            lay, field = path.split('/')
            if lay != layer: sys.exit('field %s is not under layer %s' % (path, layer))
            base, _, count = typ.partition('[')
            code, width = CODE[base]
            h = fnv(fnv(fnv(h, field.encode()), b'\x00'), bytes([code | 0x80 if count else code]))
            if count:
                n = int(count.rstrip(']')); h = fnv(h, bytes([n & 255, n >> 8])); width *= n
            size += width
    if h != want_hash or size != want_size:
        sys.exit('description says hash %08x size %s, computed %08x size %d' % (want_hash, want_size, h, size))
    print('%08x %d' % (h, size))
main()
