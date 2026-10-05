"""A machine tree seen by its codes: the description the device publishes, and nothing else a consumer needs (examples/spi, link/frame.h).

    m = Tree(StreamLink(ser.read, ser.write, ser.flush))   # a serial port; CtypesLink('./tree.so') in-process; or any object with call(op, payload)
    m.codes['temp'].scaled, m.codes['air/ctrl_meas'].lo    # what the description says: 2, 0
    m.temp                                                 # 27.62: the value, scaled by the description (temp is in 0.01 C)
    m.air.ctrl_meas = 0x27                                 # set by code: read-only and out-of-range are refused here, before anything is sent
    m.air.ctrl_meas                                        # 39: read back from the device
    m.changes()                                            # [Change('temp', 27.61, 2761), ...]: what changed since the last call
    m.missed                                               # how many notifications the device had to refuse (its queue was full) since the start

A code is a name, with `/` for the part of a group (`air/ctrl_meas` is `m.air.ctrl_meas`). The description lists, per code: its path in the
machine (`<bus>/<address>/<node>[/<child>]`, for people), whether it notifies (`sync`: a value that moves; `event`), read-only or read-write, its
kind (a value, a register with its default, a group) and, for a value, how many decimals it is scaled by and the range a set accepts.
A device that refused notifications (`missed`) says how many; the values a consumer follows are read again with get()."""
import struct
from collections import namedtuple
from .machine import LinkError

OK, BAD_LENGTH, BAD_VALUE, NO_VALUE, UNKNOWN, TOO_LONG, READ_ONLY = 0, 2, 3, 4, 0x80, 0x81, 0x82

class OutOfRange(ValueError): pass
class ReadOnly(AttributeError): pass
class UnknownCode(AttributeError): pass

Change = namedtuple('Change', 'code value raw')

class Code:
    def __init__(self, num, name, path, notify):
        self.num, self.name, self.path, self.notify = num, name, path, notify
        self.kind, self.ro, self.scaled, self.lo, self.hi, self.default, self.group_size, self.unit = 'value', True, 0, None, None, None, 0, None
    def __repr__(self): return 'Code(%d %s %s%s)' % (self.num, self.name, self.kind, ' ro' if self.ro else ' rw')
    def to_value(self, raw):
        if self.unit == 'u32': return raw & 0xFFFFFFFF
        return raw / 10 ** self.scaled if self.scaled else raw
    def to_raw(self, value): return int(round(value * 10 ** self.scaled)) if self.scaled else int(value)

def _parse(text):
    """The `published` section: one line per code, in the order the device numbers them."""
    codes, seen = [], False
    for line in text.split('\n'):
        if line == 'published': seen = True; continue
        if not seen or not line.startswith('  '): continue
        head, _, rest = line.strip().partition(' -> ')
        words = rest.split(' ')
        c = Code(len(codes), head, [int(p) for p in words[0].split('/')], words[1] if words[1] != 'silent' else None)
        if c.notify == 'notify': c.notify = words[2]; words = words[1:]     # `notify sync|event` is two words, `silent` one
        w, i = words[1:] if c.notify is None else words[2:], 0
        while i < len(w):
            t = w[i]
            if t == 'group': c.kind, c.group_size, i = 'group', int(w[i + 1]), i + 2
            elif t == 'reg': c.kind, c.default, i = 'reg', int(w[i + 3], 16), i + 4        # reg 0xF4 default 0x57
            elif t in ('ro', 'rw'): c.ro, i = t == 'ro', i + 1
            elif t == 'value': i += 1
            elif t == 'scaled': c.scaled, i = int(w[i + 1]), i + 2
            elif t == 'range': c.lo, c.hi = (int(x) for x in w[i + 1].split('..')); i += 2
            elif t == 'u32': c.unit, i = 'u32', i + 1
            else: i += 1
        codes.append(c)
    return codes

class Group:
    """m.air: the codes below `air/`."""
    def __init__(self, tree, prefix): object.__setattr__(self, '_t', tree); object.__setattr__(self, '_p', prefix)
    def __getattr__(self, name):
        if name.startswith('_'): raise AttributeError(name)
        return self._t._access(self._p + '/' + name)
    def __setattr__(self, name, value): self._t.set(self._p + '/' + name, value)
    def __dir__(self): return [c.name[len(self._p) + 1:] for c in self._t._codes if c.name.startswith(self._p + '/')]

class Tree:
    def __init__(self, link):
        object.__setattr__(self, 'link', link)
        object.__setattr__(self, 'missed', 0)
        self.refresh()

    def _call(self, op, payload=b''):
        return self.link.call(op, payload)

    def refresh(self):
        """Read the description again."""
        st, data = self._call('d')
        if st != OK: raise LinkError('description: status %d' % st)
        text = data.decode()
        codes = _parse(text)
        object.__setattr__(self, 'description', text)
        object.__setattr__(self, '_codes', codes)
        object.__setattr__(self, 'codes', {c.name: c for c in codes})

    def raw(self, code):
        """The device's integer for a code (temp: 2762)."""
        c = self._known(code)
        st, data = self._call('v', c.name.encode())
        if st == NO_VALUE: raise AttributeError('%s is a group: it has no value' % code)
        if st != OK: raise LinkError('get %s: status %d' % (code, st))
        return struct.unpack('<i', data)[0]

    def get(self, code):
        """The value of a code, scaled by the description."""
        return self._known(code).to_value(self.raw(code))

    def set(self, code, value):
        """Set a code: refused here when it is read-only or outside the range the description gives; the device checks again."""
        c = self._known(code)
        if c.kind == 'group': raise AttributeError('%s is a group: set its parts' % code)
        if c.ro: raise ReadOnly('%s is read-only' % code)
        raw = c.to_raw(value)
        if (c.lo is not None and raw < c.lo) or (c.hi is not None and raw > c.hi):
            raise OutOfRange('%s: %r is outside %d..%d' % (code, value, c.lo, c.hi))
        st, _ = self._call('w', struct.pack('<i', raw) + c.name.encode())
        if st == BAD_VALUE: raise OutOfRange('%s: the device refused %r' % (code, value))
        if st == READ_ONLY: raise ReadOnly('%s is read-only on the device' % code)
        if st != OK: raise LinkError('set %s: status %d' % (code, st))

    def changes(self):
        """What changed since the last call, oldest first. `missed` counts what the device could not queue (its store was full)."""
        st, data = self._call('n')
        if st != OK or len(data) < 1 or (len(data) - 1) % 5: raise LinkError('changes: status %d, %d bytes' % (st, len(data)))
        object.__setattr__(self, 'missed', self.missed + data[0])
        out = []
        for i in range(1, len(data), 5):
            num, raw = data[i], struct.unpack_from('<i', data, i + 1)[0]
            c = self._codes[num] if num < len(self._codes) else None
            out.append(Change(c.name if c else '#%d' % num, c.to_value(raw) if c else raw, raw))
        return out

    def fault(self, key):
        """A fault for the rig (the sketch's own keys: x resets the sensor behind the host's back, v and p reset the RFID reader)."""
        st, _ = self._call('f', key.encode() if isinstance(key, str) else bytes(key))
        if st != OK: raise LinkError('fault %r: status %d' % (key, st))

    def _known(self, code):
        try: return self.codes[code]
        except KeyError: raise UnknownCode('no code %r (the device has %s)' % (code, ', '.join(self.codes)))

    def _access(self, name):
        c = self.codes.get(name)
        if c is not None and c.kind != 'group': return self.get(name)
        if c is not None or any(k.startswith(name + '/') for k in self.codes): return Group(self, name)
        raise UnknownCode('no code %r (the device has %s)' % (name, ', '.join(self.codes)))

    def __getattr__(self, name):                       # m.temp, m.air
        if name.startswith('_') or name in ('codes', 'description', 'link', 'missed'): raise AttributeError(name)
        return self._access(name)
    def __setattr__(self, name, value):
        if name.startswith('_') or name in ('link', 'missed'): object.__setattr__(self, name, value)
        else: self.set(name, value)
    def __dir__(self): return sorted({c.name.split('/')[0] for c in self._codes})
