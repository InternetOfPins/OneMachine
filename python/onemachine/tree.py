"""A machine tree seen by its codes: the description the device publishes, and nothing else a consumer needs (examples/spi, link/frame.h).

    m = Tree(StreamLink(ser.read, ser.write, ser.flush))   # a serial port; CtypesLink('./tree.so') in-process; or any object with call(op, payload)
    m.codes['temp'].scaled, m.codes['air/ctrl_meas'].lo    # what the description says: 2, 0
    m.temp                                                 # 27.62: the value, scaled by the description (temp is in 0.01 C)
    m.air.ctrl_meas = 0x27                                 # set by code: read-only and out-of-range are refused here, before anything is sent
    m.air.ctrl_meas                                        # 39: read back from the device
    m.changes()                                            # [Change('temp', 27.61, 2761), Change('air', status='stale'), ...]: what changed since the last call
    m.status('air')                                        # 'alive', 'stale' or 'gone': the row the code is bound to (the part's, not a register that reads 0xFF)
    m.reading('temp')                                      # Reading(value=27.61, status='stale'): never raises; m.temp raises Stale when the part is not alive
    m.missed                                               # how many events the device had to refuse (its event queue was full) since the start
    m.resyncs                                              # how often the device sent every code because a reply was lost

A device may describe itself by hash only (the default of examples/spi): `d` answers the hash and each code's status, and the text is read from the
build output, `Tree(link, descriptions=dir)`, where examples/spi/describe.cpp wrote <hash>.txt from the same types (a device built with
-DONEMACHINE_DESC_TEXT sends the text itself).
A code is a name, with `/` for the part of a group (`air/ctrl_meas` is `m.air.ctrl_meas`). The description lists, per code: its path in the
machine (`<bus>/<address>/<node>[/<child>]`, for people), whether it notifies (`sync`: a value that moves; `event`), read-only or read-write, its
kind (a value, a register with its default, a group) and, for a value, how many decimals it is scaled by and the range a set accepts.
A part that is not alive is Stale or Gone: reading one of its codes raises Stale (it carries the last value), and the change is announced by changes()
as one status change per code of that part (the device sends it once, through the first of them).
State and events are told apart. A state code (a value, a register, a row's status) is one pending bit on the device: changes() gets each code
that changed once, with its value as it is now, however often it changed in between; it cannot overflow. An event code (the card) is queued, and a
device that refused events (`missed`) says how many. changes() sends the sequence number of the last reply it got; a device that answered since
(a reply that was lost) sends every code again (`resyncs` counts it)."""
import os, struct
from collections import namedtuple
from .machine import LinkError

OK, BAD_HASH, BAD_LENGTH, BAD_VALUE, NO_VALUE, UNKNOWN, TOO_LONG, READ_ONLY = 0, 1, 2, 3, 4, 0x80, 0x81, 0x82

class OutOfRange(ValueError): pass
class ReadOnly(AttributeError): pass
class UnknownCode(AttributeError): pass

Change = namedtuple('Change', 'code value raw status', defaults=(None, None, None))     # a value change has value and raw; a status change only status
Reading = namedtuple('Reading', 'value status')
STATUS = ('alive', 'stale', 'gone')

class Stale(RuntimeError):
    """A code of a part that is not alive was read: status says which, last is the last value (a register's last set, a sensor's last reading)."""
    def __init__(self, code, status, last): super().__init__('%s: the part is %s (last value %r)' % (code, status, last)); self.code, self.status, self.last = code, status, last

class Code:
    def __init__(self, num, name, path, notify):
        self.num, self.name, self.path, self.notify = num, name, path, notify
        self.kind, self.ro, self.scaled, self.lo, self.hi, self.default, self.group_size, self.unit, self.status = 'value', True, 0, None, None, None, 0, None, 'alive'
        self.src = tuple(path[:2])           # the row it is bound to: <bus>/<address>
    def __repr__(self): return 'Code(%d %s %s%s)' % (self.num, self.name, self.kind, ' ro' if self.ro else ' rw')
    def to_value(self, raw):
        if self.unit == 'u32': return raw & 0xFFFFFFFF
        return raw / 10 ** self.scaled if self.scaled else raw
    def to_raw(self, value): return int(round(value * 10 ** self.scaled)) if self.scaled else int(value)

def _fnv(data, h=2166136261):
    for b in data: h = ((h ^ b) * 16777619) & 0xFFFFFFFF
    return h

def wiring(text):
    """The `wiring` section of a description: {'board': name, <line>: {role: (gpio, drive), ...}, ...} (bus lines `i2c`, `spi`; parts by name, with
    `driver`; `at` for an address). Empty when the description has none (the text a device sends; the build output by hash has it)."""
    out, seen = {}, False
    for line in text.split('\n'):
        if line.startswith('wiring '): seen = True; out['board'] = line[7:]; continue
        if not seen or not line.startswith('  '): continue
        w = line.split()
        name, rest, entry = w[0], w[1:], {}
        if rest and rest[0] not in ('sda', 'scl', 'sck', 'miso', 'mosi', 'cs', 'rst', 'irq', 'at'): entry['driver'], rest = rest[0], rest[1:]
        i = 0
        while i < len(rest):
            if rest[i] == 'at': entry['at'] = int(rest[i + 1], 16); i += 2
            else: entry[rest[i]] = (int(rest[i + 1]), rest[i + 2]); i += 3
        out[name] = entry
    return out

def _parse(text):
    """The `published` section: one line per code, in the order the device numbers them."""
    codes, seen = [], False
    for line in text.split('\n'):
        if line == 'published': seen = True; continue
        if seen and line and not line.startswith('  '): break               # the next section (the wiring): the codes are done
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
            elif t == 'status': c.status, i = w[i + 1], i + 2
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
    def __init__(self, link, descriptions=None):
        object.__setattr__(self, 'link', link)
        object.__setattr__(self, 'descriptions', [descriptions] if isinstance(descriptions, str) else list(descriptions or []))
        object.__setattr__(self, 'missed', 0)
        object.__setattr__(self, 'resyncs', 0)
        object.__setattr__(self, '_seq', 0)
        object.__setattr__(self, '_last', {})        # the value changes() last gave per code
        object.__setattr__(self, '_told', {})        # the status changes() last gave per row
        self.refresh()

    def _call(self, op, payload=b''):
        return self.link.call(op, payload)

    def refresh(self):
        """Read the description again."""
        st, data = self._call('d')
        if st != OK: raise LinkError('description: status %d' % st)
        text = data.decode()
        statuses, h = None, None
        if text.startswith('hash '):                                          # the description by its hash: the text is in the build output
            head, digits = text.split('\n')[:2]
            h = int(head[5:], 16)
            text = self._load(h)
            statuses = [STATUS[int(d)] if int(d) < len(STATUS) else 'gone' for d in digits]
        codes = _parse(text)
        if statuses is not None:
            if len(statuses) != len(codes): raise LinkError('description %08x: %d codes, the device has %d' % (h, len(codes), len(statuses)))
            for c, st in zip(codes, statuses): c.status = st
        object.__setattr__(self, 'hash', h)                                 # the description's hash: codes go by number under it (None: by name)
        object.__setattr__(self, 'description', text)
        object.__setattr__(self, '_codes', codes)
        object.__setattr__(self, 'codes', {c.name: c for c in codes})
        object.__setattr__(self, '_status', {c.src: c.status for c in codes})      # by row: what the description said, then what the device tells

    def _load(self, h):
        """The text of the description whose hash is h, from the build output (examples/spi/describe.cpp writes <hash>.txt)."""
        name = '%08x.txt' % h
        for d in self.descriptions:
            p = os.path.join(d, name)
            if os.path.isfile(p):
                text = open(p).read()
                if _fnv(text.encode()) != h: raise LinkError('%s: its text does not hash to %08x' % (p, h))
                return text
        raise LinkError('the device describes itself by hash %08x: no %s in %s (give Tree(link, descriptions=<the build\'s description dir>), or build the device with -DONEMACHINE_DESC_TEXT)' % (h, name, self.descriptions or 'no directory'))

    def _key(self, c):
        """A code in a request: u32 the description's hash and u8 its number (a device described by hash), or its name."""
        return struct.pack('<IB', self.hash, c.num) if self.hash is not None else c.name.encode()

    def _read(self, code):
        """(status, raw) from the device: raw is None for a group or an event. The status is the part's; a part that is not alive answers its last value."""
        c = self._known(code)
        st, data = self._call('v', self._key(c))
        if st == BAD_HASH: raise LinkError('get %s: the device is another build (its description changed): refresh()' % code)
        if st not in (OK, NO_VALUE) or len(data) < 1: raise LinkError('get %s: status %d' % (code, st))
        status = STATUS[data[0]] if data[0] < len(STATUS) else 'gone'
        self._status[c.src] = status
        return status, (struct.unpack_from('<i', data, 1)[0] if st == OK else None)

    def raw(self, code):
        """The device's integer for a code (temp: 2762), whatever the part's status (not alive: the last value)."""
        status, raw = self._read(code)
        if raw is None: raise AttributeError('%s has no value' % code)
        return raw

    def reading(self, code):
        """Reading(value, status), scaled by the description; never raises for a part that is not alive."""
        c = self._known(code)
        status, raw = self._read(code)
        return Reading(c.to_value(raw) if raw is not None else None, status)

    def get(self, code):
        """The value of a code, scaled by the description. Stale when its part is not alive."""
        r = self.reading(code)
        if r.status != 'alive': raise Stale(code, r.status, r.value)
        if r.value is None: raise AttributeError('%s has no value' % code)
        return r.value

    def status(self, code, refresh=False):
        """'alive', 'stale' or 'gone': the row the code is bound to, as last told (the description, a read, a status change from changes()); refresh=True asks the device."""
        c = self._known(code)
        if refresh: self._read(code)
        return self._status[c.src]

    def set(self, code, value):
        """Set a code: refused here when it is read-only or outside the range the description gives; the device checks again."""
        c = self._known(code)
        if c.kind == 'group': raise AttributeError('%s is a group: set its parts' % code)
        if c.ro: raise ReadOnly('%s is read-only' % code)
        raw = c.to_raw(value)
        if (c.lo is not None and raw < c.lo) or (c.hi is not None and raw > c.hi):
            raise OutOfRange('%s: %r is outside %d..%d' % (code, value, c.lo, c.hi))
        st, _ = self._call('w', struct.pack('<i', raw) + self._key(c))
        if st == BAD_VALUE: raise OutOfRange('%s: the device refused %r' % (code, value))
        if st == READ_ONLY: raise ReadOnly('%s is read-only on the device' % code)
        if st == BAD_HASH: raise LinkError('set %s: the device is another build (its description changed): refresh()' % code)
        if st != OK: raise LinkError('set %s: status %d' % (code, st))

    def changes(self):
        """What changed since the last call: each state code that changed, once, with its value now; the events, oldest first. `missed` counts the
        events the device could not queue; `resyncs` counts the replies that carried every code (the previous reply did not arrive)."""
        out = []
        while True:
            st, data = self._call('n', struct.pack('<H', self._seq))
            if st != OK or len(data) < 4 or (len(data) - 4) % 5: raise LinkError('changes: status %d, %d bytes' % (st, len(data)))
            seq, flags, drops = struct.unpack_from('<HBB', data, 0)
            object.__setattr__(self, '_seq', seq)
            object.__setattr__(self, 'missed', self.missed + drops)
            if flags & 1: object.__setattr__(self, 'resyncs', self.resyncs + 1)
            told = set()                                                          # the rows whose status this reply announced
            for i in range(4, len(data), 5):
                kind, num, raw = data[i] >> 6, data[i] & 0x3F, struct.unpack_from('<i', data, i + 1)[0]
                c = self._codes[num] if num < len(self._codes) else None
                if c is None: out.append(Change('#%d' % num, raw, raw)); continue
                if kind == 3: out.append(Change(c.name, c.to_value(raw), raw)); continue     # an event: one occurrence
                status = STATUS[kind]
                moved = status != self._told.get(c.src, c.status)
                if (flags & 1 or moved) and c.src not in told:                    # the row's status: every code of that part
                    told.add(c.src); self._status[c.src] = self._told[c.src] = status
                    out.extend(Change(k.name, status=status) for k in self._codes if k.src == c.src)
                # the value: the bit was set by the value (the status did not move), or the status moved and the value with it, or a resync
                if c.kind != 'group' and c.notify != 'event' and (flags & 1 or not moved or self._last.get(c.name) != raw):
                    self._last[c.name] = raw
                    out.append(Change(c.name, c.to_value(raw), raw))
            if not flags & 2: return out

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
        if name.startswith('_') or name in ('codes', 'description', 'descriptions', 'hash', 'link', 'missed', 'resyncs'): raise AttributeError(name)
        return self._access(name)
    def __setattr__(self, name, value):
        if name.startswith('_') or name in ('link', 'missed', 'resyncs', 'descriptions'): object.__setattr__(self, name, value)
        else: self.set(name, value)
    def __dir__(self): return sorted({c.name.split('/')[0] for c in self._codes})
