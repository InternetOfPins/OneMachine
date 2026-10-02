"""A OneMachine machine seen from its roles: the only names a consumer uses (role/role.h).

    m = Machine(StreamLink.popen(['./device']))   # over a pipe or a serial port; CtypesLink('./machine.so') in-process;
                                                   # or any object with call(op, payload) -> (status, bytes)
    m.roles['x'].kind, m.roles['x'].params         # 'axis', {'steps_mm': 80, 'min_um': 0, 'max_um': 300000}
    m.refs                                         # the machine's `ref` lines (role/ref.h)
    m.cmd.x.target_um = 150000                     # the command, by role, range-checked
    m.push()                                       # sent; the device applies it at its next cycle boundary
    m.poll().x.pos_um                              # the report; `live` is False for a role whose device is not there
    m.roles['x'].tuned                             # a role::Tuned role: its parameters can change at run time
    m.tune.x.max_um = 200000; m.retune()           # within the firmware's limits (the `param` values); outside them: OutOfLimits

Where a role is (which device, which bus, which channel) is the device's business: the `at` lines are shown to people (m.where),
never used here. A consumer only reconfigures when the device's roles change: push() and poll() re-read the descriptions on BadHash
and keep every command value whose role and field still exist. A role this consumer has written that is gone, or has another kind,
raises RoleChanged instead of being retargeted."""
import struct, subprocess
from .schema import Schema, SchemaError, BadHash

OK, BAD_HASH, BAD_LENGTH, BAD_VALUE, UNKNOWN, TOO_LONG = 0, 1, 2, 3, 0x80, 0x81

class LinkError(IOError): pass
class RoleChanged(SchemaError): pass
class OutOfLimits(ValueError): pass

class StreamLink:
    """role/link.h over a byte stream: request op, u16 length, payload; response status, u16 length, payload."""
    def __init__(self, read, write, flush=lambda: None): self._read, self._write, self._flush = read, write, flush
    @classmethod
    def popen(cls, argv):
        p = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        link = cls(p.stdout.read, p.stdin.write, p.stdin.flush); link.process = p
        return link
    def call(self, op, payload=b''):
        self._write(bytes([ord(op) if isinstance(op, str) else op]) + struct.pack('<H', len(payload)) + bytes(payload)); self._flush()
        head = self._read(3)
        if len(head) != 3: raise LinkError('the device closed the link')
        st, n = struct.unpack('<BH', head)
        data = self._read(n) if n else b''
        if len(data) != n: raise LinkError('short response')
        return st, data
    def close(self):
        p = getattr(self, 'process', None)
        if p: p.stdin.close(); p.wait()

class CtypesLink:
    """role/call.h in the same process: a host build of the machine loaded as a shared library, called through its C ABI
    (<prefix>_call, <prefix>_cycle; ONEMACHINE_CALL_EXPORT). The same requests and responses as StreamLink, as a function call.
    autocycle: run the machine's cycle boundary after every call, on this process's clock (nothing else drives it in-process);
    False when the machine's owner drives it (a thread of its own, or a simulated clock)."""
    def __init__(self, path, prefix='onemachine', autocycle=True):
        import ctypes, time
        self._c, self._time = ctypes, time
        self.lib = ctypes.CDLL(path)
        self._call = getattr(self.lib, prefix + '_call')
        self._call.restype = ctypes.c_int32
        self._call.argtypes = [ctypes.c_uint8, ctypes.c_char_p, ctypes.c_uint16, ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint32]
        self._cycle = getattr(self.lib, prefix + '_cycle')
        self._cycle.restype = None
        self._cycle.argtypes = [ctypes.c_uint32]
        self.autocycle, self._t0, self._buf = autocycle, time.monotonic(), ctypes.create_string_buffer(512)
    def now(self): return int((self._time.monotonic() - self._t0) * 1000) & 0xFFFFFFFF
    def cycle(self): self._cycle(self.now())
    def call(self, op, payload=b''):
        op = ord(op) if isinstance(op, str) else op
        payload = bytes(payload)
        while True:
            r = self._call(op, payload, len(payload), self._buf, len(self._buf), self.now())
            if r >= 0: break
            self._buf = self._c.create_string_buffer(-r)          # a long reply (a description): ask again with room for it
        if self.autocycle: self.cycle()
        raw = self._buf.raw[:r]
        if r < 3: raise LinkError('short response')
        st, n = struct.unpack_from('<BH', raw)
        return st, raw[3:3 + n]
    def close(self): pass

def _fnv(h, data):
    for b in data: h = ((h ^ b) * 16777619) & 0xFFFFFFFF
    return h

def _carry(old, schema, new):
    """new, with the values of old copied in wherever old has the same layer and field (a description re-read keeps what still exists)."""
    if old is not None:
        for l in schema.layers:
            if not hasattr(old, l.name): continue
            for f in l.fields:
                v = getattr(getattr(old, l.name), f.name, None)
                if v is not None: getattr(new, l.name)._v[f.name] = v
    return new

class RoleInfo:
    def __init__(self, name, kind): self.name, self.kind, self.params, self.tuned = name, kind, {}, False
    def __repr__(self): return 'Role(%s %s %r%s)' % (self.name, self.kind, self.params, ' tuned' if self.tuned else '')

class Description:
    """The machine description (role/face.h): refs, roles with kind and parameters, and where each is (for people only)."""
    def __init__(self, text):
        lines = [l for l in text.split('\n') if l]
        if not lines or lines[0] != 'machine 1': raise SchemaError('not a machine 1 description')
        if not lines[1].startswith('hash '): raise SchemaError('no hash line')
        self.hash = int(lines[1][5:], 16)
        h = 2166136261
        self.refs, self.roles, self.where = [], {}, {}
        for l in lines[2:]:
            word, _, rest = l.partition(' ')
            if word == 'at':
                role, _, place = rest.partition(' '); self.where[role] = place; continue
            h = _fnv(h, (l + '\n').encode())
            if word == 'ref': self.refs.append(rest)
            elif word == 'role':
                name, kind = rest.split(' '); self.roles[name] = RoleInfo(name, kind)
            elif word == 'param':
                role, key, value = rest.split(' '); self.roles[role].params[key] = int(value)
            elif word == 'tune': self.roles[rest].tuned = True
            else: raise SchemaError('unknown line %r' % l)
        if h != self.hash: raise SchemaError('description says hash %08x, its lines hash to %08x' % (self.hash, h))

class Machine:
    def __init__(self, link):
        self.link = link
        self.written = set()                       # the roles this consumer has written: the ones it depends on
        self.cmd = None
        self.refresh()

    def _text(self, op):
        st, data = self.link.call(op)
        if st != OK: raise LinkError('%s: status %d' % (op, st))
        return data.decode()

    def refresh(self):
        """Read the three descriptions again; keep the command values of every role and field that still exist."""
        desc = Description(self._text('m'))
        report, command = Schema.parse(self._text('r')), Schema.parse(self._text('c'))
        old, old_desc = self.cmd, getattr(self, 'description', None)
        if old_desc is not None:
            gone = [r for r in self.written if r not in desc.roles]
            rekind = [r for r in self.written if r in desc.roles and desc.roles[r].kind != old_desc.roles[r].kind]
            if gone or rekind:
                raise RoleChanged('roles this consumer drives changed on the device: %s' %
                                  ', '.join(['%s gone' % r for r in gone] + ['%s is now a %s' % (r, desc.roles[r].kind) for r in rekind]))
        self.description, self.report_schema, self.command_schema = desc, report, command
        self.cmd = _carry(old, command, command.zero(on_set=lambda role, field: self.written.add(role)))
        self.report = None
        # tuned roles: the device's current values, then whatever this consumer had changed and not yet sent, by role and field name
        old_tune, self.tune_schema, self.tune = getattr(self, 'tune', None), None, None
        if any(r.tuned for r in desc.roles.values()):
            self.tune_schema = Schema.parse(self._text('T'))
            st, data = self.link.call('G')
            if st != OK: raise LinkError('tuning get: status %d' % st)
            self.tune = _carry(old_tune, self.tune_schema, self.tune_schema.decode(data))

    def relink(self, link):
        """Talk to the machine over another link (a reconnection, a reboot, new firmware). Nothing is re-read until the device says
        its roles changed (BadHash): a device with the same roles needs nothing from the consumer."""
        self.link = link

    @property
    def roles(self): return self.description.roles
    @property
    def refs(self): return self.description.refs
    @property
    def where(self): return dict(self.description.where)

    def push(self):
        """Send the command. On BadHash (the device's roles changed) re-read once and send again."""
        for attempt in (0, 1):
            st, _ = self.link.call('s', self.command_schema.encode(self.cmd))
            if st == OK: return
            if st == BAD_HASH and attempt == 0: self.refresh(); continue
            raise LinkError('set: status %d' % st)

    def retune(self):
        """Send the tuning (m.tune). It takes effect at once on the device, and the current command is applied again under it. A value
        outside the firmware's limits changes nothing on the device: OutOfLimits, and m.tune is read back from the device."""
        if self.tune is None: raise LinkError('this machine has no tuned role')
        for attempt in (0, 1):
            st, _ = self.link.call('S', self.tune_schema.encode(self.tune))
            if st == OK: return
            if st == BAD_HASH and attempt == 0: self.refresh(); continue
            if st == BAD_VALUE:
                bad = self.tune.to_dict()
                st2, data = self.link.call('G')
                if st2 == OK: self.tune = self.tune_schema.decode(data)
                raise OutOfLimits('tuning %r is outside the firmware limits %r' % (bad, {n: r.params for n, r in self.roles.items() if r.tuned}))
            raise LinkError('tuning set: status %d' % st)

    def poll(self):
        """Read the report. On a frame of another schema, re-read the descriptions once and read again."""
        for attempt in (0, 1):
            st, data = self.link.call('g')
            if st != OK: raise LinkError('get: status %d' % st)
            try:
                self.report = self.report_schema.decode(data); return self.report
            except BadHash:
                if attempt: raise
                self.refresh()

    def __getattr__(self, name):                   # m.x is the command of role x
        if name in ('cmd', 'description', 'tune', 'tune_schema') or name.startswith('_'): raise AttributeError(name)
        return getattr(self.cmd, name)
