"""A OneMachine state composition, built at run time from its self-description alone (the text state::describe<R>(put) prints).
No C++ knowledge and no generated file: the description is the binding.

    s = Schema.parse(text)      # checks the description's own hash and size
    st = s.decode(frame)        # refuses another schema's frame (BadHash) or a wrong length (BadLength)
    st.x.target_um = 1000       # attribute access, layer then field, range-checked on assignment
    frame = s.encode(st)        # the exact bytes state::read accepts
    s.json(st)                  # the same text as state::json
    s.stub('Net')               # a typed stub for an IDE

An independent implementation of state/wire.h; test/state's python oracles and test/role/check.py hold it to the C++ byte for byte."""
import struct, json as _json

_TYPES = {'bool': (1, '?', None)}                      # name -> (schema code, struct char, (lo, hi))
for _bits, (_u, _i) in {8: ('B', 'b'), 16: ('H', 'h'), 32: ('I', 'i'), 64: ('Q', 'q')}.items():
    _TYPES['u%d' % _bits] = (_bits // 4, _u, (0, (1 << _bits) - 1))
    _TYPES['i%d' % _bits] = (_bits // 4 + 1, _i, (-(1 << (_bits - 1)), (1 << (_bits - 1)) - 1))

def _fnv(h, data):
    for b in data: h = ((h ^ b) * 16777619) & 0xFFFFFFFF
    return h

class SchemaError(ValueError): pass
class BadHash(SchemaError): pass
class BadLength(SchemaError): pass

class Field:
    def __init__(self, name, base, n):
        self.name, self.base, self.n = name, base, n      # n = None for a scalar
        code, ch, self.range = _TYPES[base]
        self.fmt = '<' + ch * (n or 1)
    def check(self, v):
        vs = v if self.n else [v]
        if self.n and len(vs) != self.n: raise ValueError('%s: %d values, want %d' % (self.name, len(vs), self.n))
        for x in vs:
            if self.base == 'bool':
                if not isinstance(x, bool) and x not in (0, 1): raise ValueError('%s: %r is not a bool' % (self.name, x))
            elif not (self.range[0] <= int(x) <= self.range[1]): raise ValueError('%s: %r out of %s' % (self.name, x, self.base))

class Layer:
    def __init__(self, name): self.name, self.fields = name, []

class _Obj:
    """A layer's values, or the whole state: attribute access, only the declared names, range-checked on assignment."""
    __slots__ = ('_spec', '_v', '_on_set')
    def __init__(self, spec, values, on_set=None):
        object.__setattr__(self, '_spec', spec); object.__setattr__(self, '_v', values); object.__setattr__(self, '_on_set', on_set)
    def __getattr__(self, k):
        try: return self._v[k]
        except KeyError: raise AttributeError(k) from None
    def __setattr__(self, k, v):
        f = self._spec.get(k)
        if not isinstance(f, Field): raise AttributeError('%s is not a field here' % k)
        f.check(v); self._v[k] = list(v) if f.n else v
        if self._on_set: self._on_set(k)
    def __repr__(self): return repr(self.to_dict())
    def to_dict(self): return {k: (v.to_dict() if isinstance(v, _Obj) else v) for k, v in self._v.items()}

class Schema:
    def __init__(self, layers, hash_, size): self.layers, self.hash, self.size = layers, hash_, size

    @classmethod
    def parse(cls, text):
        lines = [l for l in text.split('\n') if l]
        if not lines or lines[0] != 'state 1': raise SchemaError('not a state 1 description')
        want_h = want_n = None; layers = []
        h, n = 2166136261, 4
        for l in lines[1:]:
            if l.startswith('hash '): want_h = int(l[5:], 16)
            elif l.startswith('size '): want_n = int(l[5:])
            elif l.startswith('layer '):
                layers.append(Layer(l[6:])); h = _fnv(_fnv(h, layers[-1].name.encode()), b'\xff')
            else:
                path, typ = l.split(' ')
                lay, name = path.split('/')
                if not layers or lay != layers[-1].name: raise SchemaError('%s is not under layer %s' % (path, layers[-1].name if layers else '-'))
                base, _, cnt = typ.partition('[')
                k = int(cnt.rstrip(']')) if cnt else None
                f = Field(name, base, k); layers[-1].fields.append(f)
                code = _TYPES[base][0]
                h = _fnv(_fnv(_fnv(h, name.encode()), b'\x00'), bytes([code | 0x80 if k else code]))
                if k: h = _fnv(h, bytes([k & 255, k >> 8]))
                n += struct.calcsize(f.fmt)
        if h != want_h or n != want_n:
            raise SchemaError('description says hash %08x size %s, computed %08x size %d' % (want_h or 0, want_n, h, n))
        return cls(layers, h, n)

    def zero(self, on_set=None):
        """The all-zero state. on_set(layer, field), if given, is called on every assignment to a field."""
        hook = (lambda name: (lambda k: on_set(name, k))) if on_set else (lambda name: None)
        return _Obj({l.name: l for l in self.layers},
                    {l.name: _Obj({f.name: f for f in l.fields},
                                  {f.name: ([0] * f.n if f.n else (False if f.base == 'bool' else 0)) for f in l.fields},
                                  hook(l.name))
                     for l in self.layers})

    def layer(self, name):
        for l in self.layers:
            if l.name == name: return l
        return None

    def decode(self, frame):
        frame = bytes(frame)
        if len(frame) < 4 or struct.unpack_from('<I', frame)[0] != self.hash:
            raise BadHash('frame is of another schema')
        if len(frame) != self.size: raise BadLength('%d bytes, want %d' % (len(frame), self.size))
        st, off = self.zero(), 4
        for l in self.layers:
            for f in l.fields:
                vs = struct.unpack_from(f.fmt, frame, off); off += struct.calcsize(f.fmt)
                getattr(st, l.name)._v[f.name] = list(vs) if f.n else vs[0]
        return st

    def encode(self, st):
        out = struct.pack('<I', self.hash)
        for l in self.layers:
            for f in l.fields:
                v = getattr(getattr(st, l.name), f.name); f.check(v)
                out += struct.pack(f.fmt, *(v if f.n else [v]))
        return out

    def json(self, st):
        return _json.dumps(st.to_dict(), separators=(',', ':'))

    def stub(self, name):
        py = lambda f: ('bool' if f.base == 'bool' else 'int') if not f.n else ('list[%s]' % ('bool' if f.base == 'bool' else 'int'))
        out = ['# generated from a OneMachine state description, schema %08x' % self.hash]
        for l in self.layers:
            out.append('class %s_%s:' % (name, l.name))
            out += ['    %s: %s    # %s%s' % (f.name, py(f), f.base, '[%d]' % f.n if f.n else '') for f in l.fields]
        out.append('class %s:' % name)
        out += ['    %s: %s_%s' % (l.name, name, l.name) for l in self.layers]
        return '\n'.join(out) + '\n'
