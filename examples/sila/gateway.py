#!/usr/bin/env python3
"""A SiLA 2 server for a OneMachine machine whose roles are switches, plain lights, discrete integers, selects, analog inputs, text, scaled values and actions.

    python3 gateway.py --port /dev/ttyUSB0       a Nano (or any board running the role::Link sketch) on its USB serial port, 115200
    python3 gateway.py --sim host/machine        a host build of the machine, over a pipe (no board needed)
    python3 gateway.py ... --generate            write the feature definitions to --fdl-dir (default ./fdl) and stop

What it knows about role kinds is one table, KINDS: a kind -> the SiLA words for it. A role of any other kind is refused, naming the kind; so is a
tuned role (tuning is not in the table). What a kind says about a role (the fields it reports and commands, its bounds, values, labels, scale, unit) is read
through python/onemachine (role_facts), the same for every consumer. The feature definitions (FDL) are generated at start, from the description (role names,
field types, params) and the table:

    switch   Feature <Role>   Property On     Boolean, observable   <- the report's `on`
                              Command  SetOn(On: Boolean)                   -> the command's `on`
    light    Feature <Role>   Property Level  Integer, observable   <- the report's `level`
                              Command  SetLevel(Level: Integer in [0, max])  -> the command's `level`
    discrete Feature <Role>   Property Value  Integer, observable   <- the report's `value`
                              Command  SetValue(Value: Integer in Set{...})  -> the command's `value`; the set is the role's `value` lines
    select   Feature <Role>   Property Value  String, observable, Set{labels}  <- the report's `index`, shown as the label of its `value` line
                              Command  SetValue(Value: String in Set{labels}) -> the command's `index`: the raw of the label's `value` line
    analog   Feature <Role>   Property <Q>   Real, observable, Unit  <- the report's `raw`, presented = raw * num / den (the role's `scale` line);
                              no command. The unit is the role's `unit` line, looked up in UNITS (symbol -> SI components and the property's name <Q>).
    text     Feature <Role>   Property Text   String, observable, MaximalLength N, Pattern [ -~]*  <- the report's u8[N] up to the first NUL
                              Command  SetText(Text: String, the same constraints) -> the command's u8[N]: the ASCII bytes, NUL-padded to N
    scaled   Feature <Role>   Property Value  Real, observable, Unit  <- the report's `raw`, presented = raw * num / den (the role's `scale` line)
                              Command  SetValue(Value: Real in [0, max*num/den], Unit)  -> the command's `raw`: the nearest integer to value*den/num,
                              halves away from zero, computed from the shortest decimal of the client's double with integer arithmetic (Facts.to_raw in python/onemachine);
                              the report is what the device holds, so the actual value shows. max is the role's `param <role> max`.
    action   Feature <Role>   Property Fired  Integer, observable   <- the report's `fired` (how many times the action ran since boot)
                              Command  Fire()                               -> the command's `fire` is 1 for that one frame, 0 in every other
    A role's description may have the line types its row lists in `lines` and no others (a switch with a `unit` line is refused, naming the line).
    all: no responses, no defined errors, plus SiLAService from the library. The kind's `safe` parameter and the report's `live` are not mapped.

The light's row says "`max` bounds `level`"; the value is the machine's own `param <role> max` line, and the field's type and width come from the
command and report descriptions. A set above the bound is a validation error from the server's own constraint check, before anything reaches the
machine. A command is forwarded once (m.push()); nothing is held or resent. Opening a Nano's serial port resets it (DTR): the outputs start off at every connect.
Needs sila2 and pyserial (requirements.txt), and python/onemachine from this checkout (../../python, or ONEMACHINE_PY=<dir>)."""
import argparse, atexit, logging, os, re, shutil, signal, sys, tempfile, threading, time, uuid
from decimal import Decimal
from fractions import Fraction
from queue import Queue
from xml.sax.saxutils import escape, quoteattr

HERE = os.path.dirname(os.path.abspath(__file__))
ONEMACHINE_PY = os.environ.get('ONEMACHINE_PY') or os.path.join(HERE, '..', '..', 'python')
if not os.path.isdir(os.path.join(ONEMACHINE_PY, 'onemachine')): sys.exit('gateway: python/onemachine not found at %s: run from a OneMachine checkout or set ONEMACHINE_PY' % ONEMACHINE_PY)
sys.path.insert(0, ONEMACHINE_PY)
from onemachine import Machine, StreamLink, SchemaError, LinkError, KindError, role_facts

ORIGINATOR = 'io.github.internetofpins'            # in every fully qualified feature identifier
SERVER_TYPE, SERVER_NAME = 'OneMachineGateway', 'OneMachine SiLA example'
VENDOR_URL = 'https://github.com/InternetOfPins/OneMachine'
NS = 'http://www.sila-standard.org'

class Refused(ValueError):
    """The machine has something this gateway has no SiLA words for; it does not guess."""

# the whole of what this gateway knows about role kinds: kind -> the SiLA words (the type it makes, the property, the command and its parameter). The description's field
# names and the bounds come from python/onemachine's facts for the kind.
KINDS = {
    'switch': dict(type='bool',     prop='On',    cmd='SetOn',    param='On'),
    'light':  dict(type='int',      prop='Level', cmd='SetLevel', param='Level'),
    'discrete': dict(type='discrete', prop='Value', cmd='SetValue', param='Value'),
    'select': dict(type='select',   prop='Value', cmd='SetValue', param='Value'),
    'analog': dict(type='analog',   prop=None,    cmd=None,       param=None),                  # report only; the property's name is the unit's
    'text':   dict(type='text',     prop='Text',  cmd='SetText',  param='Text'),
    'scaled': dict(type='scaled',   prop='Value', cmd='SetValue', param='Value'),
    'action': dict(type='action',   prop='Fired', cmd='Fire'),
}
# unit symbol -> the SiLA Unit constraint (SI base components; factor and offset to the SI unit) and the name of the property that carries it. Only V.
UNITS = {'%': dict(prop='Percent', label='%', factor='0.01', offset=0, components=(('Dimensionless', 1),)),
         'V': dict(prop='Voltage', label='V', factor=1, offset=0, components=(('Kilogram', 1), ('Meter', 2), ('Second', -3), ('Ampere', -1)))}
TEXT_PATTERN = '[ -~]*'                                        # printable ASCII, 0x20..0x7e: what the device keeps (a byte buffer) and a person can read

def decimal_str(x):
    """a Fraction as an exact finite decimal string, or None (the SiLA constraint is text: it must say exactly what the bound is)"""
    d = Decimal(x.numerator) / Decimal(x.denominator)
    return format(d.normalize(), 'f') if Fraction(d) == x else None

def camel(name):
    if not re.fullmatch(r'[A-Za-z0-9_]+', name) or '' in name.split('_'): raise Refused('role name %r has no SiLA identifier' % name)
    ident = ''.join(t[0].upper() + t[1:] for t in name.split('_'))
    if not re.fullmatch(r'[A-Z][A-Za-z0-9]*', ident): raise Refused('role name %r gives %r, not a SiLA identifier' % (name, ident))
    return ident

def model(m):
    """[(role name, feature identifier, kind, row, bounds)] for every role of the machine, or Refused. bounds = what the kind bounds the command with, in the shape fdl() reads."""
    out, seen = [], {'SiLAService': 'the library'}
    for name, info in m.roles.items():
        if info.kind not in KINDS: raise Refused('role %s is of kind %r; this gateway knows %s' % (name, info.kind, ', '.join(sorted(KINDS))))
        if info.tuned: raise Refused('role %s (%s) is tuned: tuning is not in this gateway\'s table' % (name, info.kind))
        try: f = role_facts(m, name)
        except KindError as e: raise Refused(str(e))
        k = dict(KINDS[info.kind], report=f.report, command=f.command, facts=f)
        t, bounds = k['type'], None
        if t == 'int': bounds = (f.low, f.high)
        elif t == 'discrete': bounds = f.values
        elif t == 'select': bounds = tuple(zip(f.values, f.labels))
        elif t in ('analog', 'scaled'):
            if f.unit not in UNITS: raise Refused('role %s (%s): unit symbol %r is not in this gateway\'s unit table (%s)' % (name, info.kind, f.unit, ', '.join(sorted(UNITS))))
            num, den = f.scale
            if t == 'analog':
                k['prop'] = UNITS[f.unit]['prop']
                bounds = (num, den, f.unit)
            else:
                lo_s, hi_s = decimal_str(Fraction(0)), decimal_str(f.presented(f.high))
                if hi_s is None: raise Refused('role %s (%s): the presented bound %d*%d/%d is not a finite decimal' % (name, info.kind, f.high, num, den))
                bounds = (num, den, f.unit, f.high, lo_s, hi_s)
        elif t == 'text': bounds = (f.size,)
        ident = camel(name)
        if ident in seen: raise Refused('roles %r and %r are both the feature %s' % (seen[ident], name, ident))
        seen[ident] = name
        out.append((name, ident, info.kind, k, bounds))
    if not out: raise Refused('the machine has no roles')
    return out

def fdl(name, ident, kind, k, bounds):
    d = lambda tag, s: '<%s>%s</%s>' % (tag, escape(s), tag)
    head = lambda i, disp, desc, ind: ''.join('%s%s\n' % (ind, d(t, v)) for t, v in (('Identifier', i), ('DisplayName', disp), ('Description', desc)))
    integer = '<DataType><Basic>Integer</Basic></DataType>'
    if k['type'] == 'bool': word = ptype = '<DataType><Basic>Boolean</Basic></DataType>'; tdesc = 'bool'
    elif k['type'] == 'select':
        word = ptype = ('<DataType><Constrained><DataType><Basic>String</Basic></DataType><Constraints><Set>%s</Set></Constraints></Constrained></DataType>'
                        % ''.join('<Value>%s</Value>' % escape(label) for _, label in bounds)); tdesc = 'label'
    elif k['type'] == 'text':
        word = ptype = ('<DataType><Constrained><DataType><Basic>String</Basic></DataType><Constraints><MaximalLength>%d</MaximalLength><Pattern>%s</Pattern></Constraints></Constrained></DataType>'
                        % (bounds[0], escape(TEXT_PATTERN))); tdesc = 'text'
    elif k['type'] == 'scaled':
        u = UNITS[bounds[2]]; tdesc = 'real'
        unit = ('<Unit><Label>%s</Label><Factor>%s</Factor><Offset>%d</Offset>%s</Unit>'
                % (escape(u['label']), u['factor'], u['offset'], ''.join('<UnitComponent><SIUnit>%s</SIUnit><Exponent>%d</Exponent></UnitComponent>' % c for c in u['components'])))
        real = '<DataType><Basic>Real</Basic></DataType>'
        word = '<DataType><Constrained>%s<Constraints>%s</Constraints></Constrained></DataType>' % (real, unit)
        ptype = ('<DataType><Constrained>%s<Constraints><MinimalInclusive>%s</MinimalInclusive><MaximalInclusive>%s</MaximalInclusive>%s</Constraints></Constrained></DataType>'
                 % (real, bounds[4], bounds[5], unit))
    elif k['type'] == 'analog':
        u = UNITS[bounds[2]]; ptype = None; tdesc = 'real'
        word = ('<DataType><Constrained><DataType><Basic>Real</Basic></DataType><Constraints><Unit><Label>%s</Label><Factor>%d</Factor><Offset>%d</Offset>%s</Unit></Constraints></Constrained></DataType>'
                % (escape(u['label']), u['factor'], u['offset'], ''.join('<UnitComponent><SIUnit>%s</SIUnit><Exponent>%d</Exponent></UnitComponent>' % c for c in u['components'])))
    else:
        word, tdesc = integer, 'integer'
        if k['type'] == 'int':
            ptype = ('<DataType><Constrained><DataType><Basic>Integer</Basic></DataType><Constraints><MinimalInclusive>%d</MinimalInclusive>'
                     '<MaximalInclusive>%d</MaximalInclusive></Constraints></Constrained></DataType>' % bounds)
        elif k['type'] == 'discrete':
            ptype = ('<DataType><Constrained><DataType><Basic>Integer</Basic></DataType><Constraints><Set>%s</Set></Constraints></Constrained></DataType>'
                     % ''.join('<Value>%d</Value>' % v for v in bounds))
        else: ptype = None
    if k['type'] == 'analog': command = ''                                                      # report only: no command
    else:
        command = '  <Command>\n' + head(k['cmd'], k['cmd'], '%s/%s %s' % (name, k['command'], tdesc if ptype else 'flag'), '    ') + '    <Observable>No</Observable>\n'
        if ptype: command += '    <Parameter>\n' + head(k['param'], k['command'], '%s/%s %s' % (name, k['command'], tdesc), '      ') + '      ' + ptype + '\n    </Parameter>\n'
        command += '  </Command>\n'
    return ('<?xml version="1.0" encoding="utf-8" ?>\n'
            '<Feature xmlns="%s" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" Category=%s FeatureVersion="1.0" MaturityLevel="Draft" Originator=%s SiLA2Version="1.0" '
            'xsi:schemaLocation="%s https://gitlab.com/SiLA2/sila_base/raw/master/schema/FeatureDefinition.xsd">\n' % (NS, quoteattr(kind), quoteattr(ORIGINATOR), NS)
            + head(ident, name, 'role %s %s' % (name, kind), '  ')
            + command
            + '  <Property>\n' + head(k['prop'], k['report'], '%s/%s %s' % (name, k['report'], tdesc), '    ') + '    <Observable>Yes</Observable>\n    ' + word + '\n  </Property>\n'
            + '</Feature>\n')

def write_fdl(models, directory):
    os.makedirs(directory, exist_ok=True)
    paths = {}
    for name, ident, kind, k, bounds in models:
        path = os.path.join(directory, '%s.sila.xml' % ident)
        with open(path, 'w', encoding='utf-8') as f: f.write(fdl(name, ident, kind, k, bounds))
        paths[ident] = path
    return paths

class Traced:
    """Wraps a link and writes every request and response: op, payload hex, status, payload hex (checked from outside by frame_decode.py),
    then the host time (seconds since the gateway started) at which the request is written to the port, and how long the exchange took."""
    def __init__(self, link, path): self.link, self.f, self.t0 = link, open(path, 'w', buffering=1), time.monotonic()
    def call(self, op, payload=b''):
        t = time.monotonic() - self.t0
        st, data = self.link.call(op, payload)
        self.f.write('%s %s %d %s %.4f %.4f\n' % (op if isinstance(op, str) else chr(op), bytes(payload).hex() or '-', st, data.hex() or '-', t, time.monotonic() - self.t0 - t))
        return st, data
    def close(self): self.f.close(); self.link.close()

def open_link(a):
    if a.sim: link = StreamLink.popen([a.sim])
    else:
        import serial
        port = serial.Serial(a.port, 115200, timeout=2)
        time.sleep(2); port.reset_input_buffer()                   # opening the port reset the board: wait for its bootloader to pass
        link = StreamLink(port.read, port.write, port.flush)
    return Traced(link, a.trace) if a.trace else link

def serve(a, m, models, paths):
    import sila2.server, sila2.framework
    from sila2.framework import Feature
    lock = threading.RLock()
    server_uuid = str(uuid.uuid5(uuid.NAMESPACE_URL, 'urn:%s:%s:%08x' % (ORIGINATOR, SERVER_TYPE, m.description.hash)))     # the same firmware is always the same server
    server = sila2.server.SilaServer(SERVER_NAME, SERVER_TYPE, 'machine hash %08x' % m.description.hash, '1.0', VENDOR_URL, server_uuid)
    impls = {}
    for name, ident, kind, k, bounds in models:
        class Impl(sila2.server.FeatureImplementationBase): pass
        impl = Impl(server)
        setattr(impl, '_%s_producer_queue' % k['prop'], Queue())
        setattr(impl, 'update_' + k['prop'], (lambda v, impl=impl, p=k['prop']: (setattr(impl, '_%s_current_value' % p, v), getattr(impl, '_%s_producer_queue' % p).put(v))))
        setattr(impl, k['prop'] + '_on_subscription', lambda *, metadata: None)
        def setter(value, *, metadata, name=name, k=k):
            f = k['facts']
            with lock:                                                # one frame at a time: a frame carries every role's command
                if k['type'] == 'text': v = f.pack_text(value)                # the SiLA edge (Pattern, MaximalLength) has already refused anything else; this is not trusted
                elif k['type'] == 'scaled': v = f.to_raw(value)
                else: v = bool(value) if k['type'] == 'bool' else f.raw_of(value) if k['type'] == 'select' else int(value)
                setattr(getattr(m.cmd, name), k['command'], v)
                m.push()                                              # once
        def fire(*, metadata, name=name, k=k):
            with lock:                                                # the flag is 1 only here, during one push, and 0 in every other frame
                layer = getattr(m.cmd, name)
                setattr(layer, k['command'], 1)
                try: m.push()                                         # once
                finally: setattr(layer, k['command'], 0)
        if k['cmd']: setattr(impl, k['cmd'], fire if k['type'] == 'action' else setter)
        impl.last = None
        impls[name] = impl
        server.set_feature_implementation(Feature(paths[ident]), impl)
    def publish():
        with lock: r = m.poll()
        for name, ident, kind, k, bounds in models:
            v = getattr(getattr(r, name), k['report'])
            if k['type'] == 'bool': v = bool(v)
            elif k['type'] == 'select': v = k['facts'].label_of(v)
            elif k['type'] in ('analog', 'scaled'): v = float(k['facts'].presented(v))
            elif k['type'] == 'text': v = k['facts'].unpack_text(v)
            else: v = int(v)
            if impls[name].last != v: impls[name].last = v; getattr(impls[name], 'update_' + k['prop'])(v)
    publish()
    server.start_insecure(a.address, a.sila_port, enable_discovery=False)
    print('serving %s on %s:%d, server %s, machine hash %08x' % (', '.join(i for _, i, _, _, _ in models), a.address, a.sila_port, server.server_uuid, m.description.hash), flush=True)
    try:
        while True:
            time.sleep(a.poll)
            try: publish()
            except (LinkError, OSError) as e: logging.getLogger('gateway').error('the machine does not answer: %s', e); time.sleep(1)
    except KeyboardInterrupt: pass
    finally: server.stop(0.5)

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--port', metavar='SERIAL'); src.add_argument('--sim', metavar='HOST_BINARY')
    ap.add_argument('--fdl-dir', help='where the feature definitions are written (default: ./fdl with --generate, else a temporary directory)'); ap.add_argument('--generate', action='store_true')
    ap.add_argument('--address', default='127.0.0.1'); ap.add_argument('--sila-port', type=int, default=50052)
    ap.add_argument('--poll', type=float, default=0.1, help='seconds between report polls'); ap.add_argument('--trace', metavar='FILE')
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(name)s %(message)s')
    try: m = Machine(open_link(a))
    except SchemaError as e: sys.exit('gateway: the binding refuses this machine: %s' % e)
    except (LinkError, OSError) as e: sys.exit('gateway: cannot reach the machine: %s' % e)
    try: models = model(m)
    except Refused as e: sys.exit('gateway: refused: %s' % e)
    if not a.fdl_dir:
        if a.generate: a.fdl_dir = 'fdl'
        else: a.fdl_dir = tempfile.mkdtemp(prefix='sila-fdl-'); atexit.register(shutil.rmtree, a.fdl_dir, True)
    paths = write_fdl(models, a.fdl_dir)
    for p in paths.values(): print('wrote', p)
    if a.generate: return
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))                 # a kill stops the server and removes the temporary directory
    serve(a, m, models, paths)

if __name__ == '__main__':
    main()
