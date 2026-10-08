#!/usr/bin/env python3
"""An MCP server (stdio) for a OneMachine machine whose roles are switches, plain lights, discrete integers, selects, analog inputs, text, scaled values
and actions: the machine of examples/sila, or any machine built from those kinds.

    python3 server.py --port /dev/ttyUSB0       a Nano (or any board running the role::Link sketch) on its USB serial port, 115200
    python3 server.py --sim host/device         a host build of the machine, over a pipe (no board needed)
    python3 server.py ... --trace FILE          write every request and response on the link: op, payload hex, status, payload hex
    python3 server.py ... --list                print the instructions and the tools as JSON and stop (no MCP session)

What it knows about role kinds is one table, KINDS: a kind -> the MCP words for it (the parameter's name; the schema and the descriptions are built from the
kind's type below). What a kind says about a role (the fields it reports and commands, its bounds, values, labels, scale, unit) is read through python/onemachine
(role_facts), the same for every consumer. A role of a kind not in the table is refused at start, naming the role and the kind; so is a tuned role. The tools are
generated from the machine's description, per role:

    <role>_get    every role                 the role's report, presented (labels, units applied)
    <role>_set    every commandable state    one parameter, from the kind's row; returns the report after the device applied it
    <role>_fire   an action                  no parameter; returns the report after the device applied it
    status        the machine                every role's report

    switch   on: boolean                                   light    level: integer 0..`max`            discrete  value: integer, enum the `value` lines
    select   value: string, enum the labels                text     text: string, maxLength N, ASCII    scaled    value: number 0..max*num/den (the `scale`, `unit` lines)
    analog   (no _set) value with its unit                 action   (no parameter)

A value outside the schema is refused here with a tool error that names the parameter and the bound, and nothing is sent. The MCP SDK's low-level Server
calls the handler without checking arguments against inputSchema, so the check is this file's own. A command is forwarded once (m.push()), then the report is
read once (m.poll()): the link is request/response and the device applies a command before it reads the next request, so the report that follows a push is
the device's state after that command. Nothing is held or resent. Needs mcp and pyserial (requirements.txt) and python/onemachine from this checkout
(ONEMACHINE_PY=<dir> to use another)."""
import argparse, json, os, re, sys, threading, time
from fractions import Fraction

HERE = os.path.dirname(os.path.abspath(__file__))
ONEMACHINE_PY = os.environ.get('ONEMACHINE_PY') or os.path.join(HERE, '..', '..', 'python')
if not os.path.isdir(os.path.join(ONEMACHINE_PY, 'onemachine')): sys.exit('server: python/onemachine not found at %s: run from a OneMachine checkout or set ONEMACHINE_PY' % ONEMACHINE_PY)
sys.path.insert(0, ONEMACHINE_PY)
from onemachine import Machine, StreamLink, SchemaError, LinkError, KindError, role_facts

class Refused(ValueError):
    """The machine has something this server has no MCP words for; it does not guess."""

# the whole of what this server knows about role kinds: kind -> the type it makes and the name of its parameter
KINDS = {
    'switch':   dict(type='bool',     param='on'),
    'light':    dict(type='int',      param='level'),
    'discrete': dict(type='discrete', param='value'),
    'select':   dict(type='select',   param='value'),
    'analog':   dict(type='analog',   param=None),
    'text':     dict(type='text',     param='text'),
    'scaled':   dict(type='scaled',   param='value'),
    'action':   dict(type='action',   param=None),
}
TOOL_NAME = re.compile(r'[A-Za-z0-9_-]{1,64}')                  # the tool names clients accept
SUFFIXES = ('get', 'set', 'fire')

def number(x):
    """a Fraction as the plainest JSON number: an int when whole, else its float"""
    return int(x) if x.denominator == 1 else float(x)

class Role:
    """One role of the machine: its tools, its schema, its checks and its presented report. Everything machine-specific is read from the description."""
    def __init__(self, name, kind, row, facts):
        self.name, self.kind, self.row, self.f, self.type = name, kind, row, facts, row['type']

    def settable(self): return self.type != 'analog'

    def top(self):
        return self.f.presented(self.f.high)                                          # scaled: the presented upper bound, max * num / den

    def schema(self):
        t, f = self.type, self.f
        if t == 'bool': return {'type': 'boolean'}
        if t == 'int': return {'type': 'integer', 'minimum': f.low, 'maximum': f.high}
        if t == 'discrete': return {'type': 'integer', 'enum': list(f.values)}
        if t == 'select': return {'type': 'string', 'enum': list(f.labels)}
        if t == 'text': return {'type': 'string', 'maxLength': f.size, 'pattern': '^[ -~]*$'}
        if t == 'scaled': return {'type': 'number', 'minimum': 0, 'maximum': number(self.top())}

    def describe(self, what):
        t, f, n = self.type, self.f, self.name
        if what == 'get':
            words = {'bool': 'switch): whether it is on.', 'int': 'light): its level and whether the device clamped the last command.',
                     'discrete': 'discrete): its value.', 'select': 'select): its value, as a label.', 'text': 'text): its text.',
                     'action': 'action): how many times it has run since boot.',
                     'analog': 'analog, read only): its reading in ' + str(f.unit) + '.', 'scaled': 'scaled): its value in ' + str(f.unit) + '.'}
            return n + ' (' + words[t]
        if what == 'fire': return '%s (action): run it once. Returns the count of runs the device holds.' % n
        held = ' Returns the value the device holds.'
        if t == 'bool': return '%s (switch): set on (true) or off (false).' % n + held
        if t == 'int': return '%s (light): set the level, %d to %d.' % (n, f.low, f.high) + held
        if t == 'discrete': return '%s (discrete): set the value, one of %s.' % (n, ', '.join(str(v) for v in f.values)) + held
        if t == 'select': return '%s (select): set the value, one of %s (the machine\'s order).' % (n, ', '.join(f.labels)) + held
        if t == 'text': return '%s (text): set the text, up to %d printable ASCII characters.' % (n, f.size) + held
        if t == 'scaled':
            return '{} (scaled): set the value in {}, 0 to {}, resolution {}; the device rounds to its resolution.'.format(n, f.unit, number(self.top()), number(Fraction(*f.scale))) + held

    def refuse(self, args):
        """(raw command value, None) for an acceptable argument, or (None, the message): which parameter, which bound. Nothing is sent for a message."""
        p, t, f = self.row['param'], self.type, self.f
        if t == 'action': return (1, None) if not args else (None, 'unknown argument %r: this tool takes none' % sorted(args)[0])
        extra = sorted(set(args) - {p})
        if extra: return None, 'unknown argument %r: the only parameter is %r' % (extra[0], p)
        if p not in args: return None, 'missing parameter %r' % p
        v = args[p]
        if t == 'bool':
            if not isinstance(v, bool): return None, '%s: %r is not a boolean' % (p, v)
            return v, None
        if t == 'text':
            if not isinstance(v, str): return None, '%s: %r is not a string' % (p, v)
            if len(v) > f.size: return None, '%s: %d characters, the maximum is %d' % (p, len(v), f.size)
            try: return f.pack_text(v), None
            except (UnicodeEncodeError, ValueError): return None, '%s: only printable ASCII (0x20 to 0x7e) is accepted' % p
        if t == 'select':
            if not isinstance(v, str): return None, '%s: %r is not a string' % (p, v)
            if v not in f.labels: return None, '%s: %r is not one of %s' % (p, v, ', '.join(f.labels))
            return f.raw_of(v), None
        if isinstance(v, bool) or not isinstance(v, (int, float)) or v != v or v in (float('inf'), float('-inf')): return None, '%s: %r is not a number' % (p, v)
        if t in ('int', 'discrete'):
            if v != int(v): return None, '%s: %r is not an integer' % (p, v)
            v = int(v)
            if t == 'discrete':
                if v not in f.values: return None, '%s: %d is not one of %s' % (p, v, ', '.join(str(x) for x in f.values))
            else:
                if v < f.low: return None, '%s: %d is below the minimum %d' % (p, v, f.low)
                if v > f.high: return None, '%s: %d is above the maximum %d' % (p, v, f.high)
            return v, None
        hi = self.top()                                                               # scaled: the bounds are on the presented value, as in the schema
        x = f.written(v)                                                              # the number as the client wrote it (1.1, not its binary neighbour)
        if x < 0: return None, '%s: %r is below the minimum 0' % (p, v)
        if x > hi: return None, '%s: %r is above the maximum %s' % (p, v, number(hi))
        return f.to_raw(v), None

    def present(self, layer):
        """the report layer as the structured result"""
        t, f = self.type, self.f
        if t == 'bool': out = {'on': bool(layer.on)}
        elif t == 'int': out = {'level': int(layer.level), 'clamped': bool(layer.clamped)}
        elif t == 'discrete': out = {'value': int(layer.value)}
        elif t == 'select': out = {'value': f.label_of(layer.index)}
        elif t == 'analog': out = {'value': float(f.presented(layer.raw)), 'unit': f.unit}
        elif t == 'text': out = {'text': f.unpack_text(layer.text)}
        elif t == 'scaled': out = {'value': number(f.presented(layer.raw)), 'unit': f.unit}
        else: out = {'fired': int(layer.fired)}
        if not layer.live: out['live'] = False                                       # the role's device is not there
        return out

def model(m):
    """[Role] for every role of the machine, or Refused, naming the role."""
    out, tools = [], {}
    for name, info in m.roles.items():
        if info.kind not in KINDS: raise Refused('role %s is of kind %r; this server knows %s' % (name, info.kind, ', '.join(sorted(KINDS))))
        if info.tuned: raise Refused('role %s (%s) is tuned: its limits can change at run time, which a tool schema cannot say' % (name, info.kind))
        try: f = role_facts(m, name)
        except KindError as e: raise Refused(str(e))
        for field in (['clamped'] if info.kind == 'light' else []) + ['live']:
            if field not in f.report_fields: raise Refused('role %s (%s): its report has no field %r' % (name, info.kind, field))
        role = Role(name, info.kind, KINDS[info.kind], f)
        for suffix in SUFFIXES:
            if suffix == 'get' or (suffix == 'set' and role.type not in ('analog', 'action')) or (suffix == 'fire' and role.type == 'action'):
                tool = '%s_%s' % (name, suffix)
                if not TOOL_NAME.fullmatch(tool): raise Refused('role %s: the tool name %r is not %s' % (name, tool, TOOL_NAME.pattern))
                if tool in tools: raise Refused('roles %r and %r are both the tool %s' % (tools[tool], name, tool))
                tools[tool] = name
        out.append(role)
    if not out: raise Refused('the machine has no roles')
    return out

def tools_of(roles):
    """[(tool name, role|None, suffix|'status', description, inputSchema, annotations dict)], all generated"""
    empty = {'type': 'object', 'properties': {}, 'additionalProperties': False}
    out = [('status', None, 'status', 'Every role\'s report in one call.', empty, dict(read_only_hint=True))]
    for r in roles:
        out.append(('%s_get' % r.name, r, 'get', r.describe('get'), empty, dict(read_only_hint=True)))
        if r.type == 'action': out.append(('%s_fire' % r.name, r, 'fire', r.describe('fire'), empty, dict(read_only_hint=False, idempotent_hint=False)))
        elif r.settable():
            p = r.row['param']
            out.append(('%s_set' % r.name, r, 'set', r.describe('set'),
                        {'type': 'object', 'properties': {p: r.schema()}, 'required': [p], 'additionalProperties': False}, dict(read_only_hint=False, idempotent_hint=True)))
    return out

def instructions_of(m, roles):
    return ('OneMachine machine %08x. Roles: %s. Each role has a _get tool; a _set tool takes the new value, a _fire tool runs an action; status reads every role. '
            'A _set or _fire returns what the device holds afterwards, which is the truth. A value outside a tool\'s bounds is refused: tell the user, do not send a different value.' % (m.description.hash, ', '.join('%s (%s)' % (r.name, r.kind) for r in roles)))

class Traced:
    """Wraps a link and writes every request and response: op, payload hex, status, payload hex (test/state/frame_decode.py decodes them), the host time at which
    the request is written, and how long the exchange took."""
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
        time.sleep(2); port.reset_input_buffer()                   # opening the port resets a Nano: wait for its bootloader to pass
        link = StreamLink(port.read, port.write, port.flush)
    return Traced(link, a.trace) if a.trace else link

def serve(m, roles, name='onemachine'):
    import anyio, anyio.to_thread
    import mcp_types as types
    from mcp.server import Server
    from mcp.server.stdio import stdio_server
    from mcp.shared.exceptions import MCPError
    lock = threading.Lock()                                          # one frame at a time: a frame carries every role's command
    table = {t[0]: t for t in tools_of(roles)}

    def reply(structured, error=False):
        return types.CallToolResult(content=[types.TextContent(text=json.dumps(structured, separators=(',', ':')))], structured_content=structured, is_error=error)
    def fail(tool, message): return types.CallToolResult(content=[types.TextContent(text='%s: %s' % (tool, message))], is_error=True)

    def run(tool, role, kind, args):
        """the whole of a call, under the lock: refuse or push once, poll once, present"""
        if kind in ('get', 'status'):
            if args: return fail(tool, 'unknown argument %r: this tool takes none' % sorted(args)[0])
            with lock: r = m.poll()
            return reply({x.name: x.present(getattr(r, x.name)) for x in roles} if kind == 'status' else role.present(getattr(r, role.name)))
        raw, error = role.refuse(args)
        if error: return fail(tool, '%s. Nothing was sent to the device.' % error)
        layer_name, field = role.name, role.f.command
        with lock:
            layer = getattr(m.cmd, layer_name)
            before = getattr(layer, field)
            setattr(layer, field, raw)
            try:
                try: m.push()                                         # once
                finally:
                    if kind == 'fire': setattr(layer, field, 0)       # the flag is 1 only during one push, and 0 in every other frame
                r = m.poll()                                          # the report after the device applied the command
            except Exception:
                if kind != 'fire': setattr(layer, field, before)      # a command that did not go is not kept
                raise
        return reply(role.present(getattr(r, role.name)))

    async def on_list_tools(ctx, params):
        return types.ListToolsResult(tools=[types.Tool(name=n, description=d, input_schema=s, annotations=types.ToolAnnotations(**a)) for n, _, _, d, s, a in table.values()])

    async def on_call_tool(ctx, params):
        entry = table.get(params.name)
        if entry is None: raise MCPError(code=types.INVALID_PARAMS, message='unknown tool %r' % params.name)
        tool, role, kind = entry[0], entry[1], entry[2]
        try: return await anyio.to_thread.run_sync(run, tool, role, kind, dict(params.arguments or {}))
        except (SchemaError, LinkError, OSError, ValueError) as e: return fail(tool, 'the machine did not take it: %s' % e)

    server = Server(name, version='0.1', instructions=instructions_of(m, roles), on_list_tools=on_list_tools, on_call_tool=on_call_tool)
    async def main():
        async with stdio_server() as (read, write):
            await server.run(read, write, server.create_initialization_options())
    anyio.run(main)

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--port', metavar='SERIAL'); src.add_argument('--sim', metavar='HOST_BINARY')
    ap.add_argument('--trace', metavar='FILE')
    ap.add_argument('--list', action='store_true', help='print the instructions and tools/list as JSON on stdout and stop (no MCP session)')
    a = ap.parse_args()
    try: m = Machine(open_link(a))
    except SchemaError as e: sys.exit('server: the binding refuses this machine: %s' % e)
    except (LinkError, OSError) as e: sys.exit('server: cannot reach the machine: %s' % e)
    try: roles = model(m)
    except Refused as e: sys.exit('server: refused: %s' % e)
    if a.list:
        print(json.dumps({'instructions': instructions_of(m, roles), 'tools': [dict(name=n, description=d, inputSchema=s, annotations=an) for n, _, _, d, s, an in tools_of(roles)]}, indent=1))
        return
    print('server: %d roles, %d tools, machine hash %08x' % (len(roles), len(tools_of(roles)), m.description.hash), file=sys.stderr, flush=True)
    serve(m, roles)

if __name__ == '__main__':
    main()
