# mcp

The machine of [`examples/sila`](../sila) served to AI assistants over MCP. Claude Code, or any other MCP client, can read and
command every role of the Nano by name, without knowing OneMachine, the pins or the board. Nothing on the board changes: this is
the same firmware as in `examples/sila` (wiring and flashing are described there), and the same machine description feeds both
examples. One machine, two consumers.

[MCP](https://modelcontextprotocol.io) (the Model Context Protocol) is the way an AI assistant is given tools. A server lists its
tools, each with a name, a description and a JSON Schema for its arguments; the assistant reads them, decides which to call, and the
server returns the result. `server.py` reads the machine's description over the serial link, generates the tools from it and serves
them on standard input and output, so the client starts it as a program. It knows only the kinds in its table: a role of any other
kind is refused at start, naming the role.

## Run

```
pip install -r requirements.txt           # mcp and pyserial; Python 3.10 or newer
python3 server.py --port /dev/ttyUSB0     # the Nano's port
```

Opening the port resets a Nano: all outputs start off at each start.

Without the board, the same machine on this computer (from this directory):

```
g++ -std=c++17 -I../../../HAPI/include -I../../include ../sila/host/main.cpp -o ../sila/host/machine
python3 server.py --sim ../sila/host/machine
```

`python3 server.py --sim ../sila/host/machine --list` prints what a client is told (the instructions and every tool with its schema)
and stops. `--trace FILE` writes every request and response on the link.

## Add it to Claude Code

```
claude mcp add machine -- python3 /path/to/examples/mcp/server.py --port /dev/ttyUSB0
```

Then ask in plain words: "What can this machine do?", "Turn on the LED.", "Set the lamp to half its maximum."

## What the tools are

For each role, `<role>_get` reads it. A commandable role also has `<role>_set` with one argument, or `<role>_fire` for an action.
`status` reads every role in one call. The schema is built from the role's kind and its description:

| role (kind) | tools | argument and its schema | what `_get` returns |
|---|---|---|---|
| `led` (`switch`) | `led_get`, `led_set` | `on`: boolean | `{"on": true}` |
| `lamp` (`light`) | `lamp_get`, `lamp_set` | `level`: integer, 0 to `max` (200) | `{"level": 100, "clamped": false}` |
| `step` (`discrete`) | `step_get`, `step_set` | `value`: integer, one of the `value` lines (0, 10, 50, 100) | `{"value": 50}` |
| `mode` (`select`) | `mode_get`, `mode_set` | `value`: string, one of the labels (off, low, high) | `{"value": "high"}` |
| `vin` (`analog`) | `vin_get` | none: read only | `{"value": 2.49, "unit": "V"}` |
| `note` (`text`) | `note_get`, `note_set` | `text`: string, at most 16 printable ASCII characters | `{"text": "hello"}` |
| `duty` (`scaled`) | `duty_get`, `duty_set` | `value`: number, 0 to 100 (`max` times the `scale` line) | `{"value": 12.4, "unit": "%"}` |
| `ping` (`action`) | `ping_get`, `ping_fire` | none | `{"fired": 3}` |

How they behave:

- **A set returns the value the device holds.** The command is sent once, then the report is read after the device has applied it:
  `duty_set 12.36` returns 12.4, the device's own resolution. The report is the truth.
- **A value outside the schema is refused, and nothing is sent.** The result is a tool error that names the parameter and the bound
  (`level: 250 is above the maximum 200`). The server checks every argument itself, whether or not the client checks the schema.
- **The assistant is told to ask, not to substitute.** The server's instructions say that a value outside a tool's bounds is
  refused, and that the assistant should tell the user instead of sending a different value.
- **Nothing is held or resent.** A `_fire` sends its flag in exactly one frame. Whatever the device does when commands stop is the
  device's own behaviour. One lock on the link serialises the calls.
- **Every argument is checked, even the ones a schema cannot express:** `text` must be printable ASCII, a `light` must be a whole
  number, a boolean is not a number.

The rules any such consumer follows are in [`docs/consumers.md`](../../docs/consumers.md). What a kind says about a role (the
fields it reports and commands, the bound, the values, the scale) is read through `python/onemachine`; this file holds the MCP words.

## An expected session

Claude Code on a Nano, a fresh session with only this server's tools (the answers are shortened; each tool call and its result are the server's own):

```
> What can this machine do?
  This machine (OneMachine `ba36fc2d`) exposes several controllable roles:
  led (switch) – on/off · lamp (light) – level 0–200 · step (discrete) – one of 0, 10, 50, 100 · mode (select) – off, low, high ·
  vin (analog, read-only) – voltage reading in V · note (text) – up to 16 ASCII characters · duty (scaled) – 0–100%, 0.1 resolution ·
  ping (action) – fires once, counts how many times it's run since boot
> Turn on the LED.
  led_set {"on": true}                 -> {"on":true}
> Set the lamp to half its maximum.
  lamp_set {"level": 100}              -> {"level":100,"clamped":false}
> Set the mode to the highest setting.
  mode_set {"value": "high"}           -> {"value":"high"}
> Write 'hi Rui' on the note.
  note_set {"text": "hi Rui"}          -> {"text":"hi Rui"}
> What voltage is on vin?
  vin_get {}                           -> {"value":0.21994134897360704,"unit":"V"}      (A0 is floating: noise between 0 and 5 V)
> Set the lamp to 300.
  That's out of bounds — lamp only accepts 0 to 200. I won't send a different value; let me know what level within that range you'd like.
```

Four commands reached the board, one frame each; the last request sent nothing. A client that sends the value anyway gets the tool
error, and the board never sees it; a client that bypasses the server and sends it to the board gets the kind's own behaviour (a
light is clamped to its `max`).

## What it does not cover

No network transport and no authentication (the client starts the server and talks to it on standard input and output), no tuned
roles (their limits can change at run time, which a tool schema cannot say), no timeout or safe fall (a value that was set stays
set), no MCP resources or prompts beyond the server's instructions, and no reconnect: if the board is unplugged, the tools return
errors until the server is restarted.
