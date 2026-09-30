#!/usr/bin/env python3
"""The role machine end to end: python/onemachine drives test/role/sim_device (machine.h) over role/link.h, as a consumer that knows
nothing but role names. Usage: check.py DIR   (DIR holds sim, sim_wiring_b, sim_v2, sim_v3). Exit 1 on any failure."""
import os, struct, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'python'))
from onemachine import Machine, StreamLink, RoleChanged, Description

D = sys.argv[1]
fails = 0
def check(what, cond, got=None):
    global fails
    if cond: print('  ok    ' + what)
    else: fails += 1; print('  FAIL  %s%s' % (what, '' if got is None else ': got %r' % (got,)))

def dev(name): return StreamLink.popen([os.path.join(D, name)])
def run(m, ms): st, _ = m.link.call('t', struct.pack('<I', ms)); assert st == 0
def writes(m, i): return m.link.call('w', bytes([i]))[0]

print('== the description: roles, kinds, parameters, references; where is for people only')
m = Machine(dev('sim'))
check('roles and kinds', {r: i.kind for r, i in m.roles.items()} == {'x': 'axis', 'white': 'light', 'blue': 'light', 'uv': 'light', 'pump': 'switch'},
      {r: i.kind for r, i in m.roles.items()})
check('axis parameters', m.roles['x'].params == {'steps_mm': 80, 'min_um': 0, 'max_um': 300000}, m.roles['x'].params)
check('a light parameter tells two lights apart', m.roles['white'].params['max'] == 4000 and m.roles['blue'].params['safe'] == 100)
check('the fixed reference', m.refs == ['https://github.com/InternetOfPins/OneMachine/blob/main/docs/role.md'], m.refs)
check('where: bound at discovery, behind the mux', m.where['white'] == 'pca9685 0x40 behind 0x70/2 #0', m.where.get('white'))
check('where: pinned by path', m.where['uv'].startswith('pca9685 path ') and m.where['uv'].endswith('#15'), m.where.get('uv'))
hashes0 = (m.description.hash, m.command_schema.hash, m.report_schema.hash)

print('== commands by role, applied at the cycle boundary')
r = m.poll()
check('every role live at start', all(getattr(r, n).live for n in m.roles), r)
m.cmd.x.target_um = 150000; m.cmd.white.level = 5000; m.cmd.blue.level = 300; m.cmd.uv.level = 4095; m.cmd.pump.on = True
m.push()
r = m.poll(); check('not applied before the cycle boundary', r.white.level == 0 and not r.pump.on, r)
run(m, 10); r = m.poll()
check('white clamped to its max', (r.white.level, r.white.clamped) == (4000, True), (r.white.level, r.white.clamped))
check('blue, uv, pump applied', (r.blue.level, r.blue.clamped, r.uv.level, r.pump.on) == (300, False, 4095, True))
check('the axis moves toward its target', 0 < r.x.pos_um < 150000, r.x.pos_um)
run(m, 400); r = m.poll(); check('the axis reaches its target', r.x.pos_um == 150000, r.x.pos_um)
m.cmd.x.target_um = 400000; m.push(); run(m, 400); r = m.poll()      # inside the 500 ms quiet time
check('an axis target past its range is clamped', (r.x.pos_um, r.x.clamped) == (300000, True), (r.x.pos_um, r.x.clamped))
check('the decoy PCA9685 at 0x40 behind channel 3 never written', writes(m, 3) == 0, writes(m, 3))
check('ours at 0x40 behind channel 2 written', writes(m, 0) > 0)
check('no write reached no device', m.link.call('e')[0] == 0)
try: m.cmd.white.level = 70000; check('a level outside u16 refused in Python', False)
except ValueError: check('a level outside u16 refused in Python', True)

print('== the link refuses what it must')
check('a short command frame: BadLength', m.link.call('s', m.command_schema.encode(m.cmd)[:-1])[0] == 2)
bad = bytearray(m.command_schema.encode(m.cmd)); bad[0] ^= 1
check('another schema: BadHash', m.link.call('s', bytes(bad))[0] == 1)
check('an unknown op', m.link.call('Z')[0] == 0x80)
check('a request longer than the buffer', m.link.call('s', bytes(100))[0] == 0x81)
check('the link still answers after all that', m.poll().white.level == 4000)

print('== the supervisor goes quiet: every role to its safe command')
run(m, 600); r = m.poll()
check('lights to their safe levels, pump off', (r.white.level, r.blue.level, r.uv.level, r.pump.on) == (0, 100, 0, False), (r.white.level, r.blue.level, r.uv.level, r.pump.on))
check('the axis holds where it is', r.x.pos_um == 300000, r.x.pos_um)

print('== a device leaves: its roles are not live, nothing is retargeted')
m.cmd.white.level = 1234; m.push(); run(m, 10)
before = writes(m, 3)
check('unplug ours', m.link.call('p', bytes([2, 0x40, 0]))[0] == 0)
r = m.poll(); check('white and blue not live, the others are', (r.white.live, r.blue.live, r.uv.live, r.x.live) == (False, False, True, True))
m.cmd.white.level = 2000; m.push(); run(m, 10)
check('the decoy at the same address is still untouched', writes(m, 3) == before == 0, writes(m, 3))
check('replug ours', m.link.call('p', bytes([2, 0x40, 1]))[0] == 0)
r = m.poll(); check('live again, with the current command', (r.white.live, r.white.level) == (True, 2000), (r.white.live, r.white.level))

print('== a reference given at run time')
m.link.call('u', b'file:///srv/machines/pilot.md'); m.refresh()
check('both references listed', m.refs == ['https://github.com/InternetOfPins/OneMachine/blob/main/docs/role.md', 'file:///srv/machines/pilot.md'], m.refs)
check('the machine hash covers it, the frames do not', m.description.hash != hashes0[0] and (m.command_schema.hash, m.report_schema.hash) == hashes0[1:])
m.link.close()

print('== rewired firmware: same roles, other devices; the consumer does nothing')
m.relink(dev('sim_wiring_b'))
m.cmd.white.level = 3000; m.push(); run(m, 10); r = m.poll()
check('the same command frame accepted as is', (r.white.live, r.white.level) == (True, 3000), (r.white.live, r.white.level))
check('now written to the PCA9685 at 0x41', writes(m, 1) > 0 and writes(m, 0) == 0, (writes(m, 1), writes(m, 0)))
check('no hash changed', (Description(m._text('m')).hash, m.command_schema.hash, m.report_schema.hash) == hashes0)
check('only where changed', Description(m._text('m')).where['white'] == 'pca9685 0x41 #7')
m.link.close()

print('== firmware with a role added: re-read on BadHash, values kept by role name')
m.cmd.x.target_um = 42000; m.relink(dev('sim_v2')); m.push(); run(m, 400); r = m.poll()
check('the new role is known', m.roles['fan'].kind == 'switch')
check('the command survived the re-read', (r.x.pos_um, r.white.level) == (42000, 3000), (r.x.pos_um, r.white.level))
m.link.close()

print('== firmware without a role this consumer drives: told, not retargeted')
m.relink(dev('sim_v3'))
try: m.push(); check('RoleChanged raised', False)
except RoleChanged as e: check('RoleChanged raised: %s' % e, 'blue gone' in str(e))
m.link.close()
n = Machine(dev('sim_v3')); n.cmd.white.level = 10; n.push(); run(n, 10)
check('a consumer that never drove blue is fine', n.poll().white.level == 10); n.link.close()

print('%d failed' % fails)
sys.exit(1 if fails else 0)
