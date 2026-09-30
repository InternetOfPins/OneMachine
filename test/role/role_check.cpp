// The role module in one process (native.sh, every compiler, sanitizers): the machine of machine.h driven through role::Link from a byte
// buffer, without Python. check.py is the same machine from the consumer's side. Prints "N failed", exits non-zero on a failure.
#include "machine.h"
#include <oneMachine/role/link.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

static std::vector<uint8_t> out;
struct Out { static void put(uint8_t b) { out.push_back(b); } };
static M::Command cmd{};
static M::Report rep{};
using L = role::Link<M, Out>;
static L link(rep, 500);
static uint32_t now = 0;
static int fails = 0;
static void check(const char* what, bool ok) { printf("  %s  %s\n", ok ? "ok  " : "FAIL", what); if (!ok) ++fails; }

struct Reply { uint8_t st; std::vector<uint8_t> data; };
static Reply call(char op, const std::vector<uint8_t>& p = {}) {
  out.clear();
  link.feed(uint8_t(op), now); link.feed(uint8_t(p.size()), now); link.feed(uint8_t(p.size() >> 8), now);
  for (uint8_t b : p) link.feed(b, now);
  Reply r{}; if (out.size() < 3) return {0xFF, {}};
  r.st = out[0]; unsigned n = out[1] | unsigned(out[2]) << 8; r.data.assign(out.begin() + 3, out.end());
  if (r.data.size() != n) r.st = 0xFE;
  return r;
}
static void cycles(unsigned n) {
  for (unsigned i = 0; i < n; i++) {
    if (link.take(cmd)) M::apply(cmd, rep);
    if (link.quiet(now)) { M::safe(cmd, rep); M::apply(cmd, rep); }
    XAt::tick(); now += 10; M::sense(rep);
  }
}
static std::vector<uint8_t> frame(const M::Command& c) { std::vector<uint8_t> f(state::wire_size<M::Command>()); state::write(c, f.data()); return f; }
static std::string text(const Reply& r) { return std::string(r.data.begin(), r.data.end()); }

int main() {
  simWire(); simDiscover(); M::apply(cmd, rep);

  printf("== binding at discovery: the Where decides, not the address alone\n");
  check("white bound to ours (behind 0x70/2)", WhiteAt::Dev::row != discover::noRow && SimWorld::reg.rows[SimWorld::reg.rows[WhiteAt::Dev::row].parent].busId == 2);
  check("uv pinned by path to 0x41", UvAt::row != discover::noRow && UvAt::addr == 0x41);
  check("M wants the PCA9685 driver, not the RTC or the mux", M::wants<Sim::Pca> && !M::wants<SimRtc> && !M::wants<SimMux>);

  printf("== descriptions\n");
  Reply m = call('m'), c = call('c'), r = call('r');
  check("machine description", m.st == 0 && text(m).find("role white light\nparam white max 4000\n") != std::string::npos);
  check("where lines", text(m).find("at white pca9685 0x40 behind 0x70/2 #0\n") != std::string::npos);
  check("command description is state::describe", c.st == 0 && text(c).rfind("state 1\n", 0) == 0 && text(c).find("white/level u16") != std::string::npos);
  check("report has live per role", r.st == 0 && text(r).find("white/live bool") != std::string::npos);
  check("describe's hash is machine_hash", text(m).find("hash ") == 10);

  printf("== commands\n");
  M::Command want{}; state::get<White>(want).level = 5000; state::get<Pump>(want).on = true; state::get<X>(want).target_um = 1000;
  check("set: Ok", call('s', frame(want)).st == 0);
  check("not applied before the boundary", state::get<White>(rep).level == 0);
  cycles(1);
  check("white clamped to 4000", state::get<White>(rep).level == 4000 && state::get<White>(rep).clamped);
  check("the PCA9685 channel holds it", Sim::dev[0].ch[0] == 4000 && Sim::dev[3].writes == 0);
  Reply g = call('g');
  M::Report back{}; check("get: a frame state::read accepts", g.st == 0 && state::read(back, g.data.data(), unsigned(g.data.size())) == state::Status::Ok);
  check("  and it is the report", state::get<White>(back).level == 4000 && state::get<Pump>(back).on);
  auto f = frame(want); f.pop_back();
  check("short frame: BadLength", call('s', f).st == 2);
  f = frame(want); f[1] ^= 1;
  check("other hash: BadHash", call('s', f).st == 1);
  f = frame(want); f[4 + 0] = 2;                          // pump/on is first (the last listed layer runs first): a bool byte of 2
  check("bad bool: BadValue", call('s', f).st == 3);
  check("too long", call('s', std::vector<uint8_t>(200)).st == role::LinkTooLong);
  check("unknown op", call('?').st == role::LinkUnknown);

  printf("== quiet supervisor\n");
  cycles(60);
  check("safe after the quiet time", state::get<White>(rep).level == 0 && state::get<Blue>(rep).level == 100 && !state::get<Pump>(rep).on);
  check("axis holds", state::get<X>(rep).pos_um == 1000);

  printf("== a device leaves\n");
  Sim::dev[0].present = false; simDiscover(); M::apply(cmd, rep);
  check("white not live, not retargeted", !state::get<White>(rep).live && Sim::dev[3].writes == 0 && WhiteAt::Dev::row == discover::noRow);
  check("uv still live", state::get<Uv>(rep).live);

  printf("%d failed\n", fails);
  return fails ? 1 : 0;
}
