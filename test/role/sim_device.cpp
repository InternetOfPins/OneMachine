// The test machine (machine.h) as a process: role::Link on stdin/stdout, simulated time, simulated bus. check.py drives it.
// Test-only ops (App::op, not part of the link): 't' u32 ms  advance simulated time in 10 ms cycles
//                                                'u' text     set the run-time reference (RefFrom)
//                                                'p' mux addr present   plug/unplug a sim device, then rediscover (pins again)
//                                                'e'          status = misrouted writes so far (must stay 0)
//                                                'w' i        status = writes to sim device i so far (capped at 127)
#include "machine.h"
#include <oneMachine/role/link.h>
#include <stdio.h>
#include <string.h>

struct Out { static void put(uint8_t b) { putchar(b); } };
static M::Command cmd{};
static M::Report rep{};
static uint32_t now = 0;

static void cycle() {
  XAt::tick();
  now += 10;
}
struct App;
using L = role::Link<M, Out, App, 64>;
static L link(rep, 500);

struct App {
  static int op(uint8_t op, const uint8_t* p, uint16_t n) {
    switch (op) {
      case 't': {
        if (n != 4) return 2;
        uint32_t ms = uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
        for (uint32_t t = 0; t < ms; t += 10) {            // the cycle boundary: new command, quiet supervisor, then the report
          if (link.take(cmd)) M::apply(cmd, rep);
          if (link.quiet(now)) { M::safe(cmd, rep); M::apply(cmd, rep); }
          cycle(); M::sense(rep);
        }
        return 0;
      }
      case 'u': { if (n >= sizeof UserRef::text) return 2; memcpy(UserRef::text, p, n); UserRef::text[n] = 0; return 0; }
      case 'p': {
        if (n != 3) return 2;
        for (auto& d : Sim::dev) if (d.addr == p[1] && d.mux == p[0]) d.present = p[2] != 0;
        simDiscover(); M::apply(cmd, rep); return 0;      // a device that comes back gets the current command
      }
      case 'e': return int(Sim::misrouted > 0x7F ? 0x7F : Sim::misrouted);
      case 'w': return n == 1 && p[0] < 4 ? int(Sim::dev[p[0]].writes > 0x7F ? 0x7F : Sim::dev[p[0]].writes) : 2;
      default: return -1;
    }
  }
};

int main() {
  simWire(); simDiscover();
  M::apply(cmd, rep);   // the power-on command (all zero), so the report starts true
  int c;
  while ((c = getchar()) != EOF) { link.feed(uint8_t(c), now); fflush(stdout); }
  return 0;
}
