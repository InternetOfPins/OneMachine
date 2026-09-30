// The test machine (machine.h) as a device a consumer reaches through role::Call, built two ways:
//   a process   role::Link's bytes on stdin/stdout (check.py over a pipe, as over a serial port)
//   -DSIM_LIB   a shared library exporting onemachine_call / onemachine_cycle (check.py in-process through ctypes)
// Simulated time and bus. Test-only ops (App::op, not part of the link):
//   't' u32 ms            advance simulated time in 10 ms cycles (role::Call::cycle, and the stepper's tick)
//   'u' text              set the run-time reference (RefFrom)
//   'p' mux addr present  plug/unplug a sim device, then rediscover (binds again) and apply the current command
//   'e'                   status = misrouted writes so far (must stay 0)
//   'w' i                 status = writes to sim device i so far (capped at 127)
#include "machine.h"
#include <oneMachine/role/call.h>
#include <stdio.h>
#include <string.h>

struct App;
static uint32_t now = 0;
static bool wired = (simWire(), simDiscover(), true);          // the bus before the device object applies its power-on command
static role::Call<M, App, 64> dev(500);

struct App {
  static int op(uint8_t op, const uint8_t* p, uint16_t n) {
    switch (op) {
      case 't': {
        if (n != 4) return 2;
        uint32_t ms = uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
        for (uint32_t t = 0; t < ms; t += 10) { dev.cycle(now); XAt::tick(); now += 10; M::sense(dev.rep); }
        return 0;
      }
      case 'u': { if (n >= sizeof UserRef::text) return 2; memcpy(UserRef::text, p, n); UserRef::text[n] = 0; return 0; }
      case 'p': {
        if (n != 3) return 2;
        for (auto& d : Sim::dev) if (d.addr == p[1] && d.mux == p[0]) d.present = p[2] != 0;
        simDiscover(); M::apply(dev.cmd, dev.rep); return 0;       // a device that comes back gets the current command
      }
      case 'e': return int(Sim::misrouted > 0x7F ? 0x7F : Sim::misrouted);
      case 'w': return n == 1 && p[0] < 4 ? int(Sim::dev[p[0]].writes > 0x7F ? 0x7F : Sim::dev[p[0]].writes) : 2;
      default: return -1;
    }
  }
};

#ifdef SIM_LIB
// ONEMACHINE_CALL_EXPORT's two functions, on the simulated clock instead of the caller's (the 't' op is this device's time)
extern "C" ONEMACHINE_CALL_API int32_t onemachine_call(uint8_t op, const uint8_t* in, uint16_t n, uint8_t* out, uint16_t cap, uint32_t) {
  (void)wired; return dev.call(op, in, n, out, cap, now); }
extern "C" ONEMACHINE_CALL_API void onemachine_cycle(uint32_t) { dev.cycle(now); }
#else
int main() {
  (void)wired;
  static uint8_t in[256], out[4096];
  for (;;) {
    int op = getchar(), lo = getchar(), hi = getchar();
    if (op == EOF || lo == EOF || hi == EOF) return 0;
    uint16_t n = uint16_t(lo | hi << 8), k = 0;
    for (; k < n; k++) { int c = getchar(); if (c == EOF) return 0; if (k < sizeof in) in[k] = uint8_t(c); }
    // a request longer than `in` is still answered by the link (TooLong); its payload is not needed then
    int32_t r = dev.call(uint8_t(op), in, n < sizeof in ? n : uint16_t(sizeof in), out, sizeof out, now);
    if (n > sizeof in) { out[0] = role::LinkTooLong; out[1] = out[2] = 0; r = 3; }
    fwrite(out, 1, size_t(r < 0 ? 0 : r), stdout); fflush(stdout);
  }
}
#endif
