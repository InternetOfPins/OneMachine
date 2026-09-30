// role/call.h's C ABI, called as a consumer in the same image would (Rust over FFI, Python through ctypes): the example machine of
// examples/python with simulated pins, exported by ONEMACHINE_CALL_EXPORT. Prints "N failed", exits non-zero on a failure.
#include "../../examples/python/src/machine.h"
#include <oneMachine/role/sim.h>
#include <oneMachine/role/call.h>
#include <stdio.h>
#include <string.h>

using M = MachineOf<role::SimPin<13>, role::SimPwm<9, 255>>;
static role::Call<M> dev(100);
ONEMACHINE_CALL_EXPORT(onemachine, dev)

static int fails = 0;
static void check(const char* what, bool ok) { printf("  %s  %s\n", ok ? "ok  " : "FAIL", what); if (!ok) ++fails; }

int main() {
  uint8_t out[512], small[8];
  int32_t n = onemachine_call('m', nullptr, 0, out, sizeof out, 0);
  check("m: the machine description", n > 3 && out[0] == 0 && memcmp(out + 3, "machine 1\n", 10) == 0 && unsigned(out[1] | out[2] << 8) == unsigned(n - 3));
  int32_t need = onemachine_call('m', nullptr, 0, small, sizeof small, 0);
  check("a reply that does not fit: minus the length it needs", need == -n);
  n = onemachine_call('c', nullptr, 0, out, sizeof out, 0);
  check("c: the command description", n > 3 && strstr(reinterpret_cast<char*>(out + 3), "lamp/level u16") != nullptr);

  M::Command c{}; state::get<Lamp>(c).level = 250; state::get<Led>(c).on = true;
  uint8_t f[state::wire_size<M::Command>()]; state::write(c, f);
  n = onemachine_call('s', f, sizeof f, small, 3, 10);
  check("s: accepted, a 3-byte reply fits in 3 bytes", n == 3 && small[0] == 0);
  n = onemachine_call('g', nullptr, 0, out, sizeof out, 10);
  M::Report r{}; state::read(r, out + 3, unsigned(n - 3));
  check("not applied before the cycle", state::get<Lamp>(r).level == 0);
  onemachine_cycle(20);
  n = onemachine_call('g', nullptr, 0, out, sizeof out, 20);
  check("g: a report frame", n > 3 && out[0] == 0 && state::read(r, out + 3, unsigned(n - 3)) == state::Status::Ok);
  check("applied at the cycle: lamp clamped to 200, led on", state::get<Lamp>(r).level == 200 && state::get<Lamp>(r).clamped && state::get<Led>(r).on);
  check("  and the pins hold it", role::SimPwm<9, 255>::v == 200 && role::SimPin<13>::v);
  onemachine_cycle(200);
  check("quiet for 100 ms: safe command at the next cycle", role::SimPwm<9, 255>::v == 0 && !role::SimPin<13>::v);
  check("unknown op", onemachine_call('?', nullptr, 0, out, sizeof out, 300) == 3 && out[0] == role::LinkUnknown);
  printf("%d failed\n", fails);
  return fails ? 1 : 0;
}
