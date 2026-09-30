// The example's machine on a host, pins simulated: role::Link on stdin/stdout, real time. drive.py --sim runs it.
#include "../src/machine.h"
#include <oneMachine/role/sim.h>
#include <oneMachine/role/link.h>
#include <stdio.h>
#include <chrono>

using M = MachineOf<role::SimPin<13>, role::SimPwm<9, 255>>;
struct Out { static void put(uint8_t b) { putchar(b); } };
static M::Command cmd{};
static M::Report rep{};
static role::Link<M, Out> link(rep, 2000);
static uint32_t millis() {
  using namespace std::chrono;
  static const auto t0 = steady_clock::now();
  return uint32_t(duration_cast<milliseconds>(steady_clock::now() - t0).count());
}

int main() {
  M::apply(cmd, rep);
  int c;
  while ((c = getchar()) != EOF) {                      // a cycle after every byte (a request's first byte runs one before it is answered)
    link.feed(uint8_t(c), millis()); fflush(stdout);
    if (link.take(cmd)) M::apply(cmd, rep);
    if (link.quiet(millis())) { M::safe(cmd, rep); M::apply(cmd, rep); }
    M::sense(rep);
  }
  return 0;
}
