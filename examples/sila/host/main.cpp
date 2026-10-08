// The machine of machine.h on a host, with simulated endpoints: role::Link on stdin/stdout. A stand-in for the Nano when none is attached.
#include "../src/machine.h"
#include <oneMachine/role/sim.h>
#include <oneMachine/role/link.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

struct SimAdc {                                // the reading is the environment variable SIM_ADC (0..1023, default 0), read once
  static bool live() { return true; }
  static uint16_t get() { static const uint16_t v = getenv("SIM_ADC") ? uint16_t(atoi(getenv("SIM_ADC"))) : 0; return v; }
  template<class Put> static void where(Put& put) { put('A'); put('0'); }
};

using M = MachineOf<role::SimPin<13>, role::SimPwm<9, 255>, SimAdc>;
struct Out { static void put(uint8_t b) { putchar(b); } };
static M::Command cmd{};
static M::Report rep{};
static role::Link<M, Out> chan(rep, 0);
static uint32_t millis() {
  using namespace std::chrono;
  static const auto t0 = steady_clock::now();
  return uint32_t(duration_cast<milliseconds>(steady_clock::now() - t0).count());
}

int main() {
  M::apply(cmd, rep);
  int c;
  while ((c = getchar()) != EOF) {
    chan.feed(uint8_t(c), millis()); fflush(stdout);
    if (chan.take(cmd)) { M::apply(cmd, rep); state::get<Ping>(cmd).fire = 0; }
    M::sense(rep);
  }
  return 0;
}
