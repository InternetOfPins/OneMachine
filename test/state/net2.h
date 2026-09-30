// The wire net: the fib net (net.h) plus a layer with every wire type (bool, i8, u32, i32, u64, i64), stepped by modular arithmetic.
#pragma once
#include "net.h"

struct Misc { ONEMACHINE_STATE_NAME(name, "misc"); };
#ifdef SLOPPY
  typedef int Sloppy32;                     // not fixed width: 16 bits on AVR, 32 on the host
#else
  typedef int32_t Sloppy32;
#endif
struct SlotMisc { bool flag; int8_t s8; uint32_t u32; Sloppy32 i32; uint64_t u64; int64_t i64;
  ONEMACHINE_STATE_NAME(n_flag, "flag"); ONEMACHINE_STATE_NAME(n_s8, "s8"); ONEMACHINE_STATE_NAME(n_u32, "u32"); ONEMACHINE_STATE_NAME(n_i32, "i32"); ONEMACHINE_STATE_NAME(n_u64, "u64"); ONEMACHINE_STATE_NAME(n_i64, "i64");
  template<class Self, class F> static constexpr void each(Self& s, F& f)
    { f(n_flag(), s.flag); f(n_s8(), s.s8); f(n_u32(), s.u32); f(n_i32(), s.i32); f(n_u64(), s.u64); f(n_i64(), s.i64); } };
struct StepMisc { template<class B, class P> static SlotMisc run(const B&, const P& p) {
  const SlotMisc& m = state::get<Misc>(p);
  SlotMisc n{};
  n.u32 = m.u32 * 1664525u + 1013904223u;
  n.i32 = Sloppy32(uint32_t(m.i32) * 3u + 7u);
  n.s8  = int8_t(uint8_t(uint8_t(m.s8) * 3u + 1u));
  n.u64 = m.u64 * 6364136223846793005ull + 1442695040888963407ull;
  n.i64 = int64_t(uint64_t(m.i64) ^ (n.u64 >> 1));
  n.flag = m.flag != bool(n.u32 & 1);
  return n; } };

using Net2 = hapi::APIOf<state::API, state::Layer<Misc,SlotMisc,StepMisc>, state::Layer<Watch,SlotWatch,StepWatch>, state::Layer<FibA,SlotFibA,StepA>, state::Layer<FibB,SlotFibB,StepB>>::Res;

// the state the peers start from: every field non-trivial, signs and high bits set
inline Net2 initial() {
  Net2 n{};
  state::get<FibA>(n).a = 7; state::get<FibB>(n).b = -3; state::get<Watch>(n).steps = 40000; state::get<Watch>(n).odd = 1;
  SlotMisc& m = state::get<Misc>(n); m.flag = true; m.s8 = -100; m.u32 = 0xDEADBEEFu; m.i32 = Sloppy32(-123456789); m.u64 = 0x0123456789ABCDEFull; m.i64 = -0x0102030405060708ll;
  return n;
}
constexpr int STEPS = 5;
