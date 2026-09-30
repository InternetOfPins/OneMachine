// A small composition with array fields of every element kind, for the array support of state.h.
#pragma once
#include <oneMachine/state/state.h>
struct Arr { ONEMACHINE_STATE_NAME(name, "arr"); };
struct Tail { ONEMACHINE_STATE_NAME(name, "tail"); };
struct SlotArr { uint8_t band[4]; int16_t w[3]; bool f[2]; uint64_t q[2]; uint16_t n;
  ONEMACHINE_STATE_NAME(n_band, "band"); ONEMACHINE_STATE_NAME(n_w, "w"); ONEMACHINE_STATE_NAME(n_f, "f"); ONEMACHINE_STATE_NAME(n_q, "q"); ONEMACHINE_STATE_NAME(n_n, "n");
  template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_band(), s.band); f(n_w(), s.w); f(n_f(), s.f); f(n_q(), s.q); f(n_n(), s.n); } };
struct SlotTail { int8_t t[2]; ONEMACHINE_STATE_NAME(n_t, "t");
  template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_t(), s.t); } };
// a step that touches every element: the array is moved along by one, the counter counts
struct StepArr { template<class B, class P> static SlotArr run(const B&, const P& p) {
  SlotArr n = state::get<Arr>(p);
  for (unsigned i = 0; i < 3; i++) { n.band[i] = n.band[i + 1]; }
  n.band[3] = uint8_t(n.n);
  for (unsigned i = 0; i < 3; i++) { n.w[i] = int16_t(n.w[i] - 1); }
  n.f[0] = !n.f[0]; n.q[1] = n.q[0] * 3u + 1u; n.n = uint16_t(n.n + 1);
  return n; } };
using ArrNet = hapi::APIOf<state::API, state::Layer<Arr,SlotArr,StepArr>, state::Layer<Tail,SlotTail>>::Res;
inline ArrNet arrInitial() {
  ArrNet r{}; SlotArr& a = state::get<Arr>(r);
  a.band[0] = 1; a.band[1] = 200; a.band[2] = 3; a.band[3] = 255; a.w[0] = -1; a.w[1] = 0x1234; a.w[2] = -32768; a.f[0] = true; a.f[1] = false;
  a.q[0] = 0x0123456789ABCDEFull; a.q[1] = ~0ull; a.n = 65534; state::get<Tail>(r).t[0] = -128; state::get<Tail>(r).t[1] = 127; return r;
}
