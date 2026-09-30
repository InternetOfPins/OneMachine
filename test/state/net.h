// The fib net: a Fibonacci recurrence (two layers reading `prev` only) and an observer above them reading their `next`.
#pragma once
#include <oneMachine/state/state.h>

struct FibA  { ONEMACHINE_STATE_NAME(name, "fibA"); };
struct FibB  { ONEMACHINE_STATE_NAME(name, "fibB"); };
struct Watch { ONEMACHINE_STATE_NAME(name, "watch"); };

struct SlotFibA { int16_t a; ONEMACHINE_STATE_NAME(n_a, "a");
  template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_a(), s.a); } };
struct SlotFibB { int16_t b; ONEMACHINE_STATE_NAME(n_b, "b");
  template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_b(), s.b); } };
struct SlotWatch { uint16_t steps; uint8_t odd; ONEMACHINE_STATE_NAME(n_steps, "steps"); ONEMACHINE_STATE_NAME(n_odd, "odd");
  template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_steps(), s.steps); f(n_odd(), s.odd); } };

struct StepA { template<class B, class P> static SlotFibA run(const B&, const P& p)
  { return {state::get<FibB>(p).b}; } };                                                                 // a' = b
struct StepB { template<class B, class P> static SlotFibB run(const B&, const P& p)
  { return {int16_t(state::get<FibA>(p).a + state::get<FibB>(p).b)}; } };                                   // b' = a + b
struct StepWatch { template<class B, class P> static SlotWatch run(const B& below, const P& p)
  { return {uint16_t(state::get<Watch>(p).steps + 1),                                                    // its own previous value
            uint8_t((state::get<FibA>(below).a + state::get<FibB>(below).b) & 1)}; } };                     // the new a and b, this pass

using Net = hapi::APIOf<state::API, state::Layer<Watch,SlotWatch,StepWatch>, state::Layer<FibA,SlotFibA,StepA>, state::Layer<FibB,SlotFibB,StepB>>::Res;
