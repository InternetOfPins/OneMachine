// Schema hash of state.h: what changes it and what does not (compile-time only; g++, clang++, avr-g++).
#include "net.h"
#include "golden.h"
template<class... L> using R = typename hapi::APIOf<state::API, L...>::Res;
struct TA { ONEMACHINE_STATE_NAME(name, "a"); };
struct TARen { ONEMACHINE_STATE_NAME(name, "aa"); };
struct TB { ONEMACHINE_STATE_NAME(name, "b"); };
template<class T> struct One { T x;   ONEMACHINE_STATE_NAME(n_x, "x"); template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_x(), s.x); } };
template<class T> struct OneRen { T x; ONEMACHINE_STATE_NAME(n_y, "y"); template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_y(), s.x); } };
template<class T> struct Two { T x, y; ONEMACHINE_STATE_NAME(n_x, "x"); ONEMACHINE_STATE_NAME(n_y, "y"); template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_x(), s.x); f(n_y(), s.y); } };
template<class T> struct TwoSwapped { T x, y; ONEMACHINE_STATE_NAME(n_y, "y"); ONEMACHINE_STATE_NAME(n_x, "x"); template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_y(), s.y); f(n_x(), s.x); } };
template<class... L> constexpr uint32_t H() { return state::schema<R<L...>>(); }
using LA  = state::Layer<TA, One<int16_t>>;
using LB  = state::Layer<TB, Two<uint8_t>>;
static_assert(H<LA,LB>() == H<LA,LB>(),                                                     "identical compositions: equal");
static_assert(H<LA,LB>() != H<LA>(),                                                        "a layer removed");
static_assert(H<LA,LB>() != H<LB,LA>(),                                                     "layers reordered");
static_assert(H<LA,LB>() != H<state::Layer<TARen,One<int16_t>>,LB>(),                          "a layer renamed");
static_assert(H<LA,LB>() != H<state::Layer<TA,OneRen<int16_t>>,LB>(),                          "a field renamed");
static_assert(H<LA,LB>() != H<state::Layer<TA,One<int32_t>>,LB>(),                             "a field widened");
static_assert(H<LA,LB>() != H<state::Layer<TA,One<uint16_t>>,LB>(),                            "a field's signedness changed");
static_assert(H<state::Layer<TA,Two<int16_t>>>() != H<state::Layer<TA,TwoSwapped<int16_t>>>(),    "same-typed fields swapped");
static_assert(H<LA,LB>() != H<LA,LB,state::Layer<TARen,One<bool>>>(),                          "a layer added");
static_assert(state::schema_v<Net> == GOLDEN_HASH,                                             "the net's hash is the pinned one on this compiler");
int main() {}
