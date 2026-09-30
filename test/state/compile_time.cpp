// Compile time of a chain of N layers, two ways: -DMODE_PLAIN (N trivial HAPI Parts through APIOf: the baseline),
// -DMODE_CONTRACT (N state layers: names, step, schema hash, frame size). -DNLAYERS=<layers>. No standard library needed (own index sequence).
#include <hapi/hapi.h>
#include <stdint.h>
#ifdef MODE_CONTRACT
  #include <oneMachine/state/state.h>
  #include <oneMachine/state/wire.h>
#endif
#ifndef NLAYERS
  #define NLAYERS 32
#endif
template<unsigned... I> struct Seq {};
template<unsigned K, unsigned... I> struct MkSeq : MkSeq<K - 1, K - 1, I...> {};
template<unsigned... I> struct MkSeq<0, I...> { using type = Seq<I...>; };

#if defined(MODE_PLAIN)
template<unsigned I> struct X { template<class O> struct Part : O { }; };
struct PlainApi {};
template<unsigned... I> hapi::APIOf<PlainApi, X<I>...> make(Seq<I...>);
using State = decltype(make(MkSeq<NLAYERS>::type{}));
int main() { State s{}; (void)s; }
#elif defined(MODE_CONTRACT)
template<unsigned I> struct Tag { static constexpr char s[] ONEMACHINE_STATE_ROM = { 'L', char('0' + I / 100 % 10), char('0' + I / 10 % 10), char('0' + I % 10), 0 };
  static constexpr state::Name name() { return state::Name(s); } };
template<unsigned I> struct Slot1 { int16_t x; ONEMACHINE_STATE_NAME(n_x, "x"); template<class Self, class V> static constexpr void each(Self& s, V& v) { v(n_x(), s.x); } };
template<unsigned I> struct Stepper { template<class B, class P> static Slot1<I> run(const B&, const P& p) { return {int16_t(state::get<Tag<I>>(p).x + 1)}; } };
template<unsigned... I> hapi::APIOf<state::API, state::Layer<Tag<I>, Slot1<I>, Stepper<I>>...> make(Seq<I...>);
using State = decltype(make(MkSeq<NLAYERS>::type{}))::Res;
constexpr uint32_t kHash = state::schema_v<State>;            // the schema hash and the frame size are part of what is timed
constexpr unsigned kSize = state::wire_size<State>();
int main() { State a{}, b{}; b.step(a); return (kHash == 0 || kSize == 0) + state::get<Tag<0>>(b).x - 1; }
#endif
