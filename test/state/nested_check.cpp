// Layers inside a nested composition: a nested APIOf (once, twice), a nested Chain, used as one component of an outer chain (host).
// Same size, walk order and schema hash as the flat composition; the step holds.
//   -DDUP_ACROSS  the same tag in the outer chain and inside the nested one: must not compile
#include <oneMachine/state/state.h>
#include <oneMachine/state/wire.h>
#include <stdio.h>

struct A { ONEMACHINE_STATE_NAME(name, "a"); }; struct B { ONEMACHINE_STATE_NAME(name, "b"); };
struct C { ONEMACHINE_STATE_NAME(name, "c"); }; struct D { ONEMACHINE_STATE_NAME(name, "d"); };
#define F(n) ONEMACHINE_STATE_NAME(n_##n, #n)
#define EACHDEF(...) template<class Self, class V> static constexpr void each(Self& s, V& v) { __VA_ARGS__ }
struct SA { int16_t x; F(x); EACHDEF(v(n_x(), s.x);) }; struct SB { int16_t x; F(x); EACHDEF(v(n_x(), s.x);) };
struct SC { int16_t x; F(x); EACHDEF(v(n_x(), s.x);) }; struct SD { int16_t x; F(x); EACHDEF(v(n_x(), s.x);) };
template<class T, class S> using L = state::Layer<T,S>;
using Api = state::API;

using Inner  = hapi::APIOf<Api, L<B,SB>, L<C,SC>>;                             // a closed composition
using Pair   = hapi::Chain<L<B,SB>, L<C,SC>>;                                  // an open one
#ifdef DUP_ACROSS
using Bad = hapi::APIOf<Api, L<A,SA>, hapi::APIOf<Api, L<A,SB>>, L<D,SD>>::Res;
int main() { Bad b{}; (void)b; }
#else
using WithApiOf = hapi::APIOf<Api, L<A,SA>, Inner, L<D,SD>>::Res;
using WithChain = hapi::APIOf<Api, L<A,SA>, Pair, L<D,SD>>::Res;
using Flat      = hapi::APIOf<Api, L<A,SA>, L<B,SB>, L<C,SC>, L<D,SD>>::Res;
using Deep      = hapi::APIOf<Api, L<A,SA>, hapi::APIOf<Api, hapi::APIOf<Api, L<B,SB>>, L<C,SC>>, L<D,SD>>::Res;   // nested twice
static int failures = 0;
static void check(const char* n, bool ok) { printf("CHECK %s: %s\n", n, ok ? "ok" : "FAIL"); if (!ok) failures++; }
template<class R> static bool works(R& r) {
  state::get<A>(r).x = 1; state::get<B>(r).x = 2; state::get<C>(r).x = 3; state::get<D>(r).x = 4;
  return state::get<A>(r).x == 1 && state::get<B>(r).x == 2 && state::get<C>(r).x == 3 && state::get<D>(r).x == 4;
}
struct Order { int n = 0; int x[4]; void layer(state::Name) {} template<class T> void operator()(state::Name, const T& v) { x[n++] = int(v); } };
int main() {
  WithApiOf a{}; WithChain b{}; Flat f{}; Deep d{};
  check("nested-apiof", works(a));
  check("nested-chain", works(b));
  check("nested-twice", works(d));
  check("same-size-as-flat", sizeof(a) == sizeof(f) && sizeof(b) == sizeof(f) && sizeof(d) == sizeof(f));
  { Order o; a.each(o); Order p; f.each(p); bool same = o.n == 4 && p.n == 4; for (int i = 0; same && i < 4; i++) same = o.x[i] == p.x[i]; works(a); works(f);
    Order q; a.each(q); Order w; f.each(w); same = q.n == 4; for (int i = 0; same && i < 4; i++) same = q.x[i] == w.x[i]; check("walk-order-as-flat", same); }
  check("hash-as-flat", state::schema_v<WithApiOf> == state::schema_v<Flat> && state::schema_v<WithChain> == state::schema_v<Flat> && state::schema_v<Deep> == state::schema_v<Flat>);
  check("size-as-flat", state::wire_size<WithApiOf>() == state::wire_size<Flat>() && state::wire_size<Deep>() == state::wire_size<Flat>());
  { WithApiOf p{}, n{}; state::get<A>(p).x = 5; n.step(p); check("step-holds", state::get<A>(n).x == 5 && state::get<B>(n).x == 0); }
  printf("%d failed\n", failures); return failures;
}
#endif
