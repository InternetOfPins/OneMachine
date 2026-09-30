// Compile-time rules of state.h: each -D case must be rejected with its own message, or is the accepted counterpart.
#include "net.h"
struct T1 { ONEMACHINE_STATE_NAME(name, "one"); };
struct T2 { ONEMACHINE_STATE_NAME(name, "two"); };
struct T2again { ONEMACHINE_STATE_NAME(name, "two"); };
struct NoNameTag {};
struct Nope {};
struct S1 { int16_t x; ONEMACHINE_STATE_NAME(n_x, "x"); template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_x(), s.x); } };
struct S2 { int16_t y; ONEMACHINE_STATE_NAME(n_y, "y"); template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_y(), s.y); } };
struct SDupField { int16_t x, y; ONEMACHINE_STATE_NAME(n_x, "x"); template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_x(), s.x); f(n_x(), s.y); } };
template<class... L> using R = typename hapi::APIOf<state::API, L...>::Res;
#if defined(DUP_TAG)
using X = R<state::Layer<T1,S1>, state::Layer<T1,S2>>;
#elif defined(DUP_LAYER_NAME)
using X = R<state::Layer<T2,S1>, state::Layer<T2again,S2>>;
#elif defined(DUP_FIELD_NAME)
using X = R<state::Layer<T1,SDupField>>;
#elif defined(TOO_MANY_LAYERS)    // build with -DONEMACHINE_STATE_MAX_NAMES=2
struct T3 { ONEMACHINE_STATE_NAME(name, "three"); };
using X = R<state::Layer<T1,S1>, state::Layer<T2,S2>, state::Layer<T3,S1>>;
#elif defined(TOO_MANY_FIELDS)    // build with -DONEMACHINE_STATE_MAX_NAMES=2
struct S3 { int16_t x, y, z; ONEMACHINE_STATE_NAME(n_x, "x"); ONEMACHINE_STATE_NAME(n_y, "y"); ONEMACHINE_STATE_NAME(n_z, "z"); template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_x(), s.x); f(n_y(), s.y); f(n_z(), s.z); } };
using X = R<state::Layer<T1,S3>>;
#elif defined(FLOAT_FIELD)
struct SFloat { float x; ONEMACHINE_STATE_NAME(n_x, "x"); template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_x(), s.x); } };
using X = R<state::Layer<T1,SFloat>>;
#elif defined(CHAR_FIELD)
struct SChar { char x; ONEMACHINE_STATE_NAME(n_x, "x"); template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_x(), s.x); } };
using X = R<state::Layer<T1,SChar>>;
#elif defined(LITERAL_NAME)       // a plain string literal is not a state::Name
struct SLit { int16_t x; template<class Self, class F> static constexpr void each(Self& s, F& f) { f("x", s.x); } };
using X = R<state::Layer<T1,SLit>>;
#elif defined(NO_NAME)
using X = R<state::Layer<NoNameTag,S1>>;
#elif defined(MISSING_TAG)
using X = Net; static int16_t f() { Net n{}; return state::get<Nope>(n).x; }
#elif defined(NEXT_OF_UPPER)      // T2 (below T1) reads the next of T1 (above it)
struct ReadsUp { template<class B, class P> static S2 run(const B& b, const P&) { return {state::get<T1>(b).x}; } };
using X = R<state::Layer<T1,S1>, state::Layer<T2,S2,ReadsUp>>;
#elif defined(PREV_OF_UPPER)      // the same read on prev: accepted
struct ReadsUp { template<class B, class P> static S2 run(const B&, const P& p) { return {state::get<T1>(p).x}; } };
using X = R<state::Layer<T1,S1>, state::Layer<T2,S2,ReadsUp>>;
#elif defined(DEPENDENT_ORDER)    // the observer listed below the layers whose next it reads
using X = hapi::APIOf<state::API, state::Layer<FibA,SlotFibA,StepA>, state::Layer<FibB,SlotFibB,StepB>, state::Layer<Watch,SlotWatch,StepWatch>>::Res;
#elif defined(OWN_NEXT)           // a layer reads its own next: it is not in `below`
struct ReadsOwn { template<class B, class P> static S1 run(const B& b, const P&) { return {state::get<T1>(b).x}; } };
using X = R<state::Layer<T1,S1,ReadsOwn>>;
#elif defined(BELOW_WRITE)        // a layer writes the slot of a layer below it
struct WritesDown { template<class B, class P> static S1 run(const B& b, const P&) { state::get<T2>(b).y = 0; return {0}; } };
using X = R<state::Layer<T1,S1,WritesDown>, state::Layer<T2,S2>>;
#elif defined(PREV_WRITE)
struct WritesPrev { template<class B, class P> static S1 run(const B&, const P& p) { state::get<T1>(p).x = 0; return {0}; } };
using X = R<state::Layer<T1,S1,WritesPrev>, state::Layer<T2,S2>>;
#else
using X = Net;
#endif
int main() { X n{}, p{}; n.step(p); return 0; }
