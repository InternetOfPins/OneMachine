// The net as a whole AVR program, typed or hand-indexed, with the same harness; the two must be the same image.
//   -DFLAT  hand-indexed (add -DFLAT_CAST for the pointer-cast accessors)     default: typed
//   -DINLINE  let the step inline into main                                  default: a noinline extern "C" run_step
//   -DHASH  also publish the schema hash                                      -DFACE  also use the layer and field names at run time
#include "net.h"
#include "flat.h"
volatile int16_t in_a, in_b; volatile int16_t out_a, out_b; volatile uint16_t out_steps; volatile uint8_t out_odd;
#ifdef INLINE
  #define NOINL inline __attribute__((always_inline))   // the wrapper is forced into main: the comparison is of code, not of the inliner's estimate
#else
  #define NOINL __attribute__((noinline))
#endif
#ifdef FLAT
extern "C" NOINL void run_step(const Flat* p, Flat* n) { flat_step(*p, *n); }
#else
extern "C" NOINL void run_step(const Net* p, Net* n) { n->step(*p); }
#endif
#ifdef HASH
volatile uint32_t out_hash;
#endif
#ifdef FACE
const char* volatile out_name;
struct Face { void layer(state::Name n) { out_name = n.p; } template<class T> void operator()(state::Name n, T&) { out_name = n.p; } };
#endif
int main() {
#ifdef HASH
  out_hash = state::schema_v<Net>;
#endif
#ifdef FLAT
  Flat p{}, n{}; wr16(p.v + F_A, in_a); wr16(p.v + F_B, in_b);
  for (;;) { run_step(&p, &n); p = n; out_a = rd16(p.v + F_A); out_b = rd16(p.v + F_B); out_steps = rdu16(p.v + F_STEPS); out_odd = p.v[F_ODD]; }
#else
  Net p{}, n{}; state::get<FibA>(p).a = in_a; state::get<FibB>(p).b = in_b;
  for (;;) { run_step(&p, &n); p = n; out_a = state::get<FibA>(p).a; out_b = state::get<FibB>(p).b; out_steps = state::get<Watch>(p).steps; out_odd = state::get<Watch>(p).odd;
#ifdef FACE
    Face f; p.each(f);
#endif
  }
#endif
}
