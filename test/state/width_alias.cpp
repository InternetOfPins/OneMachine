// Why "no plain int in a slot" cannot be a type-level rule: the fixed-width names are typedefs of the fundamental types, and which
// one differs per target. int16_t is `int` on AVR; int32_t is `int` on the host. Compiles on both, each asserting its own fact.
#include <stdint.h>
template<class A, class B> struct Same { static constexpr bool value = false; };
template<class A> struct Same<A, A> { static constexpr bool value = true; };
#ifdef __AVR__
static_assert(Same<int16_t, int>::value, "on AVR, int16_t is int");
static_assert(!Same<int32_t, int>::value && Same<int32_t, long>::value, "on AVR, int32_t is long");
#else
static_assert(Same<int32_t, int>::value, "on the host, int32_t is int");
static_assert(!Same<int16_t, int>::value, "on the host, int16_t is short");
#endif
int main() {}
