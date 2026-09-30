// The rules of role::Machine and the kinds, each a compile error with its own message (build.sh: rej). Built plain, it must compile.
#include <oneMachine/role/role.h>
#include <oneMachine/role/kinds.h>
#include <oneMachine/role/sim.h>

struct A { ONEMACHINE_STATE_NAME(name, "a"); };
struct B { ONEMACHINE_STATE_NAME(name, "b"); };
struct B2 { ONEMACHINE_STATE_NAME(name, "b"); };
using P1 = role::SimPin<1>; using P2 = role::SimPin<2>;
using W1 = role::SimPwm<1, 255>;

#if defined(TWO_ON_ONE)
using M = role::Machine<role::Role<A, role::Switch<>, P1>, role::Role<B, role::Switch<>, P1>>;
#elif defined(SAME_NAME)
using M = role::Machine<role::Role<B, role::Switch<>, P1>, role::Role<B2, role::Switch<>, P2>>;
#elif defined(MAX_ABOVE_TOP)
using M = role::Machine<role::Role<A, role::Light<4095>, W1>>;
#elif defined(SAFE_ABOVE_MAX)
using M = role::Machine<role::Role<A, role::Light<200, 300>, W1>>;
#elif defined(AXIS_RANGE)
using M = role::Machine<role::Role<A, role::Axis<80, 10, 10>, role::SimStepDir<0, 1>>>;
#else
using M = role::Machine<role::Role<A, role::Light<255>, W1>, role::Role<B, role::Switch<>, P1>>;
#endif

int main() { M::Command c{}; M::Report r{}; M::apply(c, r); return 0; }
