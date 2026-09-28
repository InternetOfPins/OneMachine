// zero_cost.sh subject: the same topic-only composition, built once with every rosCompose header in the include
// path (-DROS_ALL_HEADERS pulls in action.h, which pulls in service.h and transport.h, plus qos.h) and once with
// only transport.h ever seen at all. Neither Service, Client, ActionServer nor the QoS decorators are instantiated
// here -- templates never used are never emitted ([temp.alias]) -- so the two builds must be byte-for-byte the
// same in every section and every symbol name: a header being reachable costs nothing until something in it is
// actually composed into a type. Not a program in its own right; round1.cpp is where the composed types are real.
#include <stdint.h>
#include <hapi/hapi.h>
#ifdef ROS_ALL_HEADERS
#include <oneMachine/rosCompose/action.h>
#include <oneMachine/rosCompose/qos.h>
#else
#include <oneMachine/rosCompose/transport.h>
#endif

using namespace rosCompose;
struct Twist { int x; };
struct Echo { template<typename T> struct Part : T { volatile int seen = 0; void on(const Twist& m) { seen = m.x; } }; };
using Topic = LocalFanout<Twist, Subscriber<Twist, Echo>>;

extern "C" void __cxa_pure_virtual() { for (;;) {} }
volatile uint16_t g_sum;
__attribute__((noinline)) void done() { for (;;) {} }

int main() {
  Topic t;
  t.deliver(Twist{5});
  g_sum = 1;
  done();
}
