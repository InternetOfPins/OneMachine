// OneMachine rosCompose verification, round 1: a topic (2 local subscribers), a service (client/server round trip),
// an action (goal lifecycle including cancel) and QoS history, composed as one script with a checksum -- native and
// AVR run the identical script and must agree. WithDeadline (qos.h, needs a real millisecond clock) is out of scope
// here: it is real-time-dependent, not script-driven like everything else in this file, and untested since it was
// written; leaving it untested here too rather than making the round flaky is a scoping decision, not a gap closed.
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/rosCompose/action.h>
#include <oneMachine/rosCompose/qos.h>

using namespace rosCompose;

#if !defined(__AVR__)
#include <cstdio>
static int checks = 0, failures = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #cond); } } while (0)
#else
extern "C" void __cxa_pure_virtual() { for (;;) {} }
volatile uint16_t g_sum;
__attribute__((noinline)) void done() { for (;;) {} }
static int failures = 0;
#define CHECK(cond) do { if (!(cond)) ++failures; } while (0)
#endif

struct Twist { int x; };

// ---- topic: two local subscribers ---------------------------------------------------------------------------
static int seenA[8], seenB[8], nA = 0, nB = 0;
struct SubA { template<typename T> struct Part : T { void on(const Twist& m) { seenA[nA++] = m.x; } }; };
struct SubB { template<typename T> struct Part : T { void on(const Twist& m) { seenB[nB++] = m.x; } }; };
using CmdVelSubs = LocalFanout<Twist, Subscriber<Twist, SubA>, Subscriber<Twist, SubB>>;

// ---- service: doubler ----------------------------------------------------------------------------------------
static int lastResult = -1;
static int doubler(const int& v) { return v * 2; }
static void onDoubled(const int& r) { lastResult = r; }

// ---- QoS: KEEP_LAST history over a 3rd subscriber -------------------------------------------------------------
struct SubC { template<typename T> struct Part : T { void on(const Twist&) {} }; };
using HistTopic = LocalFanout<Twist, Subscriber<Twist, WithHistory<Twist, 3, SubC>>>;

static uint16_t checksum() {
  uint16_t s = 0;
  for (int i = 0; i < nA; ++i) s = uint16_t(s * 31 + seenA[i]);
  for (int i = 0; i < nB; ++i) s = uint16_t(s * 31 + seenB[i]);
  s = uint16_t(s * 31 + lastResult);
  return s;
}

int main() {
  // topic: 3 messages, both subscribers see all 3, in order
  CmdVelSubs topic;
  topic.deliver(Twist{1}); topic.deliver(Twist{2}); topic.deliver(Twist{3});
  CHECK(nA == 3 && nB == 3);
  CHECK(seenA[0] == 1 && seenA[1] == 2 && seenA[2] == 3);
  CHECK(seenB[0] == 1 && seenB[1] == 2 && seenB[2] == 3);

  // service: one round trip through Client -> Service -> Client's callback, correlated by token
  Client<int, int, 2> client;
  Service<int, int> service;
  service.linkOut = &client;   // loopback: the service's replies go straight back to the client
  client.linkOut = &service;   // the client's requests go straight to the service
  service.handler = doubler;
  bool sent = client.call(21, onDoubled);
  CHECK(sent);
  CHECK(lastResult == 42);

  // action: goal 1 accepted -> Execute -> Succeed; goal 2 accepted -> Execute -> CancelGoal -> Canceled.
  // stateOf() only sees a goal while its slot is held (g.used): a terminal transition (Succeeded/Canceled/
  // Aborted) frees the slot in the same advance() call, so the state is checked right after each step, not
  // after the goal has terminated -- once terminal, stateOf() correctly reports Unknown, the slot reusable.
  ActionServer<int, int, int, 4> action;
  int g1 = action.accept(0);
  action.advance(g1, GoalEvent::Execute);
  CHECK(action.stateOf(g1) == GoalState::Executing);
  action.advance(g1, GoalEvent::Succeed);
  CHECK(action.stateOf(g1) == GoalState::Unknown);
  int g2 = action.accept(0);
  action.advance(g2, GoalEvent::Execute);
  action.advance(g2, GoalEvent::CancelGoal);
  CHECK(action.stateOf(g2) == GoalState::Canceling);
  action.advance(g2, GoalEvent::Canceled);
  CHECK(action.stateOf(g2) == GoalState::Unknown);
  CHECK(g1 != 0 && g2 != 0 && g1 != g2);

  // QoS: KEEP_LAST 3 over 5 pushes -- the ring keeps the 3 most recent, oldest first
  HistTopic hs;
  for (int v = 1; v <= 5; ++v) hs.deliver(Twist{v});
  CHECK(hs.history.at(0).x == 3 && hs.history.at(1).x == 4 && hs.history.at(2).x == 5);

#ifdef __AVR__
  g_sum = failures == 0 ? checksum() : 0xFFFF;
  done();
#else
  std::printf("checksum 0x%04X\n", checksum());
  std::printf(failures == 0 ? "OK: rosCompose round1, %d/%d checks (topic fan-out, service round trip, action lifecycle+cancel, QoS history)\n"
                             : "FAIL: rosCompose round1, %d/%d checks\n", checks - failures, checks);
  return failures != 0;
#endif
}
