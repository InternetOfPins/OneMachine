// The example of the README, compiled and run: a counter and a peak over it, stepped, sent as a frame, read back.
#include <oneMachine/state/state.h>
#include <oneMachine/state/wire.h>
#include <oneMachine/state/face.h>
#include <stdio.h>

struct Count { ONEMACHINE_STATE_NAME(name, "count"); };
struct Peak  { ONEMACHINE_STATE_NAME(name, "peak"); };

struct CountSlot { uint16_t n; ONEMACHINE_STATE_NAME(n_n, "n");
  template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_n(), s.n); } };
struct PeakSlot  { uint16_t max; ONEMACHINE_STATE_NAME(n_max, "max");
  template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_max(), s.max); } };

struct CountStep { template<class Below, class Prev> static CountSlot run(const Below&, const Prev& prev)
  { return {uint16_t(state::get<Count>(prev).n + 1)}; } };
struct PeakStep  { template<class Below, class Prev> static PeakSlot run(const Below& below, const Prev& prev)
  { uint16_t n = state::get<Count>(below).n, m = state::get<Peak>(prev).max; return {n > m ? n : m}; } };

// last listed runs first: Peak reads the new Count from `below`, its own old value from `prev`
using State = hapi::APIOf<state::API, state::Layer<Peak,PeakSlot,PeakStep>, state::Layer<Count,CountSlot,CountStep>>::Res;
ONEMACHINE_STATE_PIN(State, 0xe78c2bdfu);

int main() {
  printf("0x%08xu\n", state::schema_v<State>);
  State prev{}, next{};
  next.step(prev);
  uint8_t frame[state::wire_size<State>()];
  state::write(next, frame);
  State peer{};
  bool ok = state::read(peer, frame, sizeof frame) == state::Status::Ok && state::get<Peak>(peer).max == 1;
  auto put = [](char c) { putchar(c); };
  state::describe<State>(put);
  return !ok;
}
