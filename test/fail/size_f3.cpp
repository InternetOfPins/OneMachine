// failCompose F3 on AVR: what each consumer class costs, and that a consumer declared through the delivery API with no layer costs nothing.
// One image per -DF3_x set: P a hand-written consumer (the baseline), BD/BS/BR the same sinks behind Delivered with no layer, D/S/F/X the
// display, storage, fire-and-forget and direct-with-status classes. The same session runs in every image (14 samples, the ring stalls for
// samples 4-7, the card is pulled for 6-9); -DF3_PARITY builds it natively and prints the checksum simavr must reproduce.
#include <stdint.h>
#include <hapi/hapi.h>
#include "../support/f3cfg.h"
#include <oneMachine/discover/identify.h>

using discover::Sample;

// a consumer written by hand, calling its sink from on(): what a consumer is without the delivery API
struct PlainC {
  using Accepts = hapi::Chain<Temperature>;
  template<typename Cap> struct Body {
    template<typename T> struct Part : T {
      using T::T;
      void on(const Sample<Cap>& s) { (void)Display::deliver(fail::Rec{Cap::id, s.row, int32_t(s.value)}, 0); }
    };
  };
};

struct BareD { using Sink = Display; using Accepts = hapi::Chain<Temperature>; static constexpr Consumer cls = Consumer::Direct; static constexpr uint8_t retryMask = 0, Burst = 1; template<typename E> using Stack = Bare; };
struct BareS { using Sink = Sd;      using Accepts = hapi::Chain<Temperature>; static constexpr Consumer cls = Consumer::Direct; static constexpr uint8_t retryMask = 0, Burst = 1; template<typename E> using Stack = Bare; };
struct BareR { using Sink = Ring;    using Accepts = hapi::Chain<Temperature>; static constexpr Consumer cls = Consumer::Direct; static constexpr uint8_t retryMask = 0, Burst = 1; template<typename E> using Stack = Bare; };
using BdC = Delivered<BareD>; using BsC = Delivered<BareS>; using BrC = Delivered<BareR>;
using XC  = Delivered<SdDirectCfg>;

template<typename... L> struct CatT;
template<> struct CatT<> { using Type = hapi::Chain<>; };
template<typename... A> struct CatT<hapi::Chain<A...>> { using Type = hapi::Chain<A...>; };
template<typename... A, typename... B, typename... R> struct CatT<hapi::Chain<A...>, hapi::Chain<B...>, R...> { using Type = typename CatT<hapi::Chain<A..., B...>, R...>::Type; };

#ifdef F3_P
  using GP = hapi::Chain<PlainC>;
#else
  using GP = hapi::Chain<>;
#endif
#ifdef F3_BD
  using GBD = hapi::Chain<BdC>;
#else
  using GBD = hapi::Chain<>;
#endif
#ifdef F3_BS
  using GBS = hapi::Chain<BsC>;
#else
  using GBS = hapi::Chain<>;
#endif
#ifdef F3_BR
  using GBR = hapi::Chain<BrC>;
#else
  using GBR = hapi::Chain<>;
#endif
#ifdef F3_D
  using GD = hapi::Chain<DispC>;
#else
  using GD = hapi::Chain<>;
#endif
#ifdef F3_S
  using GS = hapi::Chain<SdC>;
#else
  using GS = hapi::Chain<>;
#endif
#ifdef F3_F
  using GF = hapi::Chain<FfC>;
#else
  using GF = hapi::Chain<>;
#endif
#ifdef F3_X
  using GX = hapi::Chain<XC>;
#else
  using GX = hapi::Chain<>;
#endif
using Consumers = typename CatT<GP, GBD, GBS, GBR, GD, GS, GF, GX>::Type;

struct App : discover::World<App, mock::Twi, Consumers, hapi::Chain<SensorB<App>, Mux<App>>, 8, discover::I2cScan> {};

static uint16_t mix(uint16_t h, uint32_t v) { return uint16_t(h * 31u + uint16_t(v) + uint16_t(v >> 16)); }

static uint16_t checksum() {
  uint16_t h = 0;
  for (discover::RowId r = 0; r < App::reg.count; ++r) h = mix(h, uint32_t(App::reg.rows[r].busId) + App::reg.rows[r].parent * 7u + App::reg.rows[r].isBus);
  h = mix(h, Display::updates); h = mix(h, uint32_t(Display::shown.v));
  h = mix(h, Sd::n); for (uint8_t i = 0; i < Sd::n; ++i) h = mix(h, uint32_t(Sd::log[i].v));
  h = mix(h, Ring::n); for (uint8_t i = 0; i < Ring::n; ++i) h = mix(h, uint32_t(Ring::got[i].v));
  h = mix(h, Sd::begins); h = mix(h, Sd::refusedWrites);
#ifdef F3_D
  h = mix(h, DispE::stats().offered); h = mix(h, DispE::stats().replaced);
#endif
#ifdef F3_S
  h = mix(h, SdE::stats().offered); h = mix(h, SdE::stats().refused); h = mix(h, SdE::stats().delivered); h = mix(h, SdE::failStatus().retries);
#endif
#ifdef F3_F
  h = mix(h, FfE::stats().offered); h = mix(h, FfE::stats().refused);
#endif
#ifdef F3_X
  h = mix(h, XC::Edge::stats().offered); h = mix(h, XC::Edge::stats().failed);
#endif
  return h;
}

static void session() {
  Sd::rebegin();
  App::discover();
  uint32_t T = 0;
  for (uint8_t k = 1; k <= 14; ++k) {
    if (k == 4) Ring::stalled = true;
    if (k == 8) Ring::stalled = false;
    if (k == 6) Sd::pull();
    if (k == 10) Sd::insert();
    mock::Bus::poke(-1, 0x40, 2, k);
    App::pump();
    for (uint8_t j = 0; j < 5; ++j) { T += 20; DeliveryTicks<Consumers>::run(T); }
  }
  for (uint8_t j = 0; j < 60; ++j) { T += 100; DeliveryTicks<Consumers>::run(T); }
}

#ifdef __AVR__
extern "C" void __cxa_pure_virtual() { for (;;) {} }
volatile uint16_t g_sum;
__attribute__((noinline)) void done() { for (;;) {} }
int main() { session(); g_sum = checksum(); done(); }
#else
#include <cstdio>
int main() { session(); std::printf("checksum 0x%04X\n", checksum()); return 0; }
#endif
