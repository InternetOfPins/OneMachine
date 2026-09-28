// The consumers of F3's applications: the classes' stacks, over the mock consumers of mockdev.h. Shared by logic_f3 and net_f3.
#pragma once
#include <oneMachine/discover/registry.h>
#include "mockTwi.h"
#include "sensors.h"
#include "mockdev.h"

using namespace fail;
using namespace mockdev;

struct DispCfg {
  using Sink = Display; using Accepts = hapi::Chain<Temperature>;
  static constexpr Consumer cls = Consumer::Display;
  static constexpr uint8_t retryMask = 0, Burst = 2;
  template<typename E> using Stack = Controller<E, Latest<>, LazyStatus>;
};
struct SdCfg {
  using Sink = Sd; using Accepts = hapi::Chain<Temperature>;
  static constexpr Consumer cls = Consumer::Storage;
  static constexpr uint8_t retryMask = KindSet<Kind::Fault, Kind::Timeout, Kind::Absent>::mask, Burst = 4;
  template<typename E> using Stack = Controller<E, Buffer<4>, TickPart<Retry<0>>, HoldOp<RejectNewest>, Backoff<100, 800>, Recover, LazyStatus>;
};
struct FfCfg {
  using Sink = Ring; using Accepts = hapi::Chain<Temperature>;
  static constexpr Consumer cls = Consumer::FireForget;
  static constexpr uint8_t retryMask = 0, Burst = 8;
  template<typename E> using Stack = Controller<E, Buffer<3>, LazyStatus>;
};
struct SdDirectCfg {                                        // the same card, with no store between it and the fan-out
  using Sink = Sd; using Accepts = hapi::Chain<Temperature>;
  static constexpr Consumer cls = Consumer::Direct;
  static constexpr uint8_t retryMask = 0, Burst = 1;
  template<typename E> using Stack = Controller<E, DetectError, LazyStatus>;
};


using DispC = fail::Delivered<DispCfg>;  using SdC = fail::Delivered<SdCfg>;  using FfC = fail::Delivered<FfCfg>;
using DispE = DispC::Edge;               using SdE = SdC::Edge;               using FfE = FfC::Edge;
