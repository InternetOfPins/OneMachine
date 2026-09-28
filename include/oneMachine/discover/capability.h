// Capability -> consumers as a static fold. A capability is a tag type with `Value` and `id`;
// a consumer declares `Accepts` (Chain of tags) and `template<Cap> Body`; a driver declares `Produces`.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>

namespace discover {

  using RowId = uint8_t;
  inline constexpr RowId noRow = 0xFF;

  // The fan-out per capability: a chain of subscribers ending in a no-op. Give a subscriber a concrete
  // on(const Sample<Cap>&); the fold wiring below is identical for every one.
  template<typename Msg>
  struct FanoutEnd { static void deliver(const Msg&) {} };

  template<typename Msg, typename Body>
  struct Subscriber {
    template<typename T>
    struct Part : Body::template Part<T> {
      using Base = typename Body::template Part<T>;
      using Base::Base;
      void deliver(const Msg& m) { Base::deliver(m); this->on(m); }   // upstream subscribers first (fold order), then this one
    };
  };

  template<typename Msg, typename... Subs>
  using LocalFanout = typename hapi::Chain<Subs...>::template Part<FanoutEnd<Msg>>;

  template<typename Cap>
  struct Sample { RowId row; typename Cap::Value value; };

  // predicate: does O::List (selected by Sel) contain Cap. Same shape as hapi::SameAs.
  struct SelAccepts  { template<typename O> using Of = typename O::Accepts; };
  struct SelProduces { template<typename O> using Of = typename O::Produces; };

  template<typename Cap, typename Sel>
  struct ListHas {
    template<typename O> using Check = typename hapi::Traverse<ListHas<Cap,Sel>,O>::Beta;
    template<typename O> using Apply = hapi::Exists<hapi::SameAs<Cap>, typename Sel::template Of<O>>;
    template<typename... OO> using ApplyPack = hapi::Chain<OO...>;
  };
  template<typename Cap> using AcceptsCap  = ListHas<Cap, SelAccepts>;
  template<typename Cap> using ProducesCap = ListHas<Cap, SelProduces>;

  // capability -> the static fan-out over the consumers that accept it
  template<typename Cap, typename Consumers>
  struct CapFanout {
    using Accepting = hapi::Eval<hapi::Filter<AcceptsCap<Cap>>, Consumers>;
    template<typename C> using SubOf = Subscriber<Sample<Cap>, typename C::template Body<Cap>>;
    template<typename... SS> using Fold = LocalFanout<Sample<Cap>, SS...>;
    using Type = typename Accepting::template Map<SubOf>::template Build<Fold>;
  };
  template<typename Cap, typename Consumers>
  using CapFanoutT = typename CapFanout<Cap, Consumers>::Type;

  // drivers of `Drivers` that produce Cap
  template<typename Cap, typename Drivers>
  using ProducersOf = hapi::Eval<hapi::Filter<ProducesCap<Cap>>, Drivers>;

}
