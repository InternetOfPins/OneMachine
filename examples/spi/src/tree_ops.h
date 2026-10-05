// The link's ops over the published nodes of a machine tree (the framing is link/frame.h; the bytes are role/link.h's):
//   'd'  the description: bmpm::describe's text, then the codes that only notify (App's Extra). Codes are numbered in the order they are listed.
//   'v'  get by code           payload: the code (text)                       reply: the value, i32 little-endian
//   'w'  set by code           payload: the value (i32), then the code        reply: nothing; the set goes through the node: its limits, its capture,
//                                                                              its register
//   'n'  changes since         payload: none                                  reply: u8 how many were refused since the last 'n', then per change
//                                                                              the code's number (u8) and its value (i32)
//   status  Ok, BadLength, BadValue (outside the node's limits), NoValue (a group has no value), ReadOnly, Unknown (no such code or op)
// What changed is told by the published nodes' OnSync/OnChange functions (note(code number, value)) into a fail::Buffer<N>: a store that refuses the
// newest when it is full and counts the refusal. The consumer is told how many it missed (the u8 in the reply) and reads the values it follows again.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/fail/delivery.h>
#include <oneMachine/link/frame.h>
#include "bmp280_machine.h"

namespace bmpm {

  // the changes waiting to be read: a Buffer of N records under a status that counts what it refused
  template<uint8_t N = 8>
  struct ChangeQueue {
    struct Env {
      static constexpr uint8_t retryMask = 0;
      static constexpr bool rawStatus = true, returnPath = false, idempotent = false, async = false;
      static void busReset() {}
      static void reissue(fail::RowId) {}
      static fail::Outcome reprobe(fail::RowId) { return fail::Outcome::Ok(); }
      static void setRowState(fail::RowId, uint8_t) {}
    };
    using Q = fail::Controller<Env, fail::Buffer<N>, fail::LazyStatus>;
    inline static Q q;
    inline static uint8_t seenDrops = 0;
    static void note(uint8_t code, int32_t v) { (void)q.offer(fail::Rec{code, 0, v}); }
  };

  // a code carries its number (`static constexpr uint8_t num`), the one the change records use; the numbers are the codes' positions in the list
  template<unsigned I, typename... P> struct NumberedFrom : std::true_type {};
  template<unsigned I, typename H, typename... R> struct NumberedFrom<I, H, R...> : std::bool_constant<H::PubCode::num == I && NumberedFrom<I + 1, R...>::value> {};
  template<typename L> struct Numbered;
  template<typename... P> struct Numbered<hapi::Chain<P...>> : NumberedFrom<0, P...> {};

  // M: the machine; Pubs: Chain<published nodes>; Extra: codes that only notify (describe(put) lists them; they take the numbers after Pubs');
  // bus: the machine's position in the App
  template<typename M, typename Pubs, typename Extra, uint8_t Bus, uint8_t N = 8>
  struct TreeOps {
    static_assert(Numbered<Pubs>::value, "the published codes' numbers (Code::num) must be their positions in the list");
    using Queue = ChangeQueue<N>;
    static constexpr uint8_t numPubs = uint8_t(Pubs::size);

    template<typename P> static void describe(P& put) { bmpm::describe<M, Pubs>(put, Bus); Extra::describe(put); }

    // the code in the payload, against each published node's: its number, or -1
    template<typename Pub> static bool is(const uint8_t* s, uint16_t n) {
      const state::Name nm = Pub::PubCode::name();
      for (uint16_t i = 0; i < n; ++i) if (char(s[i]) != nm.rom(i)) return false;
      return nm.rom(n) == 0;
    }
    template<typename... Pub> static int find(hapi::Chain<Pub...>*, const uint8_t* s, uint16_t n) {
      int i = 0, hit = -1;
      ((is<Pub>(s, n) ? (hit = i, ++i) : ++i), ...);
      return hit;
    }

    template<typename Pub> static uint8_t getOne(int32_t& v) {
      using Node = typename Pub::Inner;
      if constexpr (IsGroup<Node>::value) return link::NoValue;
      else { v = int32_t(Pub{}.get()); return link::Ok; }
    }
    template<typename Pub> static uint8_t setOne(int32_t v) {
      using Node = typename Pub::Inner;
      if constexpr (IsGroup<Node>::value) return link::NoValue;
      else if constexpr (!HasSet<Node>::value) return link::ReadOnly;
      else {
        if constexpr (HasLimits<Node>::value) { if (v < Node::limLo || v > Node::limHi) return link::BadValue; }
        Pub::set(v);
        return link::Ok;
      }
    }
    // the node at position `at` of the list
    template<typename... Pub> static uint8_t get(hapi::Chain<Pub...>*, int at, int32_t& v) {
      int i = 0; uint8_t st = link::Unknown;
      ((i++ == at ? (st = getOne<Pub>(v), 0) : 0), ...);
      return st;
    }
    template<typename... Pub> static uint8_t set(hapi::Chain<Pub...>*, int at, int32_t v) {
      int i = 0; uint8_t st = link::Unknown;
      ((i++ == at ? (st = setOne<Pub>(v), 0) : 0), ...);
      return st;
    }

    template<typename R> static void request(uint8_t op, const uint8_t* in, uint16_t n, R& r) {
      static Pubs* const list = nullptr;
      switch (op) {
        case 'v': {
          const int at = find(list, in, n);
          if (at < 0) { r.status(link::Unknown); return; }
          int32_t v = 0; const uint8_t st = get(list, at, v);
          r.status(st); if (st == link::Ok) r.put32(v);
          return;
        }
        case 'w': {
          if (n < 5) { r.status(link::BadLength); return; }
          const int32_t v = int32_t(uint32_t(in[0]) | uint32_t(in[1]) << 8 | uint32_t(in[2]) << 16 | uint32_t(in[3]) << 24);
          const int at = find(list, in + 4, uint16_t(n - 4));
          if (at < 0) { r.status(link::Unknown); return; }
          r.status(set(list, at, v));
          return;
        }
        case 'n': {
          auto& q = Queue::q;
          const uint8_t drops = q.status().drops;
          r.status(link::Ok); r.put(uint8_t(drops - Queue::seenDrops)); Queue::seenDrops = drops;
          while (!q.empty() && unsigned(r.n) + 5 <= sizeof r.data) { const fail::Rec& c = q.front(); r.put(c.cap); r.put32(c.v); q.pop(); }
          return;
        }
        default: r.status(link::Unknown); return;
      }
    }
  };

}
