// The link's ops over the published nodes of a machine tree. The framing and the bytes are role/link.h's: this is an App with payload ops
// (`payload = true`: role::Link calls describe() for op 'd' and request() for the others).
//   'd'  the description, by its hash (the default): `hash <8 hex digits>\n` then one digit per code, the status of the row it is bound to now
//        (0 alive, 1 stale, 2 gone), then `\n`. The hash is FNV-1a over the description's text without the status, computed at compile time
//        (bmpm::HashWalk, Extra::describeStatic); the consumer has the text from the build (examples/spi/describe.cpp writes <hash>.txt; python: Tree(link, descriptions=dir)).
//        With -DONEMACHINE_DESC_TEXT: the text itself, bmpm::describe's walk then the codes that only notify (Extra), each line ending with the
//        status of its row. Codes are numbered in the order they are listed.
//   'v'  get by code           payload: the code (text)                       reply: the status (u8), then the value (i32 little-endian)
//        The status is the device's: Alive 0, Stale 1, Gone 2. A part that is not Alive has no live value: a register answers the last value set (its
//        capture, the intent that comes back), a sensor value the last one measured. A group or an event has no value: the status alone, with NoValue.
//   'w'  set by code           payload: the value (i32), then the code        reply: nothing; the set goes through the node: its limits, its capture,
//                                                                              its register (a part that is gone keeps it as the last intent)
//   'n'  changes since         payload: none                                  reply: u8 how many were refused since the last 'n', then per change
//                                                                              the code's number (u8) and its value (i32); a number with bit 7 set
//                                                                              is a status change: the code that stands for its row, and the new status
//   status  Ok, BadLength, BadValue (outside the node's limits), NoValue, ReadOnly, Unknown (no such code or op)
// What changed is told by the published nodes' OnSync/OnChange functions (note(code number, value)) and by watch() (a row's status): into a
// fail::Buffer<N>, a store that refuses the newest when it is full and counts the refusal. The consumer is told how many it missed (the u8 in the
// reply) and reads the values it follows again.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/fail/delivery.h>
#include <oneMachine/role/link.h>
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
    static void noteStatus(uint8_t code, uint8_t st) { note(uint8_t(0x80 | code), st); }
  };

  // a code carries its number (`static constexpr uint8_t num`), the one the change records use; the numbers are the codes' positions in the list
  template<unsigned I, typename... P> struct NumberedFrom : std::true_type {};
  template<unsigned I, typename H, typename... R> struct NumberedFrom<I, H, R...> : std::bool_constant<H::PubCode::num == I && NumberedFrom<I + 1, R...>::value> {};
  template<typename L> struct Numbered;
  template<typename... P> struct Numbered<hapi::Chain<P...>> : NumberedFrom<0, P...> {};
  template<unsigned I, typename... P> struct CodesFrom : std::true_type {};
  template<unsigned I, typename H, typename... R> struct CodesFrom<I, H, R...> : std::bool_constant<H::num == I && CodesFrom<I + 1, R...>::value> {};

  // where a published node's status comes from: the machine it is reached through (its device's row); void: it has none and is always Alive
  template<typename Pub, typename = void> struct SrcOf { using type = void; };
  template<typename Pub> struct SrcOf<Pub, std::void_t<typename Pub::Path::Src>> { using type = typename Pub::Path::Src; };
  // the first code of the list that has source S: the code that stands for the row when its status changes
  template<unsigned I, typename S, typename... P> struct FirstSrc { static constexpr int value = -1; };
  template<unsigned I, typename S, typename H, typename... R> struct FirstSrc<I, S, H, R...> {
    static constexpr int value = std::is_same<typename SrcOf<H>::type, S>::value ? int(I) : FirstSrc<I + 1, S, R...>::value;
  };
  // M: the machine; Pubs: Chain<published nodes>; Extra: the codes that only notify (Extra::Codes: Chain of code tags with `num` after Pubs',
  // describe(put) lists them, status(i) is the status of the i-th); bus: the machine's position in the App
  template<typename M, typename Pubs, typename Extra, uint8_t Bus, uint8_t N = 8>
  struct TreeOps {
    static_assert(Numbered<Pubs>::value, "the published codes' numbers (Code::num) must be their positions in the list");
    static constexpr uint8_t numPubs = uint8_t(Pubs::size);
    static constexpr uint8_t numCodes = uint8_t(numPubs + Extra::Codes::size);
    static constexpr bool payload = true;
    using Queue = ChangeQueue<N>;

    // the description's hash: its static text (bmpm::describeStatic, then Extra::describeStatic), folded at compile time
    static constexpr uint32_t hash() { bmpm::Fnv f; bmpm::describeStatic<M, Pubs>(f, Bus); Extra::describeStatic(f); return f.h; }
    template<typename P> static void describe(P& put) {
#ifdef ONEMACHINE_DESC_TEXT
      bmpm::describe<M, Pubs>(put, Bus); Extra::describe(put);
#else
      constexpr uint32_t descHash = hash();   // a constant: the walk is not in the image
      put('h'); put('a'); put('s'); put('h'); put(' ');
      for (int i = 28; i >= 0; i -= 4) { const uint8_t d = uint8_t((descHash >> i) & 15); put(char(d < 10 ? '0' + d : 'a' + d - 10)); }
      put('\n');
      for (uint8_t c = 0; c < numCodes; ++c) put(char('0' + status(c)));
      put('\n');
#endif
    }

    // the code in the payload, against each code tag's name: true when it is that code
    template<typename Code> static bool named(const uint8_t* s, uint16_t n) {
      const state::Name nm = Code::name();
      for (uint16_t i = 0; i < n; ++i) if (char(s[i]) != nm.rom(i)) return false;
      return nm.rom(n) == 0;
    }
    template<typename... Pub> static int findPub(hapi::Chain<Pub...>*, const uint8_t* s, uint16_t n) {
      int i = 0, hit = -1;
      ((named<typename Pub::PubCode>(s, n) ? (hit = i, ++i) : ++i), ...);
      return hit;
    }
    template<typename... C> static int findExtra(hapi::Chain<C...>*, const uint8_t* s, uint16_t n) {
      int i = numPubs, hit = -1;
      ((named<C>(s, n) ? (hit = i, ++i) : ++i), ...);
      return hit;
    }
    static int find(const uint8_t* s, uint16_t n) {
      const int p = findPub(static_cast<Pubs*>(nullptr), s, n);
      return p >= 0 ? p : findExtra(static_cast<typename Extra::Codes*>(nullptr), s, n);
    }

    // ---- status: the row a code is bound to ---------------------------------------------------------------------------
    template<typename Pub> static uint8_t statusOne() {
      using S = typename SrcOf<Pub>::type;
      if constexpr (std::is_void<S>::value) return 0; else return S::status();
    }
    template<typename... Pub> static uint8_t statusPubs(hapi::Chain<Pub...>*, int at) {
      int i = 0; uint8_t st = 0;
      ((i++ == at ? (st = statusOne<Pub>(), 0) : 0), ...);
      return st;
    }
    static uint8_t status(int code) { return code < numPubs ? statusPubs(static_cast<Pubs*>(nullptr), code) : Extra::status(uint8_t(code - numPubs)); }
    template<typename... Pub> static int repPubs(hapi::Chain<Pub...>*, int at) {
      int i = 0, r = at;
      ((i++ == at ? (r = FirstSrc<0, typename SrcOf<Pub>::type, Pub...>::value, 0) : 0), ...);
      return r;
    }
    static int rep(int code) { return code < numPubs ? repPubs(static_cast<Pubs*>(nullptr), code) : code; }

    // The loop's step: a row whose status moved is a change, told once through the code that stands for it. The first call only learns the statuses.
    static void watch() {
      static uint8_t last[numCodes ? numCodes : 1];
      static bool seeded = false;
      for (uint8_t c = 0; c < numCodes; ++c) {
        if (rep(c) != c) continue;
        const uint8_t s = status(c);
        if (seeded && s != last[c]) Queue::noteStatus(c, s);
        last[c] = s;
      }
      seeded = true;
    }

    // ---- get and set ------------------------------------------------------------------------------------------------------
    template<typename Pub> static uint8_t getOne(int32_t& v, uint8_t st) {
      using Node = typename Pub::Inner;
      if constexpr (IsGroup<Node>::value) return role::LinkNoValue;
      else {
        if constexpr (HasLast<Pub>::value) v = st == 0 ? int32_t(Pub{}.get()) : int32_t(Pub::last());
        else v = int32_t(Pub{}.get());
        return role::LinkOk;
      }
    }
    template<typename Pub> static uint8_t setOne(int32_t v) {
      using Node = typename Pub::Inner;
      if constexpr (IsGroup<Node>::value) return role::LinkNoValue;
      else if constexpr (!HasSet<Node>::value) return role::LinkReadOnly;
      else {
        if constexpr (HasLimits<Node>::value) { if (v < Node::limLo || v > Node::limHi) return role::LinkBadValue; }
        Pub::set(v);
        return role::LinkOk;
      }
    }
    template<typename T, typename = void> struct HasLast : std::false_type {};
    template<typename T> struct HasLast<T, std::void_t<decltype(T::last())>> : std::true_type {};
    template<typename... Pub> static uint8_t get(hapi::Chain<Pub...>*, int at, int32_t& v, uint8_t st) {
      int i = 0; uint8_t r = role::LinkUnknown;
      ((i++ == at ? (r = getOne<Pub>(v, st), 0) : 0), ...);
      return r;
    }
    template<typename... Pub> static uint8_t set(hapi::Chain<Pub...>*, int at, int32_t v) {
      int i = 0; uint8_t r = role::LinkUnknown;
      ((i++ == at ? (r = setOne<Pub>(v), 0) : 0), ...);
      return r;
    }

    template<typename R> static void request(uint8_t op, const uint8_t* in, uint16_t n, R& r) {
      static Pubs* const list = nullptr;
      switch (op) {
        case 'v': {
          const int at = find(in, n);
          if (at < 0) { r.status(role::LinkUnknown); return; }
          const uint8_t st = status(at);
          int32_t v = 0;
          const uint8_t ls = at < numPubs ? get(list, at, v, st) : role::LinkNoValue;
          r.status(ls); r.put(st); if (ls == role::LinkOk) r.put32(v);
          return;
        }
        case 'w': {
          if (n < 5) { r.status(role::LinkBadLength); return; }
          const int32_t v = int32_t(uint32_t(in[0]) | uint32_t(in[1]) << 8 | uint32_t(in[2]) << 16 | uint32_t(in[3]) << 24);
          const int at = find(in + 4, uint16_t(n - 4));
          if (at < 0) { r.status(role::LinkUnknown); return; }
          r.status(at < numPubs ? set(list, at, v) : role::LinkReadOnly);
          return;
        }
        case 'n': {
          auto& q = Queue::q;
          const uint8_t drops = q.status().drops;
          r.status(role::LinkOk); r.put(uint8_t(drops - Queue::seenDrops)); Queue::seenDrops = drops;
          while (!q.empty() && unsigned(r.n) + 5 <= sizeof r.data) { const fail::Rec& c = q.front(); r.put(c.cap); r.put32(c.v); q.pop(); }
          return;
        }
        default: r.status(role::LinkUnknown); return;
      }
    }
  };

}
