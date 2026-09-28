// The delivery edge (F3): the consumer's side of a fan-out. A consumer declared through Delivered<Cfg> is an ordinary consumer for
// discoverCompose's fan-out (Accepts + Body<Cap>::on), so the fan-out is untouched; its on() offers the sample to the consumer's
// own store and returns. The sink is called from tick() only, so what a slow or failing consumer does stays behind its store.
//   Rec        what a sink is given: capability id, producing row, value
//   Latest     Store(1), the newest record replaces the one waiting; the replaced one is counted            (display)
//   Buffer<N>  Store(N), a full store refuses the newest; the refusal is counted                             (storage, fire-and-forget)
//   the failure layers of layers.h sit below the store: Retry re-issues the head record, Backoff gates it, Recover calls Sink::rebegin()
// Cfg: Sink, cls (Consumer class: the rules of layers.h reject what the class does not accept), Accepts (capabilities), retryMask,
//      Burst (records offered per tick), Stack<Env> (Controller<...> or Bare).
// Sink: static Outcome deliver(const Rec&, uint32_t now)   Ok: taken and written; Blocked: busy, keep the record; a failure: the kind says why
//       (a consumer with Stack = Bare has no clock: the sink gets now = 0)
//       static bool rebegin()                               (only where Recover is chosen)
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/discover/capability.h>
#include "layers.h"
#include "slots.h"

namespace fail {

  struct Rec { uint8_t cap; RowId row; int32_t v; };

  // ---- the stores: a queue of records as a layer -----------------------------------------------------------------------------------
  // Part: offer(rec) -> 0 refused, 1 taken, 2 taken and the newest replaced one that was waiting; empty(), front(), pop(), size().
  template<typename R = Rec>
  struct Latest : QueueTag, LatestTag, LossyTag {
    template<typename Before, typename After> static constexpr bool rules() {
      static_assert(below<StatusTag, After>, "Latest counts what it overwrites into Status: place Status below it");
      return true;
    }
    template<typename T> struct Part : T {
      R    slot{};
      bool full = false;
      uint8_t offer(const R& r) { const bool over = full; slot = r; full = true; if (over) this->noteDrop(); return over ? 2 : 1; }
      bool empty() const        { return !full; }
      const R& front() const    { return slot; }
      void pop()                { full = false; }
      uint8_t size() const      { return full ? 1 : 0; }
    };
  };

  template<uint8_t N, typename R = Rec>
  struct Buffer : QueueTag, BufferTag, LosslessTag {
    static_assert(N >= 1, "Buffer: at least one slot");
    template<typename Before, typename After> static constexpr bool rules() {
      static_assert(below<StatusTag, After>, "Buffer counts what it refuses into Status: place Status below it");
      return true;
    }
    template<typename T> struct Part : T {
      R       q[N] = {};
      uint8_t first = 0, n = 0;
      uint8_t offer(const R& r) {
        if (n == N) {
          this->noteDrop(); return 0;
        }
        uint8_t i = uint8_t(first + n); if (i >= N) i = uint8_t(i - N);
        q[i] = r; ++n;
        return 1;
      }
      bool empty() const     { return n == 0; }
      const R& front() const { return q[first]; }
      void pop()             { if (n) { --n; if (++first == N) first = 0; } }
      uint8_t size() const   { return n; }
    };
  };

  template<typename T, typename = void> struct HasHolding : std::false_type {};
  template<typename T> struct HasHolding<T, std::void_t<decltype(std::declval<const T&>().holding())>> : std::true_type {};
  template<typename T, typename = void> struct HasQueue : std::false_type {};
  template<typename T> struct HasQueue<T, std::void_t<decltype(std::declval<T&>().front())>> : std::true_type {};
  template<typename T, typename = void> struct HasRebegin : std::false_type {};
  template<typename T> struct HasRebegin<T, std::void_t<decltype(T::rebegin())>> : std::true_type {};

  struct DeliveryStats { uint16_t offered, delivered, refused, replaced, failed; uint8_t queued; };

  template<typename Cfg>
  struct DeliveryEdge {
    using Sink = typename Cfg::Sink;

    struct Env {
      static constexpr uint8_t retryMask = Cfg::retryMask;
      static constexpr bool rawStatus = true, returnPath = true, idempotent = false, async = false;
      static constexpr Consumer consumerClass = Cfg::cls;
      static void busReset()                      { if constexpr (HasRebegin<Sink>::value) (void)Sink::rebegin(); }
      static void reissue(RowId)                  { (void)DeliveryEdge::attempt(Cause::Reissue); }
      static void setRowState(RowId, uint8_t s)   { if (s == uint8_t(RowState::Gone)) DeliveryEdge::dropHead(); }     // Retry gave up: the record goes, counted
    };
    using Stack = typename Cfg::template Stack<Env>;
    static constexpr bool kBare   = std::is_same<Stack, Bare>::value;     // no layer at all: the sink is called from on(), nothing is kept
    using Tab = SlotTable<Stack, 1, SlotIsRow<1>, true>;

    inline static uint32_t nowV = 0;
    inline static uint16_t offered = 0, delivered = 0, refused = 0, replaced = 0, failed = 0, unreported = 0;
    inline static Outcome  last, lastFail;                                  // last: the last direct delivery, or the last record dropped for good (read once); lastFail: how the head record last failed
    inline static bool     haveLast = false, headFailed = false;

    // ---- the store, when there is one ------------------------------------------------------------------------------------------
    static auto& ctl() { return Tab::slots[0]; }
    static constexpr bool kQueued = !kBare && HasQueue<Stack>::value;
    static bool holding() { if constexpr (!kBare && HasHolding<Stack>::value) return ctl().holding(); else return false; }

    // ---- what the fan-out calls: O(1), never the sink (unless the class has no store) ---------------------------------------------
    static void offer(const Rec& r) {
      if constexpr (kBare) { (void)Sink::deliver(r, 0); }                   // no layer, no state, no clock: the sink is called with now = 0
      else if constexpr (!kQueued) { direct(r); }
      else {
        ++offered;
        const uint8_t k = ctl().offer(r);
        if (k == 0) { ++refused; bump(); }
        else if (k == 2) { ++replaced; bump(); }
      }
    }

    // ---- what the app calls ------------------------------------------------------------------------------------------------------
    static void tick([[maybe_unused]] uint32_t now) {
      if constexpr (!kBare) {
        nowV = now;
        if constexpr (has_tick<Stack>::value) Tab::tick(0, now);
        if constexpr (kQueued) drain();
      }
    }

    // what the delivery has come to: a record refused or replaced since the last read (once), a record dropped for good (once), the head
    // record failing now, Pending while records wait, else Ok
    static Outcome _deliver() {
      if constexpr (kBare) return Outcome::Idle();
      else {
        if (unreported) { const uint8_t n = unreported > 255 ? uint8_t(255) : uint8_t(unreported); unreported = 0; return Outcome::Fail(Kind::Overflow, n); }
        if constexpr (kQueued) {
          if (haveLast && last.failed()) { const Outcome o = last; haveLast = false; return o; }
          if (!ctl().empty()) return headFailed && holding() ? lastFail : Outcome::Pending();
          return offered ? Outcome::Ok() : Outcome::Idle();
        } else return haveLast ? last : Outcome::Idle();
      }
    }
    [[nodiscard]] static FailStatus failStatus() { return Tab::status(0); }
    [[nodiscard]] static DeliveryStats stats() {
      uint8_t q = 0; if constexpr (kQueued) q = ctl().size();
      return {offered, delivered, refused, replaced, failed, q};
    }
    static void reset() { offered = delivered = refused = replaced = failed = unreported = 0; last = lastFail = Outcome{}; haveLast = headFailed = false; nowV = 0; Tab::reset(); }

  private:
    static void bump() { if (unreported != 0xFFFF) ++unreported; }

    // a class without a store: the sample goes to the sink now, through the stack (status only)
    static void direct(const Rec& r) {
      ++offered;
      Outcome res = Outcome::Idle();
      Tab::serve(0, Cause::Fresh, [&]() -> Outcome { res = Sink::deliver(r, nowV); return res; });
      haveLast = true; last = res;
      if (res.isOk()) ++delivered; else if (res.failed()) ++failed;
    }

    // the head record, through the stack; taken off the store when the sink says Ok
    static Outcome attempt(Cause c) {
      if constexpr (!kQueued) { (void)c; return Outcome::Idle(); }
      else {
        if (ctl().empty()) return Outcome::Idle();
        const Rec r = ctl().front();
        Outcome res = Outcome::Idle();
        Tab::serve(0, c, [&]() -> Outcome {
          res = Sink::deliver(r, nowV);
          if (res.isOk()) { ctl().pop(); ++delivered; }
          return res;
        });
        headFailed = res.failed(); if (headFailed) lastFail = res;
        return res;
      }
    }

    // the head record is gone for good (a kind that is not retried, or Retry's budget spent): counted, and the next one goes on
    static void dropHead() {
      if constexpr (kQueued) { if (!ctl().empty()) ctl().pop(); ++failed; last = lastFail; haveLast = true; headFailed = false; }
    }

    // offer the head while the sink takes them, at most Burst per tick; stop at a busy sink or at a failure Retry holds
    static void drain() {
      if constexpr (kQueued) {
        for (uint8_t i = 0; i < Cfg::Burst; ++i) {
          if (ctl().empty() || holding()) return;
          const Outcome o = attempt(Cause::Fresh);
          if (o.isOk()) continue;
          if (!o.failed()) return;                                               // Blocked or Pending: the sink is busy, the record stays
          if (holding()) return;                                                 // Retry has it
          ctl().pop(); ++failed; last = o; haveLast = true; headFailed = false;   // a kind this edge does not retry: reported, the record goes
        }
      }
    }
  };

  // ---- the consumer the fan-out sees ---------------------------------------------------------------------------------------------------
  template<typename Cfg>
  struct Delivered {
    using Accepts = typename Cfg::Accepts;
    using Edge    = DeliveryEdge<Cfg>;
    static void tick(uint32_t now) { Edge::tick(now); }
    template<typename Cap> struct Body {
      template<typename T> struct Part : T {
        using T::T;
        void on(const discover::Sample<Cap>& s) { Edge::offer(Rec{Cap::id, s.row, int32_t(s.value)}); }
      };
    };
  };

  // the consumers of a list that have a static tick(now): folded statically, nothing when none has one
  template<typename C, typename = void> struct ConsumerTicks : std::false_type {};
  template<typename C> struct ConsumerTicks<C, std::void_t<decltype(C::tick(uint32_t{}))>> : std::true_type {};
  template<typename L> struct DeliveryTicks;
  template<typename... C> struct DeliveryTicks<hapi::Chain<C...>> {
    static void run([[maybe_unused]] uint32_t now) { (one<C>(now), ...); }
  private:
    template<typename X> static void one([[maybe_unused]] uint32_t now) { if constexpr (ConsumerTicks<X>::value) X::tick(now); }
  };

}
