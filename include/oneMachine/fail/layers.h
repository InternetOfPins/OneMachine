// A failure controller for one device edge: layers over an edge terminal, one object per registry row.
//   Controller<Env, Layers...> = APIOf<EdgeTerm<Env>, Outer, Layers...>   (outermost layer first)
// Inner face: run() returns Outcome down the stack. Outer face: serve() is void, status() cannot fail.
// Return path: a layer that answers _f(id) is a CausePart; the answer travels up the same layers the operation went down, and _serve(id)
// reads it at the top. Reply stores it (sticky or read-once), Within and Overall turn silence into Timeout.
// Layers report through Status (noteOk/noteFail/...); rules() encode R-1, R-3, R-4 and the tick audit.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include "outcome.h"
#include "deadline.h"

namespace fail {

  // ---- tick: a ticking layer is a TickPart, which always forwards ----------------------------------
  template<typename T, typename = void> struct has_tick : std::false_type {};
  template<typename T> struct has_tick<T, std::void_t<decltype(std::declval<T&>().tick(uint32_t{}))>> : std::true_type {};

  struct TickPartTag {};

  // Body::Part<T> defines onTick(now); TickPart<Body> supplies tick(now) = Base::tick(now), then onTick(now).
  template<typename Body>
  struct TickPart : TickPartTag, Body {
    template<typename T>
    struct Part : Body::template Part<T> {
      using Self = typename Body::template Part<T>;
      static_assert(!has_tick<typename Body::template Part<hapi::Nil>>::value,
                    "a TickPart body defines onTick(now), not tick(now): TickPart's tick forwards to Base::tick and then calls onTick");
      void tick(uint32_t now) {
        if constexpr (has_tick<T>::value) T::tick(now);
        Self::onTick(now);
      }
    };
  };

  // a layer with its own tick(now) that is not a TickPart would hide the tick below it
  template<typename O, bool = hapi::HasPart<O>::value> struct BadTick : std::false_type {};
  template<typename O> struct BadTick<O, true>
    : std::bool_constant<has_tick<typename O::template Part<hapi::Nil>>::value && !std::is_base_of<TickPartTag, O>::value> {};

  struct BadTickLayer {
    template<typename O> using Check = typename hapi::Traverse<BadTickLayer, O>::Beta;
    template<typename O> using Apply = BadTick<O>;
    template<typename... OO> using ApplyPack = hapi::Chain<OO...>;
  };

  // ---- return path: a layer that answers _f is a CausePart, which always asks the layer below first --------------
  template<typename T, typename = void> struct has_cause_fn : std::false_type {};
  template<typename T> struct has_cause_fn<T, std::void_t<decltype(std::declval<T&>()._f(Id{}))>> : std::true_type {};

  struct CausePartTag {};

  // Body::Part<T> defines onCause(Outcome, Id) -> Outcome; CausePart<Body> supplies _f(id) = onCause(Base::_f(id), id).
  // A layer may pass the outcome, translate it, record it, or absorb it; it cannot skip the layer below.
  template<typename Body>
  struct CausePart : CausePartTag, Body {
    template<typename T>
    struct Part : Body::template Part<T> {
      using Self = typename Body::template Part<T>;
      static_assert(!has_cause_fn<typename Body::template Part<hapi::Nil>>::value,
                    "a CausePart body defines onCause(outcome, id), not _f(id): CausePart's _f asks the layer below first and then calls onCause");
      Outcome _f(Id id) { return Self::onCause(T::_f(id), id); }
    };
  };

  // a layer with its own _f that is not a CausePart would hide the return path below it
  template<typename O, bool = hapi::HasPart<O>::value> struct BadCause : std::false_type {};
  template<typename O> struct BadCause<O, true>
    : std::bool_constant<has_cause_fn<typename O::template Part<hapi::Nil>>::value && !std::is_base_of<CausePartTag, O>::value> {};

  struct BadCauseLayer {
    template<typename O> using Check = typename hapi::Traverse<BadCauseLayer, O>::Beta;
    template<typename O> using Apply = BadCause<O>;
    template<typename... OO> using ApplyPack = hapi::Chain<OO...>;
  };

  // ---- tags the rules match on ---------------------------------------------------------------------
  struct StatusTag {};  struct GateTag {};  struct RetryTag {};  struct LosslessTag {};  struct LossyTag {};
  struct ReplyTag {};   struct WithinTag {};  struct OverallTag {};  struct CoalesceTag {};  struct NoTag {};
  struct RetryCauseTag {};  struct KeepaliveTag {};  struct BackoffTag {};
  struct QueueTag {};  struct LatestTag {};  struct BufferTag {};     // a record store: any / one slot that overwrites / N slots that refuse the newest

  // hooks a driver may declare for "the bus below you came back" (see DevEdge::busReturned): recheck(row), or reinitOnBusReturn
  template<typename D, typename = void> struct HasRecheck : std::false_type {};
  template<typename D> struct HasRecheck<D, std::void_t<decltype(D::recheck(RowId{}))>> : std::true_type {};
  template<typename D, typename = void> struct ReinitOnBusReturn : std::false_type {};
  template<typename D> struct ReinitOnBusReturn<D, std::void_t<decltype(D::reinitOnBusReturn)>> : std::bool_constant<D::reinitOnBusReturn> {};
  template<typename D> struct WantsBusReturn : std::bool_constant<HasRecheck<D>::value || ReinitOnBusReturn<D>::value> {};

  template<typename Tag, typename After> constexpr bool below = hapi::query<hapi::TagIs<Tag>, After>;

  // ---- Status: counters and the last failure; writes the row's Alive/Stale/Gone ----------------------
  // Eager (F1): every failure writes Stale, every success writes Alive. Lazy (F5): a failure only counts; the row changes
  // only when a layer says so (noteStale / noteAlive / noteGone), and a success writes Alive only after this edge wrote Stale.
  template<bool Eager>
  struct StatusT : StatusTag {
    template<typename T> struct Part : T {
      uint8_t retries = 0, recovers = 0, drops = 0, fails = 0, lastKind = 0, lastDetail = 0;
      bool    okSeen = false, gone = false, stale = false;

      [[nodiscard]] FailStatus status() const noexcept { return {retries, recovers, drops, fails, lastKind, lastDetail}; }
      [[nodiscard]] bool ownStale() const noexcept { return stale; }     // this edge's own fault, apart from the registry's status

      // the row is written when this edge's state changes, not on every operation
      void noteOk() {
        const bool down = stale || (Eager && gone);
        okSeen = true; gone = false;
        if (down) { stale = false; this->rowState(RowState::Alive); }
      }
      void noteFail(Kind k, uint8_t d) {
        sat(fails); lastKind = uint8_t(k); lastDetail = d;
        if constexpr (Eager) {
          if (gone) this->rowState(RowState::Gone);
          else if (!stale) { stale = true; this->rowState(RowState::Stale); }
        }
      }
      void noteRetry()   { sat(retries); }
      void noteRecover() { sat(recovers); }
      void noteDrop()    { sat(drops); }
      void noteStale()   { stale = true; this->rowState(RowState::Stale); }
      void noteAlive()   { stale = false; gone = false; this->rowState(RowState::Alive); }
      void noteGone()    { gone = true; stale = false; this->rowState(RowState::Gone); }
      // retry budget spent: the default is the end of the row; a layer below (Reprobe) can hook it
      void exhausted()   { noteGone(); }
      bool takeOk()      { bool r = okSeen; okSeen = false; return r; }
    private:
      static void sat(uint8_t& c) { if (c != 0xFF) ++c; }
    };
  };
  using Status     = StatusT<true>;
  using LazyStatus = StatusT<false>;

  // ---- Detect, error form: an inner failure becomes an event; a success is noted --------------------
  struct DetectError {
    template<typename Before, typename After> static constexpr bool rules() {
      static_assert(below<StatusTag, After>, "DetectError reports into Status: place Status below it");
      return true;
    }
    template<typename T> struct Part : T {
      template<typename F> Outcome run(RowId row, Cause c, F&& op) {
        Outcome o = T::run(row, c, op);
        if (o.failed()) this->noteFail(o.kind(), o.detail);
        else if (o.isOk()) this->noteOk();
        return o;
      }
    };
  };

  // ---- Detect, deadline form: no success within Ms of the last one is a Timeout event ---------------
  template<uint32_t Ms>
  struct DetectDeadline {
    static_assert(Ms > 0 && Ms < 0x80000000u, "DetectDeadline: Ms must be in (0, 2^31)");
    template<typename Before, typename After> static constexpr bool rules() {
      static_assert(below<StatusTag, After>, "DetectDeadline reports into Status: place Status below it");
      return true;
    }
    template<typename T> struct Part : T {
      Deadline stale;
      void onTick(uint32_t now) {
        if (this->takeOk()) stale.arm(now, Ms);
        else if (stale.due(now)) { this->noteFail(Kind::Timeout, 0xDD); stale.disarm(); }
      }
    };
  };

  // ---- Gate, fixed interval ---------------------------------------------------------------------
  template<uint32_t Ms>
  struct Gate : GateTag {
    static_assert(Ms > 0 && Ms < 0x80000000u, "Gate: Ms must be in (0, 2^31)");
    template<typename T> struct Part : T {
      Deadline next;
      void gateStart(uint32_t now)      { next.arm(now, Ms); }
      void gateStop()                   { next.disarm(); }
      bool gateArmed() const            { return next.armed; }
      bool gateDue(uint32_t now) const  { return next.due(now); }
    };
  };

  // ---- Store: the one slot a retrying edge needs to hold its stored operation -------------------------
  // A read operation carries no payload, so "the operation" is the row's pending re-issue.
  struct RejectNewest { static constexpr bool lossless = true;  };
  struct Overwrite    { static constexpr bool lossless = false; };
  // For an idempotent operation (a read): a fresh one arriving while a retry is held is the held one; nothing is refused or counted.
  struct Coalesce     { static constexpr bool lossless = true;  static constexpr bool coalesces = true; };

  template<typename P, typename = void> struct PolicyCoalesces : std::false_type {};
  template<typename P> struct PolicyCoalesces<P, std::void_t<decltype(P::coalesces)>> : std::bool_constant<P::coalesces> {};

  template<typename Policy>
  struct HoldOp : std::conditional_t<Policy::lossless, LosslessTag, LossyTag>,
                  std::conditional_t<PolicyCoalesces<Policy>::value, CoalesceTag, NoTag> {
    template<typename Before, typename After> static constexpr bool rules() {
      static_assert(below<StatusTag, After>, "HoldOp counts what it discards into Status: place Status below it");
      return true;
    }
    template<typename T> struct Part : T {
      bool held = false;
      static constexpr bool coalesces = PolicyCoalesces<Policy>::value;
      bool holding() const { return held; }
      void hold()          { held = true; }
      void release()       { held = false; }
      // a fresh operation while one is held: refused (RejectNewest) or replaces it, counted (Overwrite)
      bool admitFresh() {
        if (!held) return true;
        if constexpr (Policy::lossless) return false;
        this->noteDrop();
        return true;
      }
    };
  };

  // ---- Act / Retry: re-issue the stored operation, gated, at most Max times ------------------------------
  // The edge declares which kinds it retries; exhaustion is counted and hands over to the layer below (default: the row is Gone).
  // Max = 0 retries without end (a bus stays Stale on its back-off). With a return path (Env::returnPath) a held operation
  // answers Pending upward; without one the failure passes as it is.
  // OnCause: the operation can be accepted (Pending) and fail later, on the return path, and a state that was Ok can turn bad
  // (a link that dies). The layer then asks the layers below how the operation is doing, on every tick and when it is asked itself,
  // and treats a retryable failure it finds there as one seen on run(). Use as TickPart<CausePart<Retry<Max, true>>>.
  template<int> struct NoState {};                       // an empty stand-in for state a layer does not keep (one type per layer: a base cannot repeat)
  struct RetryCauseState { bool awaiting = false, finSet = false, needArm = false; Outcome fin; };   // awaiting: an accepted attempt; fin: the failure being retried, final once finSet; needArm: wait again from the next tick
  template<uint8_t Max, bool OnCause = false>
  struct Retry : RetryTag, std::conditional_t<OnCause, RetryCauseTag, NoTag> {
    template<typename Before, typename After> static constexpr bool rules() {
      static_assert(below<StatusTag, After>,   "Retry reports exhaustion into Status: place Status below it");
      static_assert(below<GateTag, After>,     "Retry is gated: place a Gate below it");
      static_assert(below<LosslessTag, After>, "R-3: Retry needs a Store below it that cannot overwrite the held operation");
      static_assert(Max != 0 || below<BackoffTag, After>, "a retry without end needs a Backoff gate: retrying on a fixed interval hammers a peer that is down");
      return true;
    }
    template<typename T> struct Part : T, std::conditional_t<OnCause, RetryCauseState, NoState<1>> {
      uint8_t attempts = 0;

      template<typename F> Outcome run(RowId row, Cause c, F&& op) {
        if constexpr (T::coalesces) { if (c == Cause::Fresh && this->holding()) return Outcome::Pending(); }
        else if (c == Cause::Fresh && !this->admitFresh()) { this->noteDrop(); return Outcome::Fail(Kind::Overflow); }
        Outcome o = T::run(row, c, op);
        if constexpr (OnCause) {
          if (c == Cause::Fresh) this->finSet = false;
          this->awaiting = o.isPending();
          if (o.isBlocked() && this->holding()) return heldAnswer();                 // waiting for what is below to come back: still in flight
        }
        if (o.isOk()) { if (this->holding()) this->release(); attempts = 0; this->gateStop(); return o; }
        if (!o.failed()) return o;
        if constexpr (Max != 0) {
          if (c == Cause::Fresh) {
            if (T::retryable(o.kind())) { this->hold(); attempts = 0; this->gateStop(); if constexpr (OnCause) this->fin = o; if constexpr (T::returnPath) return Outcome::Pending(); }
          } else if (++attempts >= Max || !T::retryable(o.kind())) {
            this->release(); this->gateStop(); this->noteDrop(); this->exhausted();
            if constexpr (OnCause) { this->fin = o; this->finSet = true; }
          } else if constexpr (T::returnPath) return Outcome::Pending();
        } else {                                       // without end: only a kind this edge does not retry gives up
          if (c == Cause::Fresh) {
            if (T::retryable(o.kind())) { this->hold(); attempts = 0; this->gateStop(); if constexpr (OnCause) this->fin = o; }
          } else if (!T::retryable(o.kind())) {
            this->release(); this->gateStop(); this->noteDrop(); this->exhausted();
            if constexpr (OnCause) { this->fin = o; this->finSet = true; }
          }
        }
        return o;
      }

      // an outer deadline gave up on the loop: same end as exhaustion
      void abortLoop() {
        if (!this->holding()) return;
        attempts = 0; this->release(); this->gateStop(); this->noteDrop(); this->exhausted();
      }

      // first tick after a failure arms the gate; each re-issue re-arms it
      void onTick(uint32_t now) {
        if constexpr (OnCause) {
          if (this->awaiting || !this->holding()) { (void)observe(T::_f(0)); if (this->awaiting || !this->holding()) return; }
          if (this->needArm) { this->needArm = false; this->gateStart(now); return; }    // an attempt failed: the next one waits from here
        } else {
          if (!this->holding()) return;
        }
        if (!this->gateArmed()) { this->gateStart(now); return; }
        if (!this->gateDue(now)) return;
        this->noteRetry();
        T::reissue(this->row);
        if constexpr (OnCause) { if (this->holding() && !this->awaiting) this->gateStart(now); }   // an accepted attempt is waited for, not timed
        else { if (this->holding()) this->gateStart(now); }
      }

      // the return path: what an outer layer is told
      Outcome onCause(Outcome below, Id) { static_assert(OnCause, "onCause is for Retry<Max, true>"); return observe(below); }

    private:
      Outcome heldAnswer() const { if constexpr (Max != 0) return Outcome::Pending(); else return this->fin; }

      // an attempt (or the state it was in) failed in a way this edge retries
      Outcome failedAttempt(Outcome f) {
        this->fin = f;
        if (!this->holding()) { this->hold(); attempts = 0; this->gateStop(); }          // a new episode starts from the shortest interval
        else if (Max != 0 && ++attempts >= Max) {
          this->release(); this->gateStop(); this->noteDrop(); this->exhausted();
          this->finSet = true;
          return f;
        } else this->needArm = true;                                                       // another failed attempt: the next waits, on a longer interval
        return heldAnswer();
      }

      Outcome observe(Outcome below) {
        if (this->finSet) return this->fin;
        if (this->awaiting) {
          if (below.isPending() || below.isIdle()) return Outcome::Pending();
          this->awaiting = false;
          if (below.isOk()) { if (this->holding()) this->release(); attempts = 0; this->gateStop(); return below; }
          if (below.failed() && T::retryable(below.kind())) return failedAttempt(below);
          return below;                                                       // Blocked, or a kind this edge does not retry
        }
        if (this->holding()) return heldAnswer();
        if (below.failed() && T::retryable(below.kind())) return failedAttempt(below);   // nothing awaited, and the state has turned bad
        return below;
      }
    };
  };

  // ---- Gate that backs off: the interval doubles on each arm (capped) and starts over when the gate is stopped ----
  // Retry stops the gate when an operation succeeds, gives up, or a new failure begins a new episode.
  template<uint32_t MinMs, uint32_t MaxMs>
  struct Backoff : GateTag, BackoffTag {
    static_assert(MinMs > 0 && MinMs <= MaxMs && MaxMs < 0x8000, "Backoff: 0 < Min <= Max < 32768 ms");
    template<typename T> struct Part : T {
      Deadline next;
      uint16_t cur = 0;                                     // the interval the next arm will use; 0 = Min (keeps the state all-zero)
      void gateStart(uint32_t now) {
        const uint16_t i = cur ? cur : uint16_t(MinMs);
        next.arm(now, i);
        cur = i > MaxMs / 2 ? uint16_t(MaxMs) : uint16_t(i << 1);
      }
      void gateStop()                   { next.disarm(); cur = 0; }
      bool gateArmed() const            { return next.armed; }
      bool gateDue(uint32_t now) const  { return next.due(now); }
    };
  };

  // ---- Act / Recover: reset the bus inside the failing operation; Timeout and Fault only (R-1), plus the kinds the edge declares (Env::recoverMask) ----
  struct Recover {
    static constexpr bool recoverable(Kind k) { return k == Kind::Timeout || k == Kind::Fault; }   // fixed on every edge
    template<typename Before, typename After> static constexpr bool rules() {
      static_assert(!below<RetryTag, After>, "R-4: Recover runs inside the failing operation, below Retry; Retry below Recover would re-issue without recovering");
      static_assert(below<StatusTag, After>, "Recover reports into Status: place Status below it");
      return true;
    }
    template<typename T> struct Part : T {
      template<typename F> Outcome run(RowId row, Cause c, F&& op) {
        Outcome o = T::run(row, c, op);
        if (o.failed() && (recoverable(o.kind()) || T::recoversKind(o.kind()))) { this->noteRecover(); T::busReset(); }
        return o;
      }
    };
  };

  // ---- Act / Reprobe: a row whose retries are spent goes Stale and is re-probed, gated, until it answers or M probes fail ----
  // Place it below Retry: it takes over Retry's exhaustion. Env::reprobe(row) is read-only presence and identity; a Blocked
  // answer (the bus below is down) is not a miss.
  template<uint32_t Ms, uint8_t M>
  struct Reprobe {
    static_assert(Ms > 0 && Ms < 0x80000000u, "Reprobe: Ms must be in (0, 2^31)");
    static_assert(M > 0, "Reprobe: M must be at least 1");
    template<typename Before, typename After> static constexpr bool rules() {
      static_assert(below<StatusTag, After>, "Reprobe reports into Status: place Status below it");
      static_assert(below<RetryTag, Before>, "Reprobe takes over Retry's exhaustion: place it below Retry");
      return true;
    }
    template<typename T> struct Part : T {
      bool     probing = false;
      uint8_t  misses  = 0;
      Deadline next;

      void exhausted() { probing = true; misses = 0; next.disarm(); this->noteStale(); }
      bool reprobing() const { return probing; }

      void onTick(uint32_t now) {
        if (!probing) return;
        if (!next.armed) { next.arm(now, Ms); return; }
        if (!next.due(now)) return;
        const Outcome o = T::reprobe(this->row);
        if (o.isOk())          { probing = false; next.disarm(); this->noteAlive(); }
        else if (o.failed())   { if (++misses >= M) { probing = false; next.disarm(); this->noteGone(); } else next.arm(now, Ms); }
        else                   { next.arm(now, Ms); }
      }
    };
  };

  // the bit for an id: a constant for a single operation (a variable shift is a loop on an 8-bit core)
  template<uint8_t N> constexpr uint8_t idBit(Id id) { if constexpr (N == 1) { (void)id; return 1; } else return uint8_t(1u << id); }

  // ---- Overall: one deadline around the whole retry loop (R-5: timeout around retries) --------------------
  // The per-attempt form is Retry with its Gate: total time is the attempts times the interval. This one gives up Ms after the tick
  // that saw the loop begin, whatever the retries have left. Place it above Retry; it reads Retry's Pending, so the edge needs returnPath.
  // Use as TickPart<CausePart<Overall<Ms>>>.
  template<uint32_t Ms>
  struct Overall : OverallTag {
    static_assert(Ms > 0 && Ms < 0x80000000u, "Overall: Ms must be in (0, 2^31)");
    template<typename Before, typename After> static constexpr bool rules() {
      static_assert(below<RetryTag, After>,  "an overall deadline bounds the retry loop: place it above Retry (below Retry it would bound one attempt)");
      static_assert(below<StatusTag, After>, "Overall reports into Status: place Status below it");
      return true;
    }
    template<typename T> struct Part : T {
      bool     looping = false, aborted = false;
      Deadline limit;

      template<typename F> Outcome run(RowId row, Cause c, F&& op) {
        const Outcome o = T::run(row, c, op);
        if (o.isPending()) looping = true; else { looping = false; limit.disarm(); }
        return o;
      }
      void onTick(uint32_t now) {
        if (!looping) return;
        if (!limit.armed) { limit.arm(now, Ms); return; }
        if (!limit.due(now)) return;
        looping = false; limit.disarm(); aborted = true;
        this->noteFail(Kind::Timeout, 0xEE);
        this->abortLoop();
      }
      Outcome onCause(Outcome o, Id) {
        if (!aborted) return o;
        aborted = false;
        return Outcome::Fail(Kind::Timeout, 0xEE);
      }
    };
  };

  // ---- Within: an operation that can stay Pending has a deadline (R-2) -------------------------------------
  // The answer that never arrives becomes Fail(Timeout) on the return path, and is counted. N operations may be in flight (ids 0..N-1).
  // The deadline starts on the first tick after the operation is accepted. Use as TickPart<CausePart<Within<Ms, N>>>.
  template<uint32_t Ms, uint8_t N = 1>
  struct Within : WithinTag {
    static_assert(Ms > 0 && Ms < 0x80000000u && N >= 1 && N <= 8, "Within: Ms in (0, 2^31), 1 to 8 operations");
    template<typename Before, typename After> static constexpr bool rules() {
      static_assert(below<StatusTag, After>, "Within reports Timeout into Status: place Status below it");
      return true;
    }
    template<typename T> struct Part : T {
      uint8_t  pending = 0, expired = 0;                     // bit per id
      Deadline limit[N];

      template<typename F> Outcome run(RowId row, Cause c, F&& op) {
        const Outcome o = T::run(row, c, op);
        if (o.isPending() && o.detail < N) { pending = uint8_t(pending | idBit<N>(o.detail)); expired = uint8_t(expired & ~idBit<N>(o.detail)); limit[o.detail].disarm(); }
        return o;
      }
      void onTick(uint32_t now) {
        for (uint8_t i = 0; i < N; ++i) {
          if (!(pending & idBit<N>(i))) continue;
          if (!limit[i].armed) limit[i].arm(now, Ms);
          else if (limit[i].due(now)) expired = uint8_t(expired | idBit<N>(i));
        }
      }
      Outcome onCause(Outcome o, Id id) {
        if (id >= N) return o;
        const uint8_t b = idBit<N>(id);
        if (o.isPending() && (expired & b)) {
          pending = uint8_t(pending & ~b); expired = uint8_t(expired & ~b); limit[id].disarm();
          this->noteFail(Kind::Timeout, id);
          return Outcome::Fail(Kind::Timeout, id);
        }
        if (!o.isPending()) { pending = uint8_t(pending & ~b); expired = uint8_t(expired & ~b); limit[id].disarm(); }
        return o;
      }
    };
  };

  // ---- Reply: the outcome an operation ended with, as the app asks for it -------------------------------------
  // Sticky (ReadOnce = false): _f(id) answers the same until the next f. Read-once: it answers once, then Idle. The outermost layer
  // stores what the layers below made of each pass (Retry's Pending, the final Ok or failure); an outcome that arrives later from
  // below (an async operation) is stored when it is asked for. N ids; an id that is not one of them is Fail(Refused).
  // ReportLoss: a finished outcome that was never read and is replaced by the next f is counted, and the next _f answers
  // Fail(Overflow, how many) once before anything else (a single operation; needs Status below it).
  // The edge must declare returnPath. Use as CausePart<Reply<ReadOnce, N, ReportLoss>>.
  struct ReplyLossState { uint8_t lost = 0; };
  template<bool ReadOnce, uint8_t N = 1, bool ReportLoss = false>
  struct Reply : ReplyTag {
    static_assert(N >= 1 && N <= 8, "Reply: 1 to 8 operations");
    static_assert(!ReportLoss || N == 1, "Reply: reporting a lost outcome is for a single operation");
    template<typename Before, typename After> static constexpr bool rules() {
      static_assert(!ReportLoss || below<StatusTag, After>, "Reply counts a lost outcome into Status: place Status below it");
      return true;
    }
    template<typename T> struct Part : T, std::conditional_t<ReportLoss, ReplyLossState, NoState<2>> {
      Outcome last[N];
      uint8_t have = 0, consumed = 0;                         // bit per id

      template<typename F> Outcome run(RowId row, Cause c, F&& op) {
        if constexpr (ReportLoss) {
          if (c == Cause::Fresh && (have & 1) && !(consumed & 1)) {        // a re-issue is the same operation, not a new one
            Outcome prev = last[0];
            if (!prev.isDone()) prev = T::_f(0);              // it may have finished below without being asked
            if (prev.isDone()) { if (this->lost != 0xFF) ++this->lost; this->noteDrop(); }
          }
        }
        const Outcome o = T::run(row, c, op);
        const Id i = o.isPending() ? o.detail : Id(0);
        if (i < N) { last[i] = o; have = uint8_t(have | idBit<N>(i)); consumed = uint8_t(consumed & ~idBit<N>(i)); }
        return o;
      }
      Outcome onCause(Outcome below, Id id) {
        if constexpr (ReportLoss) { if (this->lost) { const uint8_t n = this->lost; this->lost = 0; return Outcome::Fail(Kind::Overflow, n); } }
        if (id >= N) return Outcome::Fail(Kind::Refused, id);
        const uint8_t b = idBit<N>(id);
        if (consumed & b) return Outcome::Idle();
        const Outcome mine = (have & b) ? last[id] : Outcome::Idle();
        // nothing arrives from below unless the terminal has operations in flight or a layer below answers _f (Within, Overall)
        if constexpr (!T::hasResult && std::is_same<decltype(&T::_f), typename T::AnswerFn>::value) {
          if constexpr (ReadOnce) { if (mine.isDone()) consumed = uint8_t(consumed | b); }
          return mine;
        }
        if (!mine.isDone() && !below.isDone()) return below.isIdle() ? mine : below;    // in flight, or nothing to say
        const Outcome r = mine.isDone() ? mine : below;
        last[id] = r; have = uint8_t(have | b);
        if constexpr (ReadOnce) consumed = uint8_t(consumed | b);
        return r;
      }
    };
  };

  // ---- Gate whose trigger is the link coming back, alone or with a timer --------------------------------------
  // The edge counts its (re)connections (Env::epoch()). The gate is due when the count has changed since it was armed, or when
  // Ms have passed (Ms = 0: only the reconnect). MQTT 3.1.1 resends an unacknowledged PUBLISH on reconnect; a timer resends it earlier.
  template<uint32_t Ms>
  struct ReconnectGate : GateTag {
    static_assert(Ms < 0x80000000u, "ReconnectGate: Ms must be below 2^31");
    template<typename T> struct Part : T {
      Deadline next;
      uint8_t  seen = 0;
      bool     on = false;
      void gateStart(uint32_t now)      { if constexpr (Ms != 0) next.arm(now, Ms); seen = T::Environment::epoch(); on = true; }
      void gateStop()                   { next.disarm(); on = false; }
      bool gateArmed() const            { return on; }
      bool gateDue(uint32_t now) const  { return on && (T::Environment::epoch() != seen || next.due(now)); }
    };
  };

  // ---- Keepalive: a Detect deadline that any inbound packet resets, and a Gate that pings when the link has been quiet ----
  // While the link is up: QuietMs without a packet -> the edge sends a ping; PongMs more without one -> the link is dead, and
  // the return path says Fail(Timeout). A new attempt to connect starts the watch over. The edge provides linkUp(), rxCount(),
  // sendPing() (Env). Place it below Retry. Use as TickPart<CausePart<Keepalive<QuietMs, PongMs>>>.
  template<uint32_t QuietMs, uint32_t PongMs>
  struct Keepalive : KeepaliveTag {
    static_assert(QuietMs > 0 && PongMs > 0 && QuietMs < 0x80000000u && PongMs < 0x80000000u, "Keepalive: intervals in (0, 2^31)");
    template<typename Before, typename After> static constexpr bool rules() {
      static_assert(below<StatusTag, After>, "Keepalive reports the dead link into Status: place Status below it");
      return true;
    }
    template<typename T> struct Part : T {
      Deadline quiet, pong;
      uint8_t  seen = 0;
      bool     pinged = false, dead = false;

      template<typename F> Outcome run(RowId row, Cause c, F&& op) {
        const Outcome o = T::run(row, c, op);
        quiet.disarm(); pong.disarm(); pinged = false; dead = false;
        return o;
      }
      void onTick(uint32_t now) {
        using E = typename T::Environment;
        if (dead) return;
        if (!E::linkUp()) { quiet.disarm(); pong.disarm(); pinged = false; return; }
        const uint8_t rx = E::rxCount();
        if (rx != seen) { seen = rx; pinged = false; pong.disarm(); quiet.arm(now, QuietMs); return; }
        if (!quiet.armed) { quiet.arm(now, QuietMs); return; }
        if (!pinged) { if (quiet.due(now)) { E::sendPing(); pinged = true; pong.arm(now, PongMs); } return; }
        if (pong.due(now)) { dead = true; this->noteFail(Kind::Timeout, 0xEA); }
      }
      Outcome onCause(Outcome o, Id) { return dead ? Outcome::Fail(Kind::Timeout, 0xEA) : o; }
    };
  };

  // ---- outer face: always ok ------------------------------------------------------------------------
  struct Outer {
    template<typename T> struct Part : T {
      template<typename F> void serve(RowId row, Cause c, F&& op) { this->bind(row); (void)T::run(row, c, op); }
      // the return path: what the operation came to, after every layer has had its say
      Outcome _serve(RowId row, Id id = 0) { this->bind(row); return T::_f(id); }
    };
  };

  // ---- the terminal: what the edge provides. Env: retryMask, busReset(), reissue(row), reprobe(row), setRowState(row, s) --
  // A stack with a Status layer writes row status. That is allowed only where World::setStatus is the writer (Env::lifecycle,
  // an app with lifecycle) or in a test of an app without one that says so (Env::rawStatus).
  template<typename E, typename = void> struct EnvLifecycle : std::false_type {};
  template<typename E> struct EnvLifecycle<E, std::void_t<decltype(E::lifecycle)>> : std::bool_constant<E::lifecycle> {};
  template<typename E, typename = void> struct EnvRawStatus : std::false_type {};
  template<typename E> struct EnvRawStatus<E, std::void_t<decltype(E::rawStatus)>> : std::bool_constant<E::rawStatus> {};

  // the edge's declarations: returnPath (its layers answer _f), idempotent (its operation may be coalesced), async (it can answer Pending),
  // result(row, id) (what an operation in flight has come to)
  template<typename E, typename = void> struct EnvReturnPath : std::false_type {};
  template<typename E> struct EnvReturnPath<E, std::void_t<decltype(E::returnPath)>> : std::bool_constant<E::returnPath> {};
  template<typename E, typename = void> struct EnvIdempotent : std::false_type {};
  template<typename E> struct EnvIdempotent<E, std::void_t<decltype(E::idempotent)>> : std::bool_constant<E::idempotent> {};
  template<typename E, typename = void> struct EnvAsync : std::false_type {};
  template<typename E> struct EnvAsync<E, std::void_t<decltype(E::async)>> : std::bool_constant<E::async> {};
  // a class of edge that does not accept a Store that overwrites (a network transport: a message is delivered or reported, not replaced)
  template<typename E, typename = void> struct EnvAllowsOverwrite : std::true_type {};
  template<typename E> struct EnvAllowsOverwrite<E, std::void_t<decltype(E::overwrite)>> : std::bool_constant<E::overwrite> {};
  // the class of consumer a delivery edge serves (F3): what it accepts is fixed by the class, not by the composition
  enum class Consumer : uint8_t { None = 0, Display, Storage, Network, FireForget, Direct };
  template<typename E, typename = void> struct EnvClass { static constexpr uint8_t value = 0; };
  template<typename E> struct EnvClass<E, std::void_t<decltype(E::consumerClass)>> { static constexpr uint8_t value = uint8_t(E::consumerClass); };
  // Recover is per edge: the kinds an edge asks it to act on besides Timeout and Fault (Env::recoverMask), and what the recovery is
  // (Env::recover(row) when the edge has one that needs its row: a device re-initialised; otherwise Env::busReset())
  template<typename E, typename = void> struct EnvRecoverMask { static constexpr uint8_t value = 0; };
  template<typename E> struct EnvRecoverMask<E, std::void_t<decltype(E::recoverMask)>> { static constexpr uint8_t value = E::recoverMask; };
  template<typename E, typename = void> struct EnvRecoverRow : std::false_type {};
  template<typename E> struct EnvRecoverRow<E, std::void_t<decltype(E::recover(RowId{}))>> : std::true_type {};
  template<typename E, typename = void> struct EnvResult : std::false_type {};
  template<typename E> struct EnvResult<E, std::void_t<decltype(E::result(RowId{}, Id{}))>> : std::true_type {};

  template<typename Env>
  struct EdgeTerm {
    RowId row = 0;                               // set by bind(); zero keeps the per-row tables in .bss
    static constexpr bool returnPath = EnvReturnPath<Env>::value;
    static constexpr bool hasResult  = EnvResult<Env>::value;          // operations can be in flight: the terminal answers _f
    using Environment = Env;                                           // the edge's own declarations, for layers that need them (epoch, link state)
    using AnswerFn = Outcome (EdgeTerm::*)(Id);                        // the terminal's _f, to tell whether a layer below replaced it
    void bind(RowId r) { row = r; }
    template<typename F> static Outcome run(RowId, Cause, F&& op) { return op(); }
    static constexpr bool retryable(Kind k) { return (Env::retryMask & bit(k)) != 0; }
    void busReset()                 { if constexpr (EnvRecoverRow<Env>::value) Env::recover(row); else Env::busReset(); }
    static constexpr bool recoversKind(Kind k) { return (EnvRecoverMask<Env>::value & bit(k)) != 0; }
    static void reissue(RowId r)    { Env::reissue(r); }
    static Outcome reprobe(RowId r) { return Env::reprobe(r); }
    void rowState(RowState s)       { Env::setRowState(row, uint8_t(s)); }
    // the bottom of the return path: an edge with operations in flight says how they are doing, any other has nothing to add
    Outcome _f(Id id) {
      if constexpr (EnvResult<Env>::value) return Env::result(row, id);
      else { (void)id; return Outcome::Idle(); }
    }
    template<typename Before, typename After> static constexpr bool rules() {
      static_assert(!hapi::query<BadTickLayer, After>,
                    "a layer defines tick(now) without being a TickPart: it would hide the tick below it; use TickPart<Body> with onTick(now)");
      static_assert(!hapi::query<BadCauseLayer, After>,
                    "a layer defines _f without being a CausePart: it would hide the return path below it; use CausePart<Body> with onCause");
      static_assert((!below<ReplyTag, After> && !below<OverallTag, After> && !below<RetryCauseTag, After> && !below<KeepaliveTag, After>) || EnvReturnPath<Env>::value,
                    "a Reply, Overall, Keepalive or return-path Retry layer reads what the layers below answer: the edge must declare returnPath");
      static_assert(EnvAllowsOverwrite<Env>::value || !below<LossyTag, After>,
                    "this class of edge does not accept a Store that overwrites: use RejectNewest (or Coalesce for an idempotent operation)");
      static_assert(!EnvAsync<Env>::value || below<WithinTag, After>,
                    "R-2: an operation that can stay Pending needs a deadline: place Within in the stack");
      constexpr uint8_t cls = EnvClass<Env>::value;
      static_assert(cls != uint8_t(Consumer::Display) || !below<RetryTag, After>,
                    "a display keeps only the latest record: it does not accept Retry (a retried record would arrive after a newer one)");
      static_assert(cls != uint8_t(Consumer::Display) || !below<BufferTag, After>,
                    "a display has one slot (Latest): Buffer(N) is for storage and fire-and-forget consumers");
      static_assert(cls != uint8_t(Consumer::Storage) || !below<LossyTag, After>,
                    "a storage consumer never overwrites a record it has not written: use Buffer (reject the newest, counted)");
      static_assert(cls != uint8_t(Consumer::FireForget) || !below<RetryTag, After>,
                    "a fire-and-forget consumer is never retried: a record it cannot take is counted and dropped");
      static_assert(cls != uint8_t(Consumer::Direct) || !below<QueueTag, After>,
                    "a direct shell takes the sample in on() and has no store: status only");
      static_assert(cls != uint8_t(Consumer::Direct) || !below<RetryTag, After>,
                    "a direct shell is never retried: it has no store to hold the record");
      static_assert(!below<CoalesceTag, After> || EnvIdempotent<Env>::value,
                    "Coalesce is for idempotent operations: declare idempotent on the edge, or use a Store that refuses the fresh one");
      static_assert(!hapi::query<hapi::TagIs<StatusTag>, After> || EnvLifecycle<Env>::value || EnvRawStatus<Env>::value,
                    "a stack that writes row status needs an app with lifecycle: World::setStatus is the only writer (declare lifecycle in the app and in the Env)");
      return true;
    }
  };

  template<typename Env, typename... L>
  using Controller = hapi::APIOf<EdgeTerm<Env>, Outer, L...>;

  // ---- no failure components: stateless, serve() is the operation --------------------------------------
  struct Bare {
    template<typename F> constexpr void serve(RowId, Cause, F&& op) const { (void)op(); }
  };

  // one stack object per registry row; nothing at all when the stack is empty
  template<typename S, uint8_t N, bool Empty = __is_empty(S)> struct Table;
  template<typename S, uint8_t N> struct Table<S, N, true>  { static S at(RowId) { return S{}; } };
  template<typename S, uint8_t N> struct Table<S, N, false> {
    inline static S rows[N];
    static S& at(RowId r) { return rows[r]; }
  };

  // ---- tick collection: the drivers whose stack has tick(now), folded statically ------------------------
  template<typename D, typename = void> struct StackOf { using Type = Bare; };
  template<typename D> struct StackOf<D, std::void_t<typename D::Stack>> { using Type = typename D::Stack; };

  struct TicksStack {
    template<typename D> using Check = typename hapi::Traverse<TicksStack, D>::Beta;
    template<typename D> using Apply = has_tick<typename StackOf<D>::Type>;
    template<typename... DD> using ApplyPack = hapi::Chain<DD...>;
  };

}
