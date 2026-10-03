// A registry-wide health monitor (F7): every row's flap rate and an approximate bus cost, watched from outside the
// per-row failure edges -- there is no operation to run here and no per-row call site, so this is not a Chain<> of
// Controller<Env,Layers...> layers (that shape is for one row's data path). Detect: a row's own Alive->Stale
// transition, counted only when the row itself is at fault (Self::ownStale: F5's own attribution, reused as-is --
// a device following its bus down is not the device's flap). Status: two shift-based EWMAs per row (flap rate, cost
// rate), hysteresis (enter > exit) so the monitor cannot flap itself. Gate: a quarantine duration that grows across
// episodes (Fibonacci steps) and relaxes after a quiet spell. Act: quarantine (pump() and the device edge's own tick
// both skip the row, with a 1-in-N probation trickle to test recovery), disconnect (calls the driver's optional
// isolate(row)), or escalate (a `required` row is never quarantined or disconnected).
//
// Two cadences, not one: a transition is caught the instant it happens (onEdge, called every raw tick -- a cheap
// status read and compare, same cost class as pump() skipping a dead row) so a flap that starts and clears inside
// one control period is never missed; the EWMAs decay and the policy runs on the coarser control period (onTick,
// Cfg::tickMs) -- the "decayed on tick" the round's handoff asked for. Found building this, not anticipated in the
// gate: ticking the decay on every raw call (as a first cut did) makes the average meaningless whenever the control
// period is much longer than the shift's own time constant; it needs its own period, independent of how often the
// caller happens to tick.
//
// Zero cost when an app declares no `Health` type: `discover::HealthOf<W>` (driver.h) is the only thing pump() and
// DevEdge::tickRow see, and it compiles to nothing when absent.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/discover/registry.h>
#include "busedge.h"
#include "devedge.h"
#include "cause.h"

namespace fail {

  // Fibonacci step, integer: up() advances one step (capped at `cap`), down() is its exact inverse (a(n),b(n) -> the
  // pair one step earlier), so cooling off after a quiet spell retraces the same sequence a growing episode climbed.
  struct FibStep {
    uint16_t prev = 1, cur = 1;                                // `cur` is the active multiplier
    void up(uint16_t cap) { const uint16_t n = uint16_t(prev + cur > cap ? cap : prev + cur); prev = cur; cur = n; }
    void down() { if (cur > 1) { const uint16_t p = uint16_t(cur - prev); cur = prev; prev = p; } }
  };

  // optional per-driver declarations the monitor reads; all default to "do nothing extra"
  template<typename D, typename = void> struct RequiredOf : std::false_type {};
  template<typename D> struct RequiredOf<D, std::void_t<decltype(D::required)>> : std::bool_constant<D::required> {};
  template<typename D, typename = void> struct MayIsolateOf : std::false_type {};
  template<typename D> struct MayIsolateOf<D, std::void_t<decltype(D::mayIsolate)>> : std::bool_constant<D::mayIsolate> {};
  template<typename D, typename = void> struct HasIsolate : std::false_type {};
  template<typename D> struct HasIsolate<D, std::void_t<decltype(D::isolate(RowId{}))>> : std::true_type {};

  // why a row escalated: the flap rate crossed enterQ, the cost rate crossed enterD, or both
  enum class EscalateReason : uint8_t { Flap = 1, Cost = 2, Both = 3 };

  // the master policy hook, declared by the app: onEscalate(row, reason). Default: none -- escalation is only
  // counted (HealthRow::escalations), reaching nowhere; a real destination (a bus reset, a safe-state action) is
  // actuation, out of this round's scope (C1 item 4's own boundary). Zero cost when the app declares no hook.
  template<typename W, typename = void> struct HasOnEscalate : std::false_type {};
  template<typename W> struct HasOnEscalate<W, std::void_t<decltype(W::onEscalate(RowId{}, EscalateReason{}))>> : std::true_type {};

  // one row's FailStatus, from its own driver (pointer-compare fold, no virtual -- the same idiom as OwnStaleFold)
  template<typename L> struct FailStatusFold;
  template<typename... D> struct FailStatusFold<hapi::Chain<D...>> {
    template<typename W> static FailStatus of(RowId m) { FailStatus s{}; (one<W, D>(m, s) || ...); return s; }
  private:
    template<typename W, typename Dr> static bool one(RowId m, FailStatus& s) {
      if (W::reg.rows[m].drv != discover::instOf<Dr>()) return false;
      s = Dr::failStatus(m); return true;
    }
  };
  template<typename L> struct RequiredFold;
  template<typename... D> struct RequiredFold<hapi::Chain<D...>> {
    template<typename W> static bool of(RowId m) { return (one<W, D>(m) || ...); }
  private:
    template<typename W, typename Dr> static bool one(RowId m) { const bool here = W::reg.rows[m].drv == discover::instOf<Dr>(); return here && RequiredOf<Dr>::value; }
  };
  // a bus row's own driver is the bridge behind it (Mux, say); the root row's is whatever the app's topology gives it
  // (often none) -- reuses the same RequiredOf trait, generalized over any Drivers list the app names.
  template<typename W2> struct RequiredBusFold {
    static bool of(RowId m) { return RequiredFold<typename W2::DriverList>::template of<W2>(m); }
  };

  template<typename L> struct MayIsolateFold;
  template<typename... D> struct MayIsolateFold<hapi::Chain<D...>> {
    template<typename W> static bool of(RowId m) { return (one<W, D>(m) || ...); }
  private:
    template<typename W, typename Dr> static bool one(RowId m) { const bool here = W::reg.rows[m].drv == discover::instOf<Dr>(); return here && MayIsolateOf<Dr>::value; }
  };
  template<typename L> struct IsolateFold;
  template<typename... D> struct IsolateFold<hapi::Chain<D...>> {
    // true when the row's driver has an isolate() and it ran
    template<typename W> static bool call(RowId m) { return (one<W, D>(m) || ...); }
  private:
    template<typename W, typename Dr> static bool one([[maybe_unused]] RowId m) {
      if constexpr (HasIsolate<Dr>::value) { if (W::reg.rows[m].drv == discover::instOf<Dr>()) { Dr::isolate(m); return true; } }
      return false;
    }
  };

  // roughly how long a failed transaction of this kind holds the bus, in weight units, not microseconds: a real
  // measurement would need a clock read around every checkedRead/probe in busedge.h/devedge.h, live even when no
  // Health is composed unless every call site grew its own guard -- a wider change than ranking devices by drag
  // needs. Timeout/Fault/Corrupt ride the core's bounded wait or a retried transaction (heavy); Absent/Refused/
  // Overflow/Unknown are a NACK or a local decision (light). An approximation, reported as one.
  constexpr uint8_t costWeight(uint8_t kind) {
    return (kind == uint8_t(Kind::Timeout) || kind == uint8_t(Kind::Fault) || kind == uint8_t(Kind::Corrupt)) ? 32 : 1;
  }

  struct DefaultHealthCfg {
    static constexpr uint32_t tickMs = 500;                // the monitor's own control period: decay and policy run this often, not on every raw call
    static constexpr uint16_t enterQ = 96, exitQ = 32;     // flap EWMA thresholds, 0..256 scale
    static constexpr uint16_t enterD = 96, exitD = 32;     // cost EWMA thresholds, 0..256 scale
    static constexpr uint32_t quarantineUnitMs = 2000;     // the Fibonacci multiplier's unit
    static constexpr uint16_t fibCap = 21;
    static constexpr uint8_t  probeN = 4;                  // one control period in N is a probation probe window
    static constexpr uint8_t  quietPeriodsToCool = 6;
    static_assert(enterQ > exitQ && enterD > exitD, "Health: enter thresholds must exceed exit thresholds (hysteresis)");
  };

  struct HealthRow {
    uint8_t  lastStatus = uint8_t(discover::Status::Alive);
    uint16_t flapEwma = 0, costEwma = 0;
    uint16_t flapCount = 0, escalations = 0;
    uint8_t  lastFails = 0, lastRetries = 0, quietPeriods = 0;
    bool     quarantined = false, probation = false, disconnected = false, probeWindowOpen = false, probing = false;
    uint32_t until = 0;
    uint16_t periodsInProbation = 0;
    FibStep  fib;
  };

  // W: the app (CRTP; reg, ownStale). Drivers: the entries list (for FailStatus/required/mayIsolate/isolate lookup).
  // N: the registry's own row capacity (every row, bus or device, gets a slot; a bus row is report-only this round).
  template<typename W, typename Drivers, uint8_t N, typename Cfg = DefaultHealthCfg>
  struct HealthT {
    inline static HealthRow rows[N] = {};
    inline static uint32_t nextTick = 0;
    inline static uint16_t period = 0;

    // pump() and DevEdge::tickRow both ask this before touching the row: false during a hard quarantine, true on a
    // 1-in-probeN control period (the probation trickle), true otherwise. Overrides F5's own Alive/Stale verdict.
    static bool polled(RowId r) {
      const HealthRow& h = rows[r];
      return !h.quarantined || h.probeWindowOpen;
    }

    [[nodiscard]] static const HealthRow& status(RowId r) noexcept { return rows[r]; }
    static void reset() { for (auto& h : rows) h = HealthRow{}; nextTick = 0; period = 0; }

    // called every raw tick (cheap: one status read and compare per row) so a flap that starts and clears inside one
    // control period is never missed; pushes the EWMA up immediately on a transition, independent of decay's cadence.
    static void onEdge() {
      for (RowId r = 0; r < W::reg.count; ++r) {
        HealthRow& h = rows[r];
        const uint8_t cur = uint8_t(W::reg.status(r));
        if (h.lastStatus == uint8_t(discover::Status::Alive) && cur == uint8_t(discover::Status::Stale) && W::ownStale(r)) {
          h.flapEwma = ewma(h.flapEwma, 256);
          if (h.flapCount != 0xFFFF) ++h.flapCount;
          h.quietPeriods = 0;
        }
        h.lastStatus = cur;
      }
    }

    // the monitor's own control period: decay, cost sampling and policy. A no-op between periods (one comparison).
    static void onTick(uint32_t now) {
      if (now < nextTick) return;
      nextTick = now + Cfg::tickMs;
      ++period;
      for (RowId r = 0; r < W::reg.count; ++r) {
        HealthRow& h = rows[r];
        h.flapEwma = ewma(h.flapEwma, 0);                                  // decay: pulled toward 0 when nothing flapped this period
        if (h.flapEwma < Cfg::exitQ && !h.quarantined) { if (h.quietPeriods != 0xFF) ++h.quietPeriods; }   // a row that is not polled is quiet by construction: that does not cool it
        if (!W::reg.rows[r].isBus) {
          const FailStatus fs = FailStatusFold<Drivers>::template of<W>(r);
          const bool moved = fs.fails != h.lastFails || fs.retries != h.lastRetries;
          h.lastFails = fs.fails; h.lastRetries = fs.retries;
          const uint16_t raw = moved ? uint16_t(costWeight(fs.lastKind)) * 8u : 0;
          h.costEwma = ewma(h.costEwma, raw > 255 ? uint16_t(255) : raw);
        }
        h.probeWindowOpen = h.quarantined && h.probation && (period % Cfg::probeN == 0);
        policy(r, h, now);
      }
    }

  private:
    static uint16_t ewma(uint16_t avg, uint16_t sample) { return uint16_t(int32_t(avg) + ((int32_t(sample) - int32_t(avg)) >> 3)); }

    static void policy(RowId r, HealthRow& h, uint32_t now) {
      const bool required = W::reg.rows[r].isBus ? RequiredBusFold<W>::of(r) : RequiredFold<Drivers>::template of<W>(r);
      if (W::reg.rows[r].isBus) {                                          // report-only for a bus row: nothing to disconnect it from
        if (required && (h.flapEwma >= Cfg::enterQ)) escalate(r, h, EscalateReason::Flap);
        else if (h.flapEwma < Cfg::exitQ) { if (h.quietPeriods >= Cfg::quietPeriodsToCool) { h.fib.down(); h.quietPeriods = 0; } }
        return;
      }
      if (h.quarantined) {
        if (!h.probation) {
          if (now < h.until) return;                                       // the hard block: not polled at all, no probes to look at
          h.probation = true; return;                                      // elapsed: open-ended probation starts now
        }
        if (h.probeWindowOpen) { h.probing = true; return; }               // a probe window: the row is polled and ticked this period; what it did is judged at the next tick
        if (!h.probing) return;                                            // between probe windows: nothing new to decide
        h.probing = false;                                                 // the window has closed: judge the probe, not the averages that decayed while the row was left alone
        if (W::reg.status(r) == discover::Status::Alive && h.flapEwma < Cfg::exitQ && h.costEwma < Cfg::exitD)
        { h.quarantined = false; h.probation = false; h.probeWindowOpen = false; h.disconnected = false; return; }
        h.fib.up(Cfg::fibCap); h.probation = false; h.until = now + h.fib.cur * Cfg::quarantineUnitMs;   // still bad on a probe: a fresh, longer hard block
        return;
      }
      const bool overQ = h.flapEwma >= Cfg::enterQ, overD = h.costEwma >= Cfg::enterD;
      if (overQ || overD) {
        if (required) { escalate(r, h, overQ && overD ? EscalateReason::Both : overD ? EscalateReason::Cost : EscalateReason::Flap); return; }
        if (!MayIsolateFold<Drivers>::template of<W>(r)) return;                        // Report only: no isolate declared for this row
        h.fib.up(Cfg::fibCap);
        if (overD) h.disconnected = IsolateFold<Drivers>::template call<W>(r);
        h.quarantined = true; h.probation = false; h.until = now + h.fib.cur * Cfg::quarantineUnitMs;
      } else if (h.quietPeriods >= Cfg::quietPeriodsToCool) { h.fib.down(); h.quietPeriods = 0; }
    }

    static void escalate([[maybe_unused]] RowId r, HealthRow& h, [[maybe_unused]] EscalateReason reason) {
      if (h.escalations != 0xFFFF) ++h.escalations;
      if constexpr (HasOnEscalate<W>::value) W::onEscalate(r, reason);
    }
  };

}
