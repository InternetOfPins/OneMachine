// The device edge (F5): what a polled driver adds to take part in failure handling. One failure controller per registry row.
//   Mode: checked (the driver observes the bus's verdicts), lifecycle, DevStack<Env>. K: the most rows of this driver a discovery can produce.
// A driver derives from DevEdge<Impl,W,Mode,N> and provides attempt(row) -> Outcome (its operation, on the checked read)
// and reinit(row) (what to do when its device comes back). Bus-level causes never reach this edge as failures:
// W::verdict() hands them to the bus edge and returns Blocked. Optional: recheck(row) or reinitOnBusReturn (see busReturned); presenceOnly (no ID register to reprobe).
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneBus/twiMaster.h>
#include "layers.h"
#include "cause.h"
#include "slots.h"

namespace fail {

  // kinds a driver adds to the edge's retry mask (retryExtra) and to what Recover acts on (recoverMask); none by default
  template<typename D, typename = void> struct RetryExtraOf { static constexpr uint8_t value = 0; };
  template<typename D> struct RetryExtraOf<D, std::void_t<decltype(D::retryExtra)>> { static constexpr uint8_t value = D::retryExtra; };
  template<typename D, typename = void> struct RecoverMaskOf { static constexpr uint8_t value = 0; };
  template<typename D> struct RecoverMaskOf<D, std::void_t<decltype(D::recoverMask)>> { static constexpr uint8_t value = D::recoverMask; };

  // a device without an ID register: an ACK at its address is all its identity there is (the reprobe reads nothing to compare)
  template<typename D, typename = void> struct PresenceOnly : std::false_type {};
  template<typename D> struct PresenceOnly<D, std::void_t<decltype(D::presenceOnly)>> : std::bool_constant<D::presenceOnly> {};

  template<typename Impl, typename W, typename Mode, uint8_t K>
  struct DevEdge {
    struct Env {
      // a NACK, or a failure the core cannot explain, is worth a gated retry; Recover is never on this edge (R-1)
      static constexpr uint8_t retryMask = uint8_t(KindSet<Kind::Absent, Kind::Unknown>::mask | RetryExtraOf<Impl>::value);
      static constexpr bool    lifecycle = Mode::lifecycle;
      static constexpr bool    returnPath = Mode::returnPath;          // the layers answer _f: Retry answers Pending while it holds
      static constexpr bool    idempotent = Mode::idempotent;          // the reads may be coalesced into a held retry
      // Recover on this edge re-initialises the device (kinds: those the driver declares in recoverMask, e.g. a state-loss canary's Corrupt)
      static constexpr uint8_t recoverMask = RecoverMaskOf<Impl>::value;
      static void busReset()                      {}
      static void recover(RowId r)                { W::route(W::reg.rows[r].parent); Impl::reinit(r); }
      static void reissue(RowId r)                { Impl::reissue(r); }
      static Outcome reprobe(RowId r)             { return DevEdge::reprobeRow(r); }
      static void setRowState(RowId r, uint8_t s) { DevEdge::applyState(r, s); }
    };
    using Stack = typename Mode::template DevStack<Env>;
    using Tab   = SlotTable<Stack, K, DriverRank<W, Impl>, Mode::returnPath>;

    static void serve(RowId row, Cause c) { Tab::serve(row, c, [row]() -> Outcome { return Impl::attempt(row); }); }
    // a stored operation is issued from tick, where nobody has routed the bus to this device yet
    static void reissue(RowId row)        { W::route(W::reg.rows[row].parent); serve(row, Cause::Reissue); }

    // the bus below came back: the state of the device is unknown until it says otherwise. What a driver answers: recheck(row), its own check that the device
    // kept its state (a canary read, forced now), or reinitOnBusReturn, its init is safe to run again. Neither: nothing is done for it. Called by the bus edge.
    static void busReturned([[maybe_unused]] RowId row) {
      if constexpr (HasRecheck<Impl>::value) Impl::recheck(row);
      else if constexpr (ReinitOnBusReturn<Impl>::value) { W::route(W::reg.rows[row].parent); Impl::reinit(row); }
    }

    // only while the bus below is up, and only while a health monitor (if any) has not quarantined the row: a
    // quarantine that only stopped pump()'s poll would still let Reprobe/Retry touch the bus on their own schedule,
    // which is exactly the bus time a flapping device's quarantine exists to stop (found building F7, not in its
    // gate: quarantine is "not polled" in both senses, the data path and the failure edge's own ticking).
    static void tickRow([[maybe_unused]] RowId row, [[maybe_unused]] uint32_t now) {
      if constexpr (has_tick<Stack>::value) {
        if (W::reg.status(W::reg.rows[row].parent) != discover::Status::Alive) return;
#ifndef F5_NEG_HEALTH_TICK_IGNORED
        if constexpr (discover::HealthOf<W>::value) if (!W::Health::polled(row)) return;
#endif
        Tab::tick(row, now);
      }
    }
    [[nodiscard]] static FailStatus failStatus(RowId row) noexcept { return Tab::status(row); }
    [[nodiscard]] static bool ownStale(RowId row)                  { return Tab::ownStale(row); }
    [[nodiscard]] static uint8_t overflows()                       { return Tab::overflowCount(); }
    // the return path: what the row's last operation came to (Idle without a Reply layer)
    static Outcome _serve(RowId row, Id id = 0)                    { return Tab::_serve(row, id); }
    static void resetFail()                                        { Tab::reset(); }

    // a checked register read: begin_write/end_write/request_from report, verdict() classifies
    static Outcome checkedRead(RowId row, uint8_t reg, uint8_t* out, uint8_t n) {
      using Twi = typename W::Twi;
      const uint8_t addr = W::reg.rows[row].busId;
      (void)Twi::begin_write(addr); Twi::write_byte(reg);
      if (!Twi::end_write()) return W::verdict(row);
      if (Twi::request_from(addr, n) == 0) return W::verdict(row);
      for (uint8_t i = 0; i < n; ++i) out[i] = Twi::read_byte();
      return Outcome::Ok();
    }

  private:
    // read-only presence and identity: the address answers, then the ID register says who
    static Outcome reprobeRow(RowId row) {
      using Twi = typename W::Twi;
      const uint8_t addr = W::reg.rows[row].busId;
      W::route(W::reg.rows[row].parent);
      if (!oneBus::probe<Twi>(addr)) return W::verdict(row);
#ifndef F5_NEG_PRESENCE_READS_ID
      if constexpr (PresenceOnly<Impl>::value) return Outcome::Ok();
#endif
      uint8_t got = 0;
      const Outcome o = checkedRead(row, discover::IdRegOf<Impl>::value, &got, 1);
      if (!o.isOk()) return o;
      return got == Impl::id ? Outcome::Ok() : Outcome::Fail(Kind::Absent, got);       // somebody else answers there
    }

    // the device edge's decision, into the registry. A device that came back on its own has lost its state, so its init runs
    // again; a bus that came back (bus edge) has not touched its devices, so nothing is re-initialised.
    static void applyState(RowId row, uint8_t s) {
      if (s == uint8_t(RowState::Alive)) {
        W::route(W::reg.rows[row].parent);
#ifndef F5_NEG_NO_REINIT
        Impl::reinit(row);
#endif
      }
      W::setStatus(row, discover::Status(s));
    }
  };

}
