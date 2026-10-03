// The bus edge (F5): one failure controller per bus row (the root, each mux channel), and the glue that
//   - hands a bus-level cause seen on a device operation to the bus that failed, and tells the device it was Blocked;
//   - gates everything below a faulted bus: its rows are Stale (not polled), its controllers do not tick;
//   - recovers a bus by one gated probe per interval (the controller's own Retry), Alive when it answers, Gone after M misses;
//   - writes status only through Self::setStatus, and puts back the faults a recovering bus does not own;
//   - tells the drivers of the devices below a bus that came back (busReturned): their state is unknown now.
// Self is the app (CRTP): Twi, reg, route(), setStatus(), under(), busReset(). Mode: lifecycle, BusStack<Env>.
// KB: the most bus rows a discovery can produce (the root and each bridge channel).
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneBus/twiMaster.h>
#include <oneMachine/discover/registry.h>
#include "layers.h"
#include "cause.h"
#include "slots.h"

namespace fail {

  struct BusBare { template<typename E> using Stack = Bare; };
  template<typename... L> struct BusCtl { template<typename E> using Stack = Controller<E, L...>; };

  template<typename D, typename = void> struct HasOwnStale : std::false_type {};
  template<typename D> struct HasOwnStale<D, std::void_t<decltype(D::ownStale(RowId{}))>> : std::true_type {};

  // is there a driver below that wants to be told its bus came back?
  template<typename L> struct AnyBusReturn;
  template<typename... D> struct AnyBusReturn<hapi::Chain<D...>> { static constexpr bool value = (WantsBusReturn<D>::value || ...); };

  // the driver of a device row is told its bus came back
  template<typename L> struct BusReturnFold;
  template<typename... D> struct BusReturnFold<hapi::Chain<D...>> {
    template<typename W> static void call(RowId m) { (one<W, D>(m), ...); }
  private:
    template<typename W, typename Dr> static void one([[maybe_unused]] RowId m) {
      if constexpr (WantsBusReturn<Dr>::value) { if (W::reg.rows[m].drv == discover::instOf<Dr>()) Dr::busReturned(m); }
    }
  };

  // does the row's own device edge hold a fault of its own?
  template<typename L> struct OwnStaleFold;
  template<typename... D> struct OwnStaleFold<hapi::Chain<D...>> {
    template<typename W> static bool any(RowId m) { return (one<W, D>(m) || ...); }
  private:
    template<typename W, typename Dr> static bool one([[maybe_unused]] RowId m) {
      if constexpr (HasOwnStale<Dr>::value) return W::reg.rows[m].drv == discover::instOf<Dr>() && Dr::ownStale(m);
      else return false;
    }
  };

  // ownStale for a world without a bus edge (an SPI bus: no ACK, no bus-level cause): a device row holds a fault of its own when its
  // driver's edge says so, a bus row never does. What a health monitor asks of Self::ownStale(m).
  template<typename Self, typename Drivers>
  struct DeviceOwnStale {
    [[nodiscard]] static bool ownStale(RowId m) { return !Self::reg.rows[m].isBus && OwnStaleFold<Drivers>::template any<Self>(m); }
  };

  template<typename Self, typename Drivers, uint8_t KB, typename Mode>
  struct BusEdge {
    struct Env {
      static constexpr uint8_t retryMask = KindSet<Kind::Timeout, Kind::Fault>::mask;
      static constexpr bool    lifecycle = Mode::lifecycle;
      static constexpr bool    returnPath = Mode::returnPath;
      static constexpr bool    idempotent = Mode::idempotent;
      static void busReset()                      { Self::busReset(); }
      static void reissue(RowId r)                { BusEdge::reissueBus(r); }
      static Outcome reprobe(RowId)               { return Outcome::Ok(); }
      static void setRowState(RowId r, uint8_t s) { BusEdge::applyState(r, s); }
    };
    using Ctl = typename Mode::template BusStack<Env>;
    using Tab = SlotTable<Ctl, KB, BusRank<Self>, Mode::returnPath>;
    static constexpr RowId root = discover::rootRow;

    // ---- what a device operation reports -----------------------------------------------------------------
    // The outcome of a failed operation on `row`'s device. A bus-level cause is the bus's: it goes to the controller of the
    // bus that failed and the device is only Blocked (nothing is counted against it).
    [[nodiscard]] static Outcome verdict(RowId row) {
      const oneBus::TwiCause c = oneBus::causeOf<typename Self::Twi>();
      const RowId bus = Self::reg.rows[row].parent;
      // A failure the core cannot explain (a read leg on Wire) may still be the bus's: the write probe reports a cause where the
      // read did not, so one probe of the bus settles it. Only on an Unknown, only once.
      if (c == oneBus::TwiCause::None || c == oneBus::TwiCause::Unknown) {
        const Outcome p = probeBus(bus);
        if (p.failed()) {
          const Outcome s = settle(bus, p);
          if (s.failed()) busFault(bus, s);
          return Outcome::Blocked();
        }
      }
      if (c == oneBus::TwiCause::None) return Outcome::Fail(Kind::Unknown, uint8_t(c));    // failed, and the core does not say why
      const Outcome f = fromCause(c);
      if (!busLevel(c)) return f;
      const Outcome s = settle(bus, f);
      if (s.failed()) busFault(bus, s);
      return Outcome::Blocked();
    }

    // ---- bus faults ----------------------------------------------------------------------------------------------
    static void busFault(RowId bus, Outcome o) {
      Tab::serve(bus, Cause::Fresh, [o]() -> Outcome { return o; });
    }

    // A bus-level failure seen through `bus`. If the bus above still answers, the fault is this bus's; if it does not, the
    // fault is the bus above's (reported to it) and this bus is only Blocked. Returns what to record for `bus`.
    static Outcome settle(RowId bus, Outcome o) {
      if (bus == root) return o;
      const RowId up = carrierOf(bus);
      if (Self::reg.status(up) != discover::Status::Alive) return Outcome::Blocked();     // already down: its own controller owns it
      const Outcome c = probeBus(up);
      if (!c.failed()) return o;
      const Outcome u = settle(up, c);
      if (u.failed()) busFault(up, u);
      return Outcome::Blocked();
    }

    // the bus probe: route to the bus, ask one address on it. Anything but a bus-level cause means the bus answered (a NACK
    // is a device not being there, and that is the device's business).
    static Outcome probeBus(RowId bus) {
      if (bus != root) Self::route(bus);
      const uint8_t a = refAddr(bus);
      if (a == 0) return Outcome::Ok();
      if (oneBus::probe<typename Self::Twi>(a)) return Outcome::Ok();
      const oneBus::TwiCause c = oneBus::causeOf<typename Self::Twi>();
      return busLevel(c) ? fromCause(c) : Outcome::Ok();
    }

    static void reissueBus(RowId bus) {
      Tab::serve(bus, Cause::Reissue, [bus]() -> Outcome {
        const Outcome o = probeBus(bus);
        return o.failed() ? settle(bus, o) : o;
      });
    }

    // ---- the one place a failure component's decision becomes registry status ---------------------------------------
    static void applyState(RowId row, uint8_t s) {
      Self::setStatus(row, discover::Status(s));
      if (s == uint8_t(RowState::Alive) && Self::reg.rows[row].isBus) {
        reapply(row);
        busReturned(row);
      }
    }

    // ---- time: buses first, each only while the bus above it is up ----------------------------------------------------
    static void tickBuses([[maybe_unused]] uint32_t now) {
      if constexpr (has_tick<Ctl>::value) {
        for (RowId r = 0; r < Self::reg.count; ++r) {
          if (!Self::reg.rows[r].isBus) continue;
          if (r != root && Self::reg.status(carrierOf(r)) != discover::Status::Alive) continue;
          Tab::tick(r, now);
        }
      }
    }

    static void resetCtl() { Tab::reset(); }
    [[nodiscard]] static FailStatus busStatus(RowId r) noexcept { return Tab::status(r); }
    [[nodiscard]] static uint8_t overflows() { return Tab::overflowCount(); }
    static Outcome busServe(RowId bus, Id id = 0) { return Tab::_serve(bus, id); }

    // ---- who owns a fault ------------------------------------------------------------------------------------------------
    [[nodiscard]] static bool ownStale(RowId m) {
      if (Self::reg.rows[m].isBus) return Tab::ownStale(m);
      return OwnStaleFold<Drivers>::template any<Self>(m);
    }

    // the bus a bus row hangs off: a channel's is the bus its bridge sits on
    static RowId carrierOf(RowId bus) { return Self::reg.rows[Self::reg.rows[bus].parent].parent; }

  private:
    // a bus that comes back brings its subtree back, except the rows that are down for their own reasons
    static void reapply(RowId bus) {
      for (RowId m = RowId(bus + 1); m < Self::reg.count; ++m)
        if (Self::under(m, bus) && ownStale(m)) Self::setStatus(m, discover::Status::Stale);
    }

    // A bus that came back has left the state of the devices below it unknown: each one that is up is asked to check it (or to initialise
    // again, if its driver says that is safe). A device that is down for its own reasons is initialised when it comes back, not now.
    static void busReturned([[maybe_unused]] RowId bus) {
      if constexpr (AnyBusReturn<Drivers>::value) {
        for (RowId m = RowId(bus + 1); m < Self::reg.count; ++m) {
          if (Self::reg.rows[m].isBus) continue;
          if (!Self::under(m, bus)) continue;
          if (Self::reg.status(m) != discover::Status::Alive) continue;      // reapply() has put a device that is down for its own reasons back to Stale
          BusReturnFold<Drivers>::template call<Self>(m);
        }
      }
    }

    // one address that lives on `bus`: the first device row under it
    static uint8_t refAddr(RowId bus) {
      for (RowId m = 0; m < Self::reg.count; ++m)
        if (!Self::reg.rows[m].isBus && Self::reg.rows[m].parent == bus) return Self::reg.rows[m].busId;
      return 0;
    }
  };

}
