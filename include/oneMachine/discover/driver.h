// Drivers: one polymorphic interface at the top (IDriver::poll, the only indirect hop on the data path);
// everything a concrete driver does after that is static. W is the world type (incomplete when a driver
// is declared, complete by the time a member body is instantiated).
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include "capability.h"
#include "state.h"

namespace discover {

  enum class Status : uint8_t { Alive, Stale, Gone };

  struct IDriver {
    virtual void poll(RowId row) = 0;
  protected:
    ~IDriver() = default;
  };

  // one singleton per driver type; a data member of a template, not a function-local static (no guard variable)
  template<typename Dr> struct Inst { inline static Dr obj{}; };
  template<typename Dr> IDriver* instOf() { return static_cast<IDriver*>(&Inst<Dr>::obj); }

  struct IsBridge {
    template<typename O> using Check = typename hapi::Traverse<IsBridge,O>::Beta;
    template<typename O> using Apply = std::bool_constant<O::isBridge>;
    template<typename... OO> using ApplyPack = hapi::Chain<OO...>;
  };

  // W::bind<Impl>(row) is called from found() when W declares it: the point where the concrete type and the new row are known.
  template<typename W, typename Impl, typename = void> struct HasBind : std::false_type {};
  template<typename W, typename Impl>
  struct HasBind<W, Impl, std::void_t<decltype(W::template bind<Impl>(RowId{}))>> : std::true_type {};

  // W::Health::polled(row) says whether a row a health monitor has quarantined should be skipped by pump() this tick;
  // absent by default (no monitor composed, or none for this app).
  template<typename W, typename = void> struct HealthOf : std::false_type {};
  template<typename W> struct HealthOf<W, std::void_t<decltype(W::Health::polled(RowId{}))>> : std::true_type {};

  // is the row's read(row) called by pump()? By default when the driver produces something; `static constexpr bool polled` says otherwise
  // (a display produces nothing and is refreshed).
  template<typename D, typename = void> struct PolledOf : std::bool_constant<(D::Produces::size > 0)> {};
  template<typename D> struct PolledOf<D, std::void_t<decltype(D::polled)>> : std::bool_constant<D::polled> {};

  // Concrete driver = DriverBase<Impl,W> + statics: addrLo/addrHi/id, Produces, read(row); a bridge adds
  // isBridge/channels/select(addr,ch)/clear(addr); optional polled. Impl is stateless; per-device state lives in the registry row.
  template<typename Impl, typename W>
  struct DriverBase : IDriver {
    static constexpr bool isBridge = false;
    using Produces = hapi::Chain<>;
    using Self     = Impl;

    // total: an unknown row reads Gone
    [[nodiscard]] static Status status(RowId row) noexcept { return W::reg.status(row); }

    void poll([[maybe_unused]] RowId row) override final {
      if constexpr (PolledOf<Impl>::value) {
        W::route(W::reg.rows[row].parent);
        Impl::read(row);
      }
    }

    static uint8_t addrOf(RowId row) { return W::reg.rows[row].busId; }

    // the row's DeviceState, when Impl declares one
    template<typename I = Impl> static auto& dev(RowId row) { return W::template devState<I>(row); }

    static void readRegs(uint8_t addr, uint8_t reg, uint8_t* out, uint8_t n) {
      using Twi = typename W::Twi;
      Twi::begin_write(addr); Twi::write_byte(reg); Twi::end_write();
      (void)Twi::request_from(addr, n);
      for (uint8_t i = 0; i < n; ++i) out[i] = Twi::read_byte();
    }

    static bool probe(uint8_t addr) {
      uint8_t got;
      readRegs(addr, 0, &got, 1);
      return got == Impl::id;
    }

    static void found(uint8_t addr, RowId bus) {
      RowId self;
      if constexpr (W::reviveRows) self = W::reg.addReusing(addr, instOf<Impl>(), bus);   // a part that came back takes its Gone row again
      else self = W::reg.add(addr, instOf<Impl>(), bus, false);
      if constexpr (Impl::isBridge) {
        Impl::clear(addr);   // a bridge may keep a stale selection across a warm reset
        if (self != noRow)
          for (uint8_t ch = 0; ch < Impl::channels; ++ch) W::reg.add(ch, instOf<Impl>(), self, true);
      }
      if constexpr (DeviceStateOf<Impl>::has) { if (self != noRow) W::clearState(self); }
      if constexpr (HasInit<Impl>::value) { if (self != noRow) Impl::init(self); }
      if constexpr (HasBind<W, Impl>::value)
        if (self != noRow) W::template bind<Impl>(self);
    }

    template<typename Cap>
    static void emit(RowId row, typename Cap::Value v) {
      static_assert(hapi::Exists<hapi::SameAs<Cap>, typename Impl::Produces>::value,
                    "driver emits a capability it does not declare in Produces");
      W::template emit<Cap>(row, v);
    }
  };

  template<typename... TT> struct AllDistinct : std::true_type {};
  template<typename A, typename... R> struct AllDistinct<A,R...>
    : std::bool_constant<(!std::is_same<A,R>::value && ...) && AllDistinct<R...>::value> {};

  template<typename L> struct DriverSet;
  template<typename... D>
  struct DriverSet<hapi::Chain<D...>> {
    static constexpr bool distinct = AllDistinct<D...>::value;

    // pointer-compare dispatch to the bridge type behind `d` (false if `d` is not one); no indirect call
    static bool select([[maybe_unused]] const IDriver* d, [[maybe_unused]] uint8_t addr, [[maybe_unused]] uint8_t ch) {
      return ((d == instOf<D>() ? (D::select(addr, ch), true) : false) || ...);
    }
    static bool clear([[maybe_unused]] const IDriver* d, [[maybe_unused]] uint8_t addr) {
      return ((d == instOf<D>() ? (D::clear(addr), true) : false) || ...);
    }
  };

}
