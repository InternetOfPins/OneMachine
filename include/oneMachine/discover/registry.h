// Registry: the only runtime-stateful object. Flat fixed-capacity table, parent by row index.
// A row's busId is its identity as seen by its parent bus: an I2C address for a device row,
// a channel number for a bus row behind a bridge. Row 0 is the root bus (the MCU peripheral).
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include "driver.h"
#include "entries.h"

// keeps pump() a function of its own, so the one indirect call it makes can be found in a disassembly
#if defined(_MSC_VER)
#define ONEMACHINE_NOINLINE __declspec(noinline)
#else
#define ONEMACHINE_NOINLINE __attribute__((noinline))
#endif

namespace discover {

  inline constexpr RowId rootRow = 0;

  struct I2cBus { static constexpr uint32_t idMax = 0x7F; };

  template<typename... B> constexpr uint32_t maxBusId() {
    uint32_t m = 0;
    ((m = B::idMax > m ? B::idMax : m), ...);
    return m;
  }
  template<uint32_t V> using UIntFor =
    std::conditional_t<(V <= 0xFF), uint8_t, std::conditional_t<(V <= 0xFFFF), uint16_t, uint32_t>>;

  template<typename L> struct BusIdOf;
  template<typename... B> struct BusIdOf<hapi::Chain<B...>> { using Type = UIntFor<maxBusId<B...>()>; };

  template<typename BusIdT, uint8_t N> struct Registry;
  // Scan: the bus-specific identification fold, given by the app (its `run<Self,Drivers>(bus)` probes one bus row); the registry
  // itself names no bus protocol. discover::I2cScan (identify.h) is the I2C one; an app includes it beside its bus, not through here.
  template<typename Self, typename TwiT, typename Consumers, typename Drivers, uint8_t N, typename Scan, typename Buses = hapi::Chain<I2cBus>> struct World;
#ifdef DISCOVER_TEST_RAW_STATUS
  struct RawStatus;
#endif

  // A row's status is read through status() and written only by the registry (World::setStatus), so a component cannot
  // change it without going through the lifecycle. The declaration order is the layout: status, then isBus, in one byte.
  template<typename BusIdT>
  struct RowT {
    BusIdT   busId;
    IDriver* drv;
    RowId    parent;
  private:
    uint8_t  st_   : 2;    // Status; a plain integer field, gcc 7 rejects a 2-bit scoped-enum field
  public:
    uint8_t  isBus : 1;
    [[nodiscard]] Status status() const noexcept { return Status(st_); }
  private:
    template<typename B, uint8_t M> friend struct Registry;
  };

  template<typename BusIdT, uint8_t N>
  struct Registry {
    using Row = RowT<BusIdT>;
    Row     rows[N] = {};   // constant-initialized: no guard variable, no global ctor
    RowId   count    = 0;
    uint8_t overflow = 0;

    void reset() { count = 0; overflow = 0; add(0, nullptr, noRow, true); }

    RowId add(BusIdT busId, IDriver* drv, RowId parent, bool isBus) {
      if (count >= N) { ++overflow; return noRow; }
      Row& r = rows[count];
      r.busId = busId; r.drv = drv; r.parent = parent;
      r.st_ = uint8_t(Status::Alive); r.isBus = isBus;
      return count++;
    }

    [[nodiscard]] Status status(RowId r) const noexcept { return r < count ? rows[r].status() : Status::Gone; }
    // total like status(): a row that is not in the table has no driver
    [[nodiscard]] const IDriver* driverOf(RowId r) const noexcept { return r < count ? rows[r].drv : nullptr; }

  private:
    void writeStatus(RowId r, Status s) noexcept { if (r < count) rows[r].st_ = uint8_t(s); }
    template<typename S, typename T, typename C, typename D, uint8_t M, typename SC, typename B> friend struct World;
#ifdef DISCOVER_TEST_RAW_STATUS
    friend struct RawStatus;
#endif
  };

#ifdef DISCOVER_TEST_RAW_STATUS
  // Writes a status outside World::setStatus: for a test of an app that has no lifecycle (where a binding's own status
  // check is what is under test). A component uses World::setStatus; naming RawStatus is a deliberate bypass -- gated
  // behind DISCOVER_TEST_RAW_STATUS so an ordinary #include never sees the escape hatch (C1 item 10: test-only bypasses
  // live in test support, not in a header a user includes as-is).
  struct RawStatus {
    template<typename Reg> static void set(Reg& reg, RowId r, Status s) { reg.writeStatus(r, s); }
  };
#endif

  // an app opts in to row lifecycle (skip dead rows, release bindings and state when a row goes Gone) by declaring
  // `static constexpr bool lifecycle = true;` and the hooks `release(row)` and `unbindAll()`
  template<typename S, typename = void> struct LifecycleOf : std::false_type {};
  template<typename S> struct LifecycleOf<S, std::void_t<decltype(S::lifecycle)>> : std::bool_constant<S::lifecycle> {};

  // Self is the concrete application type (CRTP): drivers name it before it is complete.
  template<typename Self, typename TwiT, typename Consumers, typename Drivers, uint8_t N, typename Scan, typename Buses>
  struct World {
    using Twi    = TwiT;
    using Bus    = TwiT;   // the same bus, by its generic name: an SPI World's TwiT is its SpiSlots bus
    using BusIdT = typename BusIdOf<Buses>::Type;

    inline static Registry<BusIdT, N> reg;

    // DeviceState: one slot per row, the largest declared by the drivers of the list; nothing when none declares one.
    // Named through a nested template so that an app whose drivers declare none never instantiates it.
    // `Drivers` is a list of entries (identify.h); a plain list of drivers is one. DriverList: the distinct drivers they name.
    using Entries    = Drivers;   // the entries as given: what the drivers' ids are read from (identify.h, DeclaredIds)
    using DriverList = DriversIn<Drivers>;

    template<typename D = DriverList> struct Dev {
      using Slot  = DevSlotOf<D>;
      using Table = discover::Table<Slot, N>;
    };
    template<typename Impl> static auto& devState(RowId r) {
      static_assert(DeviceStateOf<Impl>::has, "driver declares no DeviceState");
      return Dev<>::Table::at(r).template as<typename DeviceStateOf<Impl>::Type>();
    }
    static void clearState(RowId r) { Dev<>::Table::clear(r); }
    template<typename Cap> inline static CapFanoutT<Cap, Consumers> fan{};

    template<typename Cap>
    static void emit(RowId row, typename Cap::Value v) { fan<Cap>.deliver(Sample<Cap>{row, v}); }

    // Make `bus` the only thing visible: top-down along the path from the root, each level selects the
    // bridge channel on the path and clears every other bridge on that level (and all bridges on `bus`).
    static void route(RowId bus) {
      using Bridges = hapi::Eval<hapi::Filter<IsBridge>, DriverList>;
      RowId path[N];
      uint8_t n = 0;
      for (RowId b = bus; b != rootRow && b != noRow && n < N; b = reg.rows[reg.rows[b].parent].parent)
        path[n++] = b;   // channel row -> its bridge row -> the bus the bridge sits on
      RowId level = rootRow;
      for (;;) {
        const RowId next = n ? path[n - 1] : noRow;
        for (RowId m = 0; m < reg.count; ++m) {
          const auto& row = reg.rows[m];
          if (row.isBus || row.parent != level) continue;
          if constexpr (LifecycleOf<Self>::value) if (row.status() != Status::Alive) continue;
          if (next != noRow && reg.rows[next].parent == m)
            DriverSet<Bridges>::select(row.drv, row.busId, reg.rows[next].busId);
          else
            DriverSet<Bridges>::clear(row.drv, row.busId);
        }
        if (!n) break;
        level = path[--n];
      }
    }

    // a selected bridge channel is electrically joined to its parent bus, so a scan of it also answers
    // for every device already claimed on the buses upstream
    static bool claimedUpstream(RowId bus, uint8_t addr) {
      for (;;) {
        for (RowId r = 0; r < reg.count; ++r)
          if (!reg.rows[r].isBus && reg.rows[r].parent == bus && reg.rows[r].busId == addr) return true;
        if (bus == rootRow) return false;
        bus = reg.rows[reg.rows[bus].parent].parent;
      }
    }

    // bridges first, each cleared as it is found, so a stale selection cannot show devices behind it
    // as if they sat on this bus; then everything else -- Scan's own concern, the registry only routes first
    static void scan(RowId bus) { route(bus); Scan::template run<Self, Drivers>(bus); }

    // breadth-first: rows appended while scanning are visited by the same loop
    static void discover() {
      reg.reset();
      if constexpr (LifecycleOf<Self>::value) { Dev<>::Table::clearAll(); Self::unbindAll(); }
      for (RowId r = 0; r < reg.count; ++r)
        if (reg.rows[r].isBus) scan(r);
    }

    // one indirect call per device row
    ONEMACHINE_NOINLINE static void pump() {
      if constexpr (LifecycleOf<Self>::value) {
        for (RowId r = 0; r < reg.count; ++r)
          if (!reg.rows[r].isBus && reg.status(r) == Status::Alive) {
            if constexpr (HealthOf<Self>::value) if (!Self::Health::polled(r)) continue;
            reg.rows[r].drv->poll(r);
          }
      } else {
        for (RowId r = 0; r < reg.count; ++r)
          if (!reg.rows[r].isBus) reg.rows[r].drv->poll(r);
      }
    }

    // the one place a row's status is written. The subtree below the row follows it (children come after their
    // parent in the table); Gone is final until the next discovery, and clears the rows' DeviceState and releases
    // the consumers bound to them.
    static bool under(RowId m, RowId r) {
      for (RowId p = reg.rows[m].parent; p != noRow; p = reg.rows[p].parent)
        if (p == r) return true;
      return false;
    }
    static void setStatus(RowId r, Status st) {
      static_assert(LifecycleOf<Self>::value,
                    "setStatus writes status, and only an app with lifecycle skips dead rows and releases their bindings: declare `static constexpr bool lifecycle = true`");
      if (r >= reg.count) return;
      apply(r, st);
      for (RowId m = RowId(r + 1); m < reg.count; ++m)
        if (under(m, r))
          apply(m, st);
    }
  private:
    static void apply(RowId m, Status st) {
      if (reg.status(m) == Status::Gone) return;
      reg.writeStatus(m, st);
      if constexpr (LifecycleOf<Self>::value) {
        if (st == Status::Gone) {
          Dev<>::Table::clear(m);
          Self::release(m);
        }
      }
    }
  public:
  };

}
