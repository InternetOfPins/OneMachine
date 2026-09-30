#pragma once
// oneMachine/role/found.h -- a role endpoint bound when discovery finds its device; the device routes every request to it.
//
// role::Found<W, Driver, Addr, Where, Unit>:
//   W       the discover::World (or anything with a static `reg` registry and `static void route(RowId bus)`)
//   Driver  the device's driver type: which found() binds it. Typed access, static: `set(addr, unit, v)`, `get(addr, unit)`, `top` for a
//           Light, `name()` for the description. No virtual call is added.
//   Addr    its address on its bus
//   Where   discover's own bus predicate (discover::Anywhere, discover::Behind<Bridge, Ch>): the same type a Use<> entry can name
//   Unit    which output of the device (a PCA9685 channel)
// Roles on one device share one binding (role::Device: 2 bytes). Its identity is those types, checked once, when discovery finds a Driver: DriverBase::found() calls W::bind<Impl>(row), and role::Machine
// is a binder (wants<Impl>, bind, unbind, release: discover::BinderSet's shape). Only the roles whose Driver is Impl are compiled into
// Impl's found(): no search, no path parse. A request is a liveness check of the row, W::route to its bus, and the write at a constant
// address. Not found, gone, or released: live() is false, role::Machine does not apply the command, the report says live=false.
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/entries.h>
#include <oneMachine/state/face.h>

namespace role {
  template<class D, class = void> struct TopOf { static constexpr uint16_t value = 0; };   // a Light on a driver without `top` fails its own check
  template<class D> struct TopOf<D, std::void_t<decltype(D::top)>> { static constexpr auto value = D::top; };

  template<class P> void put_hex8(P& put, uint8_t v) { put('0'); put('x'); put(char(v >> 4 < 10 ? '0' + (v >> 4) : 'a' + (v >> 4) - 10)); put(char((v & 15) < 10 ? '0' + (v & 15) : 'a' + (v & 15) - 10)); }
  template<class Wh> struct WhereText { template<class P> static void put(P&) {} };
  template<uint8_t B, uint8_t C> struct WhereText<discover::Behind<B, C>> {
    template<class P> static void put(P& p) { p(' '); p('b'); p('e'); p('h'); p('i'); p('n'); p('d'); p(' '); put_hex8(p, B); p('/'); state::put_dec(p, unsigned(C)); } };

  // one device, bound once: shared by every role on it (the roles differ by Unit only)
  template<class W, class Driver, uint8_t Addr, class Where>
  struct Device {
    static inline discover::RowId row = discover::noRow, bus = discover::noRow;
    template<class Impl> static constexpr bool wants = std::is_same<Impl, Driver>::value;
    ONEMACHINE_NOINLINE static void bind(discover::RowId r) {
      const auto& x = W::reg.rows[r]; if (x.busId == Addr && Where::template holds<W>(x.parent)) { row = r; bus = x.parent; } }
    static void unbind() { row = bus = discover::noRow; }
    static void release(discover::RowId r) { if (row == r) unbind(); }
    static bool live() { return row != discover::noRow && W::reg.status(row) == discover::Status::Alive; }
  };

  template<class W, class Driver, uint8_t Addr, class Where, uint8_t Unit>
  struct Found {
    using Dev = Device<W, Driver, Addr, Where>;
    static constexpr auto top = TopOf<Driver>::value;

    template<class Impl> static constexpr bool wants = Dev::template wants<Impl>;
    template<class Impl> static void bind(discover::RowId r) { if constexpr (wants<Impl>) Dev::bind(r); }
    static void unbind() { Dev::unbind(); }
    static void release(discover::RowId r) { Dev::release(r); }
    static bool live() { return Dev::live(); }
    template<class V> static void set(V v) { W::route(Dev::bus); Driver::set(Addr, Unit, v); }
    static auto get() { W::route(Dev::bus); return Driver::get(Addr, Unit); }
    template<class P> static void where(P& put) {        // "pca9685 0x40 behind 0x70/2 #0", " (not found)" while unbound
      state::put_name(put, Driver::name()); put(' '); put_hex8(put, Addr); WhereText<Where>::put(put);
      put(' '); put('#'); state::put_dec(put, unsigned(Unit));
      if (Dev::row == discover::noRow) { const char* t = " (not found)"; while (*t) put(*t++); }
    }
  };
}
