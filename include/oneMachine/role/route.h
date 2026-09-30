#pragma once
// oneMachine/role/route.h -- a device path as one number: the busIds from the root bus down (a bridge address, its channel, then the device
// address), hashed with FNV-1a, root first, one byte per level. role::path(0x70, 2, 0x40) at compile time, pathOf(reg, row) at run time,
// findPath(reg, p) to find the row again after a rescan (rows are renumbered by every rescan, a path is not).
//
// role::Pinned<W, Path, Driver, Unit> is a role endpoint at a path: pin() (role::Machine::pin(), after every discover() and reprobe) finds
// the row, a request is a liveness check, W::route to its bus and the write (docs/role.md: about 80 cycles per role per request over a
// fixed endpoint on an ATmega328P, and a table walk per role per pin()). role/found.h's Found is the default: it binds when discovery
// finds the device, with no search; Pinned is for a location that is data rather than a type (a run-time role profile, a path received
// in a machine-to-machine exchange).
#include <oneMachine/discover/registry.h>
#include <oneMachine/state/face.h>

namespace role {
  constexpr uint32_t pathStart = 2166136261u;
  constexpr uint32_t path() { return pathStart; }
  template<class... BB> constexpr uint32_t path(uint8_t first, BB... rest) {   // root-first: path(muxChannel, address)
    uint32_t h = state::fnv(pathStart, first);
    ((h = state::fnv(h, uint8_t(rest))), ...);
    return h;
  }
  template<class Reg> uint32_t pathOf(const Reg& reg, discover::RowId r) {
    if (r == discover::rootRow || r >= reg.count) return pathStart;
    return state::fnv(pathOf(reg, reg.rows[r].parent), uint8_t(reg.rows[r].busId));
  }
  template<class Reg> discover::RowId findPath(const Reg& reg, uint32_t p) {
    for (discover::RowId r = 1; r < reg.count; r++) if (!reg.rows[r].isBus && pathOf(reg, r) == p) return r;
    return discover::noRow;
  }

  template<class W, uint32_t Path, class Driver, uint8_t Unit>
  struct Pinned {
    static inline discover::RowId row = discover::noRow;
    static inline uint8_t addr = 0;
    static constexpr auto top = Driver::top;

    static void pin() { row = findPath(W::reg, Path); if (row != discover::noRow) addr = uint8_t(W::reg.rows[row].busId); }
    static bool live() { return row != discover::noRow && W::reg.status(row) == discover::Status::Alive; }
    template<class V> static void set(V v) { W::route(W::reg.rows[row].parent); Driver::set(addr, Unit, v); }
    static auto get() { W::route(W::reg.rows[row].parent); return Driver::get(addr, Unit); }
    template<class P> static void where(P& put) {        // "pca9685 path 1a2b3c4d #3", " (not found)" while unpinned
      state::put_name(put, Driver::name()); const char* t = " path "; while (*t) put(*t++);
      for (int i = 28; i >= 0; i -= 4) { uint8_t d = uint8_t((Path >> i) & 15); put(char(d < 10 ? '0' + d : 'a' + d - 10)); }
      put(' '); put('#'); state::put_dec(put, unsigned(Unit));
      if (row == discover::noRow) { t = " (not found)"; while (*t) put(*t++); }
    }
  };
}
