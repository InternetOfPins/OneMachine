// DeviceState placement, sizes only: one slot per row sized to the largest (built, state.h) against a pool per driver type
// with a row -> slot map (not built: derived here from sizeof of the same types). Native program; the sizes are the
// AVR sizes for these types (no padding on AVR, so byte counts are the same or smaller there).
#include <stdio.h>
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/discover/state.h>

struct Tiny  { bool on; };                  // 1 B
struct Small { int16_t offset; };           // 2 B
struct Big   { int32_t a, b; };             // 8 B

template<typename T> struct Decl { using DeviceState = T; };
struct D1 : Decl<Tiny> {};
struct D2 : Decl<Small> {};
struct D3 : Decl<Big> {};
struct D0 {};

template<typename L> constexpr size_t slotBytes() { return sizeof(discover::DevSlotOf<L>) * (__is_empty(discover::DevSlotOf<L>) ? 0 : 1); }

// one row of the table: rows N, then per driver type "capacity" c_t (instances that may exist at once)
static void row(const char* what, unsigned N, size_t slot, unsigned c1, unsigned c2, unsigned c3, size_t s1, size_t s2, size_t s3) {
  const size_t perRow = N * slot;
  const size_t pool   = c1 * s1 + c2 * s2 + c3 * s3 + N * 1;      // one byte per row for the row -> slot map
  printf("  %-46s N=%-3u  per-row %4zu B   pool %4zu B (%s)\n", what, N, perRow, pool, pool < perRow ? "pool smaller" : "per-row smaller or equal");
}

int main() {
  using L3 = hapi::Chain<D0, D1, D2>;                 // R3's shape: a 1 B and a 2 B state
  using L4 = hapi::Chain<D0, D1, D2, D3>;             // plus one 8 B state
  using L0 = hapi::Chain<D0>;
  printf("slot type sizes (union of the declared DeviceStates): none=%zu  {1,2}=%zu  {1,2,8}=%zu\n", slotBytes<L0>(), slotBytes<L3>(), slotBytes<L4>());
  row("R3 app: display 1 B x1, sensors 2 B x2 of 10 rows", 10, slotBytes<L3>(), 1, 2, 0, 1, 2, 0);
  row("same, capacity of every type = N (undeclared)",     10, slotBytes<L3>(), 10, 10, 0, 1, 2, 0);
  row("mostly small: 1 B x2, 2 B x2, one 8 B x1",          16, slotBytes<L4>(), 2, 2, 1, 1, 2, 8);
  row("mostly small, many rows",                           64, slotBytes<L4>(), 2, 2, 1, 1, 2, 8);
  row("all rows stateful, same size",                      16, slotBytes<hapi::Chain<D2>>(), 0, 16, 0, 1, 2, 0);
  return 0;
}
