// Failure state for the rows of one kind, in as many slots as that kind can have (K), not one per registry row.
//   SlotTable<Stack, K, Rank>: slot i belongs to the i-th row of its kind, in table order. Rank::of(row) says which, or noSlot.
//   A row of the kind past the K-th has no slot: its operation runs as it is (no retry, no status), and `overflows` counts it.
//   An empty stack stores nothing and serve() is the operation.
//   _serve(row, id) is the return path at the top: the row's outcome, or, for a row past the K-th, Fail(Overflow) once (detail = K).
//   Report: the return path is on, so the table keeps the bits that say which overflowing rows _serve() has already reported.
// The rank is computed from the registry on each use, so nothing needs to be told about a discovery except reset().
#pragma once
#include <stdint.h>
#include <oneMachine/discover/registry.h>
#include "layers.h"

namespace fail {

  inline constexpr uint8_t noSlot = 0xFF;

  // the row is the slot (a table whose rows are not registry rows: an outbox, a consumer's controller)
  template<uint8_t K> struct SlotIsRow { static uint8_t of(RowId r) { return r < K ? r : noSlot; } };

  // the rows whose driver is Dr
  template<typename W, typename Dr> struct DriverRank {
    static uint8_t of(RowId row) {
      const discover::IDriver* d = discover::instOf<Dr>();
      if (row >= W::reg.count || W::reg.rows[row].drv != d) return noSlot;
      uint8_t k = 0;
      for (RowId r = 0; r < row; ++r)
        if (W::reg.rows[r].drv == d) ++k;
      return k;
    }
  };

  // the bus rows (the root and each bridge channel)
  template<typename W> struct BusRank {
    static uint8_t of(RowId row) {
      if (row >= W::reg.count || !W::reg.rows[row].isBus) return noSlot;
      uint8_t k = 0;
      for (RowId r = 0; r < row; ++r) if (W::reg.rows[r].isBus) ++k;
      return k;
    }
  };

  // the rows whose overflow _serve() has reported (a bit per row, row mod 16): kept only where the return path is on
  template<bool On, typename Tag> struct ReportedBits { inline static uint16_t bits = 0; };
  template<typename Tag> struct ReportedBits<false, Tag> {};

  template<typename S, uint8_t K, typename Rank, bool Report = false, bool Empty = __is_empty(S)> struct SlotTable;

  template<typename S, uint8_t K, typename Rank, bool Report> struct SlotTable<S, K, Rank, Report, true> {
    template<typename F> static void serve(RowId, Cause, F&& op) { (void)op(); }
    static void tick(RowId, uint32_t) {}
    static FailStatus status(RowId) { return {}; }
    static bool ownStale(RowId) { return false; }
    static Outcome _serve(RowId, Id = 0) { return Outcome::Idle(); }
    static uint8_t overflowCount() { return 0; }
    static void reset() {}
  };

  template<typename S, uint8_t K, typename Rank, bool Report> struct SlotTable<S, K, Rank, Report, false> {
    static_assert(K > 0, "a table needs at least one slot");
    inline static S       slots[K] = {};                 // all-zero: .bss
    inline static uint8_t overflows = 0;                 // operations that found their row without a slot (saturating)

    static S* get(RowId row) {
      const uint8_t i = Rank::of(row);
      return i < K ? &slots[i] : nullptr;
    }
    template<typename F> static void serve(RowId row, Cause c, F&& op) {
      if (S* s = get(row)) { s->serve(row, c, op); return; }
      if (overflows != 0xFF) ++overflows;
      (void)op();
    }
    static void tick(RowId row, uint32_t now) { if (S* s = get(row)) { s->bind(row); s->tick(now); } }
    static Outcome _serve(RowId row, Id id = 0) {
      if (S* s = get(row)) return s->_serve(row, id);
      const uint8_t i = Rank::of(row);
      if (i == noSlot) return Outcome::Idle();                              // not a row of this kind
      if constexpr (Report) {
        using R = ReportedBits<Report, S>;
        const uint16_t b = uint16_t(1u << (row & 15));
        if (R::bits & b) return Outcome::Idle();
        R::bits = uint16_t(R::bits | b);
        return Outcome::Fail(Kind::Overflow, K);                            // the row has no slot: say so, once
      } else return Outcome::Idle();                                        // no return path: the counter is the only report
    }
    static FailStatus status(RowId row) { const S* s = get(row); return s ? s->status() : FailStatus{}; }
    static bool ownStale(RowId row) { const S* s = get(row); return s && s->ownStale(); }
    static uint8_t overflowCount() { return overflows; }
    static void reset() { for (uint8_t i = 0; i < K; ++i) slots[i] = S{}; overflows = 0; if constexpr (Report) ReportedBits<Report, S>::bits = 0; }
  };

}
