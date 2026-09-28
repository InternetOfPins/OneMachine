// State a driver declares, and where it lives.
//   DeviceState  member type of the driver: one slot per row (the row's device), sized to the largest declared.
//   ClientState  member type of the driver: one per binding consumer (shell or class handle).
//   init(row)    static member of the driver: called from found() after the row and its slot exist.
// Nothing declared -> NoSlot, an empty type, and no storage.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include "capability.h"

namespace discover {

  struct NoSlot {};

  template<typename D, typename = void> struct DeviceStateOf { static constexpr bool has = false; };
  template<typename D> struct DeviceStateOf<D, std::void_t<typename D::DeviceState>> {
    static constexpr bool has = true; using Type = typename D::DeviceState;
  };
  template<typename D, typename = void> struct ClientStateOf { static constexpr bool has = false; };
  template<typename D> struct ClientStateOf<D, std::void_t<typename D::ClientState>> {
    static constexpr bool has = true; using Type = typename D::ClientState;
  };
  template<typename D, typename = void> struct HasInit : std::false_type {};
  template<typename D> struct HasInit<D, std::void_t<decltype(D::init(RowId{}))>> : std::true_type {};

  // predicates for Filter (the ListHas shape)
  struct DeclaresDevice {
    template<typename O> using Check = typename hapi::Traverse<DeclaresDevice,O>::Beta;
    template<typename O> using Apply = std::bool_constant<DeviceStateOf<O>::has>;
    template<typename... OO> using ApplyPack = hapi::Chain<OO...>;
  };
  struct DeclaresClient {
    template<typename O> using Check = typename hapi::Traverse<DeclaresClient,O>::Beta;
    template<typename O> using Apply = std::bool_constant<ClientStateOf<O>::has>;
    template<typename... OO> using ApplyPack = hapi::Chain<OO...>;
  };
  template<typename D> using DevTypeOf = typename DeviceStateOf<D>::Type;
  template<typename D> using CliTypeOf = typename ClientStateOf<D>::Type;

  // typed storage for "one of these": a union, so access stays typed and the size is the largest member
  template<typename T, typename... R> union Slot {
    T head; Slot<R...> tail;
    template<typename U> U& as() {
      if constexpr (std::is_same<T, U>::value) return head; else return tail.template as<U>();
    }
  };
  template<typename T> union Slot<T> {
    T head;
    template<typename U> U& as() { static_assert(std::is_same<T, U>::value, "not a declared state type"); return head; }
  };
  template<typename... T> struct SlotSel { using Type = Slot<T...>; };
  template<> struct SlotSel<> { using Type = NoSlot; };

  // the slot type for the drivers of a list that declare a DeviceState / ClientState
  template<typename L> using DevSlotOf =
    typename hapi::Eval<hapi::Filter<DeclaresDevice>, L>::template Map<DevTypeOf>::template Build<SlotSel>::Type;
  template<typename L> using CliSlotOf =
    typename hapi::Eval<hapi::Filter<DeclaresClient>, L>::template Map<CliTypeOf>::template Build<SlotSel>::Type;

  template<typename S> inline void zeroBytes(S& s) {
    auto* p = reinterpret_cast<uint8_t*>(&s);
    for (__SIZE_TYPE__ i = 0; i < sizeof(S); ++i) p[i] = 0;
  }

  // one slot per registry row; nothing at all when no driver declares a DeviceState
  template<typename S, uint8_t N, bool Empty = __is_empty(S)> struct Table;
  template<typename S, uint8_t N> struct Table<S, N, true> {
    static void clear(RowId) {}
    static void clearAll() {}
  };
  template<typename S, uint8_t N> struct Table<S, N, false> {
    inline static S rows[N + 1] = {};          // the last slot is scratch: a row past the table lands there
    static S& at(RowId r) { return rows[r < N ? r : N]; }
    static void clear(RowId r) { zeroBytes(at(r)); }
    static void clearAll() { for (RowId r = 0; r <= N; ++r) clear(r); }
  };

  // what a consumer holds for its ClientState: an empty base when there is none
  template<typename S> struct Held {
    S cs{};
    void resetClient() { zeroBytes(cs); }
  };
  template<> struct Held<NoSlot> { void resetClient() {} };

  // a direct consumer's ClientState: one static cell per (consumer, driver type). Named only where used, so a shell
  // whose driver declares none costs nothing, and instantiating the shell does not instantiate the driver.
  template<typename Owner, typename Iface> struct ClientCell { inline static typename ClientStateOf<Iface>::Type v{}; };

}
