// The entry vocabulary the registry itself needs, with no bus protocol in it: a list given to World is a list of
// entries (a bare driver, or one of these three), and the registry reduces that list to its distinct drivers.
//   Use<Probe, Driver>   a row for Driver at the addresses Probe accepts (Use<Own, D>: D's addrLo..addrHi, identify.h's
//                        two-stage probe reads D's idReg against D::id)
//   Claim<Probe>         an address taken without a row (Ignore<Lo,Hi> is the unconditional one)
//   Protect<Lo,Hi>       a rule: no entry that writes to the device may cover the range
// What a Probe is (`lo..hi`, `Wh`, `presence`, `certain`, `writes`, `test<W,D>(addr,bus)`) and every real probe
// (AddressProbe, IdProbe, ReadProbe, ...) and the scan itself are identify.h's: they call the bus, this does not.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include "driver.h"

namespace discover {

  // ---- which buses (a probe's Wh) ------------------------------------------------------------------
  struct Anywhere { template<typename W> static bool holds(RowId) { return true; } };

  // the bus behind channel Ch of the bridge at address Bridge
  template<uint8_t Bridge, uint8_t Ch>
  struct Behind {
    template<typename W> static bool holds(RowId bus) {
      const auto& b = W::reg.rows[bus];
      return b.isBus && b.parent < W::reg.count && b.busId == Ch && W::reg.rows[b.parent].busId == Bridge;
    }
  };

  // no traffic: accepts every address of the range (Claim<AnyAt<..>> is Ignore; also usable directly)
  template<uint8_t Lo, uint8_t Hi = Lo, typename Where = Anywhere>
  struct AnyAt {
    static constexpr uint8_t lo = Lo, hi = Hi;
    using Wh = Where;
    static constexpr bool presence = false, readOnly = false, certain = true, writes = false;
    template<typename W, typename D> static bool test(uint8_t, RowId) { return true; }
  };

  // a bare driver's own probe: identify.h's Norm wraps a driver with no explicit entry in one of these
  template<typename D>
  struct LegacyProbe {
    static constexpr uint8_t lo = D::addrLo, hi = D::addrHi;
    using Wh = Anywhere;
    static constexpr bool presence = false, readOnly = false, certain = false, writes = true, own = true;
  };

  template<typename P, typename = void> struct IsOwnProbe : std::false_type {};
  template<typename P> struct IsOwnProbe<P, std::void_t<decltype(P::own)>> : std::true_type {};

  // ---- entries --------------------------------------------------------------------------------------
  struct Own {};   // in Use<Own, D>: IdProbe over D's addrLo..addrHi and id, in two stages (identify.h)

  template<typename P, typename D> struct Use   { static constexpr uint8_t entryKind = 1; using Probe = P; using Driver = D; };
  template<typename P>             struct Claim { static constexpr uint8_t entryKind = 2; using Probe = P; };
  template<uint8_t Lo, uint8_t Hi = Lo, typename Where = Anywhere>
  struct Ignore : Claim<AnyAt<Lo, Hi, Where>> {};    // not a bridge, never touch: an address the scan skips and nothing else does either

  // a bridge whose channels this app never looks behind: the scan skips its address, but clears it once claimed, so a
  // selection left over from before (identify.h's own standing note: an ignored real mux is never cleared, a stale
  // channel selection then shows its devices on the root bus) does not linger. Bridge is a real bridge driver type
  // (isBridge, ::clear(addr)); identify.h's EntriesCheck rejects listing the same Bridge again elsewhere in the entries.
  template<typename Bridge, typename Where = Anywhere>
  struct IgnoreBridge : Claim<AnyAt<Bridge::addrLo, Bridge::addrHi, Where>> { using Cleared = Bridge; };

  template<typename E, typename = void> struct ClearedOf { using Type = void; };
  template<typename E> struct ClearedOf<E, std::void_t<typename E::Cleared>> { using Type = typename E::Cleared; };
  template<uint8_t Lo, uint8_t Hi = Lo>
  struct Protect { static constexpr uint8_t entryKind = 3; static constexpr uint8_t lo = Lo, hi = Hi; };

  template<typename E, typename = void> struct KindOf { static constexpr uint8_t value = 0; };   // a bare driver
  template<typename E> struct KindOf<E, std::void_t<decltype(E::entryKind)>> { static constexpr uint8_t value = E::entryKind; };

  // the entry as it runs: a bare driver becomes Use<LegacyProbe<D>, D>; identify.h adds the Use<Own,D> specialization
  // (it needs IdProbe, a real probe -- the one place this generic shape reaches into the bus-specific module).
  template<typename E, uint8_t K = KindOf<E>::value> struct Norm { using Type = Use<LegacyProbe<E>, E>; };
  template<typename E> struct Norm<E, 1> { using Type = E; };
  template<typename E> struct Norm<E, 2> { using Type = E; };
  template<typename E> struct Norm<E, 3> { using Type = E; };

  // the register that holds a device's identity: the driver's `idReg`, register 0 when it declares none
  template<typename D, typename = void> struct IdRegOf { static constexpr uint8_t value = 0; };
  template<typename D> struct IdRegOf<D, std::void_t<decltype(D::idReg)>> { static constexpr uint8_t value = D::idReg; };

  // the driver an entry names, if any
  template<typename E, uint8_t K = KindOf<E>::value> struct DriverOf_ { using Type = E; };
  template<typename E> struct DriverOf_<E, 1> { using Type = typename E::Driver; };
  template<typename E> struct DriverOf_<E, 2> { using Type = void; };
  template<typename E> struct DriverOf_<E, 3> { using Type = void; };

  template<typename D, typename... L> struct Has : std::false_type {};
  template<typename D, typename H, typename... L> struct Has<D, H, L...> : std::bool_constant<std::is_same<D, H>::value || Has<D, L...>::value> {};

  template<typename D, typename L> struct HasIn;
  template<typename D, typename... L> struct HasIn<D, hapi::Chain<L...>> : Has<D, L...> {};

  // the distinct drivers the entries name, in order: the list the rest of World works on
  template<typename Acc, typename... E> struct DriversAcc;
  template<typename... D> struct DriversAcc<hapi::Chain<D...>> { using Type = hapi::Chain<D...>; };
  template<typename... D, typename E, typename... R> struct DriversAcc<hapi::Chain<D...>, E, R...> {
    using Dr   = typename DriverOf_<E>::Type;
    using Next = std::conditional_t<std::is_void<Dr>::value || Has<Dr, D...>::value, hapi::Chain<D...>, hapi::Chain<D..., Dr>>;
    using Type = typename DriversAcc<Next, R...>::Type;
  };
  template<typename L> struct DriversOfEntries;
  template<typename... E> struct DriversOfEntries<hapi::Chain<E...>> { using Type = typename DriversAcc<hapi::Chain<>, E...>::Type; };
  template<typename Entries> using DriversIn = typename DriversOfEntries<Entries>::Type;

  // 0: bridge entry, 1: any other row entry, 2: a claim (both passes), 3: a rule (none) -- identify.h's pass order
  template<typename E, uint8_t K = KindOf<E>::value> struct PassOf { static constexpr uint8_t value = E::isBridge ? 0 : 1; };
  template<typename E> struct PassOf<E, 1> { static constexpr uint8_t value = E::Driver::isBridge ? 0 : 1; };
  template<typename E> struct PassOf<E, 2> { static constexpr uint8_t value = 2; };
  template<typename E> struct PassOf<E, 3> { static constexpr uint8_t value = 3; };

}
