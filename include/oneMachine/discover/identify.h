// Identification as a composition of entries. The list given to World is a list of entries:
//   Driver               a bare driver: its own addrLo..addrHi and id, one-stage (SLA+W, pointer byte, SLA+R)
//   Use<Probe, Driver>   a row for Driver at the addresses Probe accepts (Use<Own, D>: D's addrLo..addrHi, its idReg (0 if none) equal to its id)
//   Claim<Probe>         an address taken without a row (Ignore<Lo,Hi> is the unconditional one)
//   Protect<Lo,Hi>       a rule: no entry that writes to the device may cover the range
// The first entry that accepts an address wins; bridge entries are tried before the others; a Claim takes part in both.
// A probe is a type: `lo..hi` and `Wh` (which buses), `presence` (an ACK first, once per address and pass),
// `certain` (accepts every address that passes stage 1), `writes`, and `test<W,D>(addr, bus)` (stage 2).
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneBus/twiMaster.h>
#include "entries.h"

namespace discover {

  // ---- probes: real bus traffic (entries.h's Use/Claim/Protect/Own/Norm/DriversIn carry no bus calls) -----------
  // stage 1 only: whatever acknowledges the address is the device
  template<uint8_t Lo, uint8_t Hi = Lo, typename Where = Anywhere>
  struct AddressProbe {
    static constexpr uint8_t lo = Lo, hi = Hi;
    using Wh = Where;
    static constexpr bool presence = true, readOnly = false, certain = true, writes = false;
    template<typename W, typename D> static bool test(uint8_t, RowId) { return true; }
  };

  // stage 2: the pointer byte Reg is written, then one byte read must equal Val
  template<uint8_t Reg, uint8_t Val, uint8_t Lo, uint8_t Hi = Lo, typename Where = Anywhere>
  struct IdProbe {
    static constexpr uint8_t lo = Lo, hi = Hi;
    using Wh = Where;
    static constexpr bool presence = true, readOnly = false, certain = false, writes = true;
    template<typename W, typename D> static bool test(uint8_t addr, RowId) {
      using Twi = typename W::Twi;
      Twi::begin_write(addr); Twi::write_byte(Reg); Twi::end_write();
      (void)Twi::request_from(addr, 1);
      return Twi::read_byte() == Val;
    }
  };

  // stage 1 as a read-probe and one read: nothing is ever written to the device
  template<uint8_t Lo, uint8_t Hi, uint8_t Val, typename Where = Anywhere>
  struct ReadProbe {
    static constexpr uint8_t lo = Lo, hi = Hi;
    using Wh = Where;
    static constexpr bool presence = true, readOnly = true, certain = false, writes = false;
    template<typename W, typename D> static bool test(uint8_t addr, RowId) {
      using Twi = typename W::Twi;
      (void)Twi::request_from(addr, 1);
      return Twi::read_byte() == Val;
    }
  };

  // a runtime table: Tag::has(addr)
  template<typename Tag, typename Where = Anywhere>
  struct InSet {
    static constexpr uint8_t lo = 0x08, hi = 0x77;
    using Wh = Where;
    static constexpr bool presence = false, readOnly = false, certain = false, writes = false;
    template<typename W, typename D> static bool test(uint8_t addr, RowId) { return Tag::has(addr); }
  };

  // Use<Own, D>: entries.h's generic Norm wraps a bare driver in LegacyProbe; Own asks for D's own idReg checked
  // in two stages instead -- the one place the generic entry shape reaches into a real probe.
  template<typename D> struct Norm<Use<Own, D>, 1> { using Type = Use<IdProbe<IdRegOf<D>::value, D::id, D::addrLo, D::addrHi>, D>; };

  // ---- the fold: one pass over the address, entries in list order ---------------------------------------
  struct Seen { int8_t known[2] = {-1, -1}; };   // stage-1 answer per probe kind, for one address in one pass

  // the normalized entries of a pass: the row entries of that pass and every claim
  template<uint8_t Pass, typename Acc, typename... E> struct KeepPass;
  template<uint8_t Pass, typename... K> struct KeepPass<Pass, hapi::Chain<K...>> { using Type = hapi::Chain<K...>; };
  template<uint8_t Pass, typename... K, typename E, typename... R> struct KeepPass<Pass, hapi::Chain<K...>, E, R...> {
    static constexpr uint8_t p = PassOf<E>::value;
    using Type = typename KeepPass<Pass, std::conditional_t<p == Pass || p == 2, hapi::Chain<K..., typename Norm<E>::Type>, hapi::Chain<K...>>, R...>::Type;
  };
  template<uint8_t Pass, typename L> struct PassEntries;
  template<uint8_t Pass, typename... E> struct PassEntries<Pass, hapi::Chain<E...>> { using Type = typename KeepPass<Pass, hapi::Chain<>, E...>::Type; };

  template<typename W>
  struct Identify {
    template<typename L> struct Run;
    template<typename... E> struct Run<hapi::Chain<E...>> {
      static bool at([[maybe_unused]] uint8_t addr, [[maybe_unused]] RowId bus) { [[maybe_unused]] Seen seen; return (tryEntry<E>(addr, bus, seen) || ...); }
    };

    // first entry that accepts the address; a Use creates the row, a Claim only ends the search
    template<bool Bridges, typename Entries> using Pass = Run<typename PassEntries<Bridges ? 0 : 1, Entries>::Type>;

  private:
    template<typename E> static bool tryEntry(uint8_t addr, [[maybe_unused]] RowId bus, [[maybe_unused]] Seen& seen) {
      using P = typename E::Probe;
      if (addr < P::lo || addr > P::hi) return false;
      if constexpr (!std::is_same<typename P::Wh, Anywhere>::value) { if (!P::Wh::template holds<W>(bus)) return false; }
      if constexpr (P::presence) { if (!present(addr, seen, P::readOnly)) return false; }
      if constexpr (KindOf<E>::value == 1) {
        using Dr = typename E::Driver;
        static_assert(std::is_same<typename Dr::Self, Dr>::value,
                      "driver must derive from DriverBase<itself,W>: a listed driver derived from another driver would probe, register and poll as its base");
        if constexpr (IsOwnProbe<P>::value) { if (!Dr::probe(addr)) return false; }
        else { if (!P::template test<W, Dr>(addr, bus)) return false; }
        Dr::found(addr, bus);
        return true;
      } else {
        if (!P::template test<W, void>(addr, bus)) return false;
        if constexpr (!std::is_void<typename ClearedOf<E>::Type>::value) ClearedOf<E>::Type::clear(addr);
        return true;
      }
    }

    // stage 1 through oneBus::probe: a read-probe where probeKindFor says so (0x30-0x37, 0x50-0x5F) or the entry asks for one
    static bool present(uint8_t addr, Seen& seen, bool readOnly) {
      const oneBus::ProbeKind k = readOnly ? oneBus::ProbeKind::Read : oneBus::probeKindFor(addr);
      int8_t& m = seen.known[uint8_t(k)];
      if (m < 0) {
        using Twi = typename W::Twi;
        bool ok = oneBus::probe<Twi>(addr, k);
        // a bus fault (a timeout: the interface starts over after it) is the bus's, not the address's: try once more before calling it absent
        if (!ok && oneBus::isBusFault(oneBus::causeOf<Twi>())) ok = oneBus::probe<Twi>(addr, k);
        m = ok ? 1 : 0;
      }
      return m == 1;
    }
  };

  // ---- rules ---------------------------------------------------------------------------------------
  template<typename A, typename B> struct WhereMeets : std::bool_constant<std::is_same<typename A::Wh, Anywhere>::value || std::is_same<typename B::Wh, Anywhere>::value || std::is_same<typename A::Wh, typename B::Wh>::value> {};
  template<typename A, typename B> struct Overlaps : std::bool_constant<A::lo <= B::hi && B::lo <= A::hi && WhereMeets<A, B>::value> {};
  template<typename A, typename B> struct Covers   : std::bool_constant<A::lo <= B::lo && B::hi <= A::hi && (std::is_same<typename A::Wh, Anywhere>::value || std::is_same<typename A::Wh, typename B::Wh>::value)> {};

  // A is listed before B; the bridge pass runs before the other, so a bridge entry that is listed later still comes first
  template<typename A, typename B, bool Swap> struct Ordered { using First = A; using Second = B; };
  template<typename A, typename B> struct Ordered<A, B, true> { using First = B; using Second = A; };

  template<typename RA, typename RB>
  struct PairOk {
    static constexpr uint8_t pa = PassOf<RA>::value, pb = PassOf<RB>::value;
    static constexpr bool swap = pa < 2 && pb < 2 && pa > pb;
    using First  = typename Ordered<RA, RB, swap>::First;
    using Second = typename Ordered<RA, RB, swap>::Second;
    using PF = typename First::Probe;
    using PS = typename Second::Probe;
    static constexpr bool bothPins = KindOf<First>::value == 1 && KindOf<Second>::value == 1 && PF::certain && PS::certain;

    static_assert(!(bothPins && Overlaps<PF, PS>::value),
                  "an address is pinned by two entries: the later one can never see it");
    static_assert(!(KindOf<First>::value == 2 && PF::certain && Covers<PF, PS>::value),
                  "a claim without a row covers every address of a later entry: that entry can never match");
    static_assert(!(!bothPins && KindOf<First>::value == 1 && PF::certain && Covers<PF, PS>::value),
                  "a pin covers every address of a later entry: that entry can never match");
    static constexpr bool value = true;
  };

  template<typename R, typename... P> struct ProtectOk : std::true_type {};
  template<typename R, uint8_t Lo, uint8_t Hi, typename... P> struct ProtectOk<R, Protect<Lo, Hi>, P...> {
    static_assert(!(R::Probe::writes && R::Probe::lo <= Hi && Lo <= R::Probe::hi),
                  "an entry that writes to the device covers an address of a Protect range: use AddressProbe or ReadProbe there");
    static constexpr bool value = ProtectOk<R, P...>::value;
  };
  template<typename R, typename H, typename... P> struct ProtectOk<R, H, P...> : ProtectOk<R, P...> {};

  // Protect entries are rules: they take no part in the ordering pairs and are not themselves writers
  template<typename A, typename B, bool Skip = (KindOf<A>::value == 3 || KindOf<B>::value == 3)> struct PairGuard : PairOk<A, B> {};
  template<typename A, typename B> struct PairGuard<A, B, true> : std::true_type {};
  template<typename R, bool Skip, typename... P> struct ProtectGuard : ProtectOk<R, P...> {};
  template<typename R, typename... P> struct ProtectGuard<R, true, P...> : std::true_type {};

  template<typename... E> struct Rules;
  template<> struct Rules<> { static constexpr bool value = true; };
  template<typename H, typename... T> struct Rules<H, T...> {
    static constexpr bool value = (PairGuard<H, T>::value && ...) && Rules<T...>::value;
  };

  template<typename L, typename Protects> struct EntryRules;
  template<typename... E, typename... P> struct EntryRules<hapi::Chain<E...>, hapi::Chain<P...>> {
    static constexpr bool value = (ProtectGuard<E, KindOf<E>::value == 3, P...>::value && ...);
  };

  template<typename... E> struct RepeatFree : std::true_type {};
  template<typename A, typename... R> struct RepeatFree<A, R...>
    : std::bool_constant<(!std::is_same<A, R>::value && ...) && RepeatFree<R...>::value> {};

  // the Protect entries of a list
  template<typename Acc, typename... E> struct ProtectsAcc;
  template<typename... P> struct ProtectsAcc<hapi::Chain<P...>> { using Type = hapi::Chain<P...>; };
  template<typename... P, typename E, typename... R> struct ProtectsAcc<hapi::Chain<P...>, E, R...> {
    using Type = typename ProtectsAcc<std::conditional_t<KindOf<E>::value == 3, hapi::Chain<P..., E>, hapi::Chain<P...>>, R...>::Type;
  };

  // an IgnoreBridge<Bridge> says Bridge is not used, only cleared: it must not also be a driver the entries actually use
  template<typename Bridge, typename... E> struct NotAlsoUsed : std::true_type {};
  template<typename Bridge, typename H, typename... T> struct NotAlsoUsed<Bridge, H, T...>
    : std::bool_constant<!std::is_same<Bridge, typename DriverOf_<H>::Type>::value && NotAlsoUsed<Bridge, T...>::value> {};
  template<typename E, typename... All> struct IgnoreBridgeOk : std::true_type {};
  template<typename E, typename... All> struct CheckIgnoreBridge {
    static_assert(std::is_void<typename ClearedOf<E>::Type>::value || NotAlsoUsed<typename ClearedOf<E>::Type, All...>::value,
                  "IgnoreBridge names a bridge the entries also use elsewhere: state one meaning, not both");
    static constexpr bool value = true;
  };

  template<typename L> struct EntriesCheck;
  template<typename... E> struct EntriesCheck<hapi::Chain<E...>> {
    static_assert(RepeatFree<E...>::value, "driver list has a repeated type: an entry is listed twice");
    using N = hapi::Chain<typename Norm<E>::Type...>;
    static constexpr bool pairs      = Rules<typename Norm<E>::Type...>::value;
    static constexpr bool protects   = EntryRules<N, typename ProtectsAcc<hapi::Chain<>, E...>::Type>::value;
    static constexpr bool notIgnored = (CheckIgnoreBridge<E, E...>::value && ...);
    static constexpr bool value      = pairs && protects && notIgnored;
  };

  // The I2C scan fold, given to World<> as its Scan parameter (registry.h names no bus protocol of its own): the
  // conflict rules once, then the two-stage 7-bit address scan (bridges first, so a stale selection cannot show
  // devices behind it as if they sat on this bus; then everything else). Self::claimedUpstream is the registry's.
  struct I2cScan {
    template<typename Self, typename Drivers> static void run(RowId bus) {
      [[maybe_unused]] constexpr bool entriesOk = EntriesCheck<Drivers>::value;   // each conflict rule is a static_assert with its own message
      using Ident = Identify<Self>;
      for (uint8_t a = 0x08; a <= 0x77; ++a)
        if (!Self::claimedUpstream(bus, a)) Ident::template Pass<true, Drivers>::at(a, bus);
      for (uint8_t a = 0x08; a <= 0x77; ++a)
        if (!Self::claimedUpstream(bus, a)) Ident::template Pass<false, Drivers>::at(a, bus);
    }
  };

}
