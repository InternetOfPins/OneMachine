// SPI discovery: the Scan a World is given when its bus is an SPI bus with statically declared chip selects
// (oneBus::SpiSlots). SPI has no address and no acknowledge, so the runtime question is narrower than on I2C:
// which declared slot holds which device, if any. A slot is identified by reading an ID register.
//
//   using Bus = hapi::APIOf<oneBus::SpiAPI, oneBus::SpiSlots<CsA, CsB, CsC>, oneBus::SpiMaster<4000000>, Core>;
//   using Entries = Chain<Rc522<App>, Bmx280Spi<App>, Fixed<2, Dac<App>>>;
//   struct App : World<App, Bus, Consumers, Entries, 4, SpiScan, Chain<SpiSlotIds<Bus::slots>>> {};
//
// Entries, tried in list order on every slot:
//   D            an SPI driver (SpiDriverBase): its idCmd is sent with the slot selected, the byte after it must be one
//                of D::Ids, twice in a row; the bus is set to D's own clock and mode first
//   Fixed<S, D>  a row for D on slot S without any traffic: a device with no ID register. A Fixed slot is never
//                probed by any other entry: claim with Fixed a slot whose chip would take another driver's probe as a write.
// An empty slot reads the idle MISO level (0x00 or 0xFF) and a floating one reads noise: an ID set may hold neither
// value (a compile error), and a match must repeat.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include "registry.h"

namespace discover {

  // World's Buses entry for an SPI bus: a row's busId is its slot
  template<uint8_t Slots> struct SpiSlotIds {
    static_assert(Slots > 0, "an SPI bus with no slots");
    static constexpr uint32_t idMax = Slots - 1;
  };

  // the values an ID register may hold for one driver (clones of a chip often report their own)
  template<uint8_t... V> struct IdSet {
    static_assert(sizeof...(V) > 0, "IdSet: at least one id");
    static_assert(((V != 0x00 && V != 0xFF) && ...),
                  "an SPI id of 0x00 or 0xFF is what an empty slot reads: it cannot identify a device");
    static constexpr bool has(uint8_t v) { return ((v == V) || ...); }
    static constexpr uint8_t list[] = {V...};          // in order, for a manifest
    static constexpr unsigned count = sizeof...(V);
  };

  template<uint8_t S> struct AtSlot { static constexpr uint8_t slot = S; };
  template<uint8_t S, typename D> struct Fixed { static constexpr uint8_t entryKind = 1; using Probe = AtSlot<S>; using Driver = D; };

  template<typename E> struct IsFixed : std::false_type {};
  template<uint8_t S, typename D> struct IsFixed<Fixed<S, D>> : std::true_type {};

  // the bus as the drivers see it: one setup per change of clock or mode, not one per exchange
  template<typename Bus> struct SpiCfg {
    inline static uint32_t hz   = 0;
    inline static uint8_t  mode = 0xFF;
    static void use(uint32_t h, uint8_t m) {
      if (h == hz && m == mode) return;
      hz = h; mode = m; Bus::setup(h, m);
    }
  };

  // An SPI driver: DriverBase + spiHz, spiMode (0-3), idCmd and Ids (none needed when the driver is only ever Fixed).
  // A row's busId is its slot. xfer() selects the row's slot for the whole exchange, at the driver's own clock and mode.
  template<typename Impl, typename W>
  struct SpiDriverBase : DriverBase<Impl, W> {
    static uint8_t slotOf(RowId row) { return uint8_t(W::reg.rows[row].busId); }
    static void xfer(RowId row, const uint8_t* tx, uint8_t* rx, uint16_t n) {
      SpiCfg<typename W::Bus>::use(Impl::spiHz, Impl::spiMode);
      W::Bus::xfer(slotOf(row), tx, rx, n);
    }
  };

  template<typename E> struct SpiEntryOk {
    static_assert(KindOf<E>::value == 0 || IsFixed<E>::value,
                  "an SPI entry is an SPI driver or Fixed<Slot, Driver>: Use/Claim/Protect are I2C address entries");
    static constexpr bool value = true;
  };

  // the slot a Fixed entry claims; 0xFF for any other entry
  template<typename X> constexpr uint8_t fixedSlotOf() {
    if constexpr (IsFixed<X>::value) return X::Probe::slot; else return 0xFF;
  }

  template<typename... E> struct FixedDistinct : std::true_type {};
  template<typename A, typename... R> struct FixedDistinct<A, R...>
    : std::bool_constant<((!IsFixed<A>::value || fixedSlotOf<A>() != fixedSlotOf<R>()) && ...) && FixedDistinct<R...>::value> {};

  // the byte a driver's ID command reads from a slot, at the driver's own clock and mode; the scan and the failure edge's reprobe both read it this way
  template<typename Bus, typename X> uint8_t spiIdByte(uint8_t slot) {
    SpiCfg<Bus>::use(X::spiHz, X::spiMode);
    uint8_t io[2] = {X::idCmd, 0};
    Bus::xfer(slot, io, io, 2);
    return io[1];
  }

  struct SpiScan {
    template<typename Self, typename Entries> static void run(RowId bus) {
      if (bus != rootRow) return;   // an SPI bus has no bridges: one root, its slots
      Run<Self, Entries>::all(bus);
    }
    // the slot of a row that went Gone, looked at again by the same entries
    template<typename Self, typename Entries> static void revive(RowId bus, uint8_t slot) { Run<Self, Entries>::one(slot, bus); }

  private:
    template<typename Self, typename L> struct Run;
    template<typename Self, typename... E> struct Run<Self, hapi::Chain<E...>> {
      static_assert((SpiEntryOk<E>::value && ...), "");
      static_assert(FixedDistinct<E...>::value, "two Fixed entries name the same slot");

      static void all(RowId bus) {
        using Bus = typename Self::Bus;
        static_assert(((!IsFixed<E>::value || fixedSlotOf<E>() < Bus::slots) && ...),
                      "a Fixed slot past the bus's last chip select");
        for (uint8_t slot = 0; slot < Bus::slots; ++slot) {
          const bool fixed = ((fixedSlotOf<E>() == slot) || ...);
          (void)(tryEntry<E>(slot, bus, fixed) || ...);
        }
      }

      static void one(uint8_t slot, RowId bus) {
        const bool fixed = ((fixedSlotOf<E>() == slot) || ...);
        (void)(tryEntry<E>(slot, bus, fixed) || ...);
      }

      template<typename X> static bool tryEntry(uint8_t slot, RowId bus, bool fixed) {
        if constexpr (IsFixed<X>::value) {
          if (slot != X::Probe::slot) return false;
          X::Driver::found(slot, bus);
          return true;
        } else {
          if (fixed) return false;
          static_assert(std::is_same<typename X::Self, X>::value,
                        "driver must derive from SpiDriverBase<itself,W>: a listed driver derived from another driver would probe, register and poll as its base");
          const int16_t a = readId<X>(slot), b = readId<X>(slot);
          if (a < 0 || a != b) return false;
          X::found(slot, bus);
          return true;
        }
      }

      template<typename X> static int16_t readId(uint8_t slot) {
        const uint8_t b = spiIdByte<typename Self::Bus, X>(slot);
        return X::Ids::has(b) ? int16_t(b) : int16_t(-1);
      }
    };
  };

}
