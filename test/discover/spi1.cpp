// SPI discovery, round 1: a bus with statically declared chip selects (oneBus::SpiSlots), the runtime question being
// which slot holds which device. Native only (the mock SPI bus is test/support/mockSpi.h):
//   - identification by ID register: the RC522 and a BMP280 are found on their slots, each at its own clock and mode
//   - an empty slot (idle MISO high or low, or floating noise) and a stuck-low MISO give no row
//   - a Fixed slot gets its row without one byte of traffic, and no other entry probes it
//   - the real RC522 driver (examples/spi/src/rc522.h) over the register model: init, a card's UID once on arrival,
//     0 on departure (after 3 polls without it), the READY quirk, a corrupt BCC and a collision counted, never emitted
#include <stdint.h>
#include <cstdio>
#include <hapi/hapi.h>
#include <oneMachine/discover/spi.h>
#include "../support/mockSpi.h"
#include "../../examples/spi/src/rc522.h"

using discover::RowId;
using discover::Sample;
using hapi::Chain;

struct Pressure { using Value = int32_t; static constexpr uint8_t id = 30, decimals = 0; static constexpr const char* name = "p"; };

// a BMP280/BME280 in SPI mode, identification only: mode 3 so the test sees the bus change mode for it
template<typename W>
struct Bmx280Spi : discover::SpiDriverBase<Bmx280Spi<W>, W> {
  using Produces = Chain<Pressure>;
  static constexpr uint32_t spiHz = 1000000;
  static constexpr uint8_t  spiMode = 3, idCmd = 0xD0;
  using Ids = discover::IdSet<0x58, 0x60>;
  static void read(RowId) {}
};

// a device with no ID register: only ever Fixed
template<typename W>
struct Plain : discover::SpiDriverBase<Plain<W>, W> {
  static constexpr uint32_t spiHz = 2000000;
  static constexpr uint8_t  spiMode = 0;
  static constexpr bool polled = true;
  inline static uint8_t last = 0;
  static void read(RowId row) { uint8_t io[1] = {0}; Plain::xfer(row, io, io, 1); last = io[0]; }
};

struct CardLog {
  using Accepts = Chain<rc522::Card>;
  inline static uint32_t log[8];
  inline static uint8_t n = 0;
  template<typename Cap> struct Body {
    template<typename T> struct Part : T {
      using T::T;
      void on(const Sample<Cap>& s) { if (n < 8) log[n++] = s.value; }
    };
  };
};

struct App;
using Rfid = rc522::Rc522<App>;
#ifdef NEG_ID_IDLE
using BadIds = discover::IdSet<0x58, 0xFF>;
#endif
#ifdef NEG_FIXED_TWICE
using Entries = Chain<Rfid, discover::Fixed<3, Bmx280Spi<App>>, discover::Fixed<3, Plain<App>>>;
#elif defined(NEG_FIXED_PAST)
using Entries = Chain<Rfid, Bmx280Spi<App>, discover::Fixed<4, Plain<App>>>;
#elif defined(NEG_I2C_ENTRY)
using Entries = Chain<Rfid, discover::Use<discover::Own, Bmx280Spi<App>>, discover::Fixed<3, Plain<App>>>;
#elif defined(NEG_DERIVED_DRIVER)
template<typename W> struct Rc522b : rc522::Rc522<W> {};
using Entries = Chain<Rc522b<App>, Bmx280Spi<App>, discover::Fixed<3, Plain<App>>>;
#else
using Entries = Chain<Rfid, Bmx280Spi<App>, discover::Fixed<3, Plain<App>>>;
#endif
struct App : discover::World<App, mspi::Bus, Chain<CardLog>, Entries, 6, discover::SpiScan, Chain<discover::SpiSlotIds<4>>> {};

static_assert(sizeof(App::BusIdT) == 1, "four slots -> 8-bit busId");
#ifdef NEG_ID_IDLE
static_assert(BadIds::has(0x58), "");
#endif
static_assert(mspi::Bus::slots == 4, "");

static int failures = 0;
#define CHECK(c) do { if (!(c)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

static RowId rowOn(uint8_t slot) {
  for (RowId r = 1; r < App::reg.count; ++r) if (App::reg.rows[r].busId == slot) return r;
  return discover::noRow;
}

int main() {
  using mspi::State; using mspi::Kind; using discover::instOf;

  // ---- identification -------------------------------------------------------------------------------
  State::reset();
  State::kind[0] = Kind::Rc522; State::kind[1] = Kind::Bmx; State::kind[2] = Kind::Empty; State::kind[3] = Kind::Plain;
  mspi::Bus::begin();
  CHECK(State::sel == -1);                         // every CS high after begin
  App::discover();
  CHECK(App::reg.count == 4);                      // root + three devices; slot 2 empty
  CHECK(rowOn(0) != discover::noRow && App::reg.rows[rowOn(0)].drv == instOf<Rfid>());
  CHECK(rowOn(1) != discover::noRow && App::reg.rows[rowOn(1)].drv == instOf<Bmx280Spi<App>>());
  CHECK(rowOn(2) == discover::noRow);
  CHECK(rowOn(3) != discover::noRow && App::reg.rows[rowOn(3)].drv == instOf<Plain<App>>());
  CHECK(State::bytes[3] == 0);                     // a Fixed slot is never probed
  CHECK(State::modeAt[1] == 3);                    // the BMP280 was read in its own mode
  CHECK(State::modeAt[0] == 0);                    // and the RC522 in its own
  CHECK((State::rc.regs[0x14] & 0x03) == 0x03);    // init ran: antenna on
  CHECK(State::rc.regs[0x2C] == 0x03 && State::rc.regs[0x2D] == 0xE8);
  CHECK(App::devState<Rfid>(rowOn(0)).initTries >= 1); // the configuration read back
  CHECK(State::sel == -1);                         // nothing left selected

  // the BME280 id and the clone RC522 ids are accepted too
  State::bmxId = 0x60; State::rc.regs[0x37] = 0x88;
  App::discover();
  CHECK(rowOn(0) != discover::noRow && rowOn(1) != discover::noRow);

  // an id the driver does not list is not that driver
  State::bmxId = 0x61; State::rc.regs[0x37] = 0x90;
  App::discover();
  CHECK(App::reg.count == 2 && rowOn(3) != discover::noRow);
  State::bmxId = 0x58; State::rc.regs[0x37] = 0x92;

  // ---- empty slots ------------------------------------------------------------------------------------
  for (uint8_t idle : {uint8_t(0xFF), uint8_t(0x00)}) {
    State::reset(); State::idle = idle;
    App::discover();
    CHECK(App::reg.count == 2);                    // only the Fixed row
  }
  // floating MISO: noise that hits an accepted id once does not repeat it
  State::reset(); State::floating = true;
  App::discover();
  CHECK(App::reg.count == 2);
  // a stuck-low MISO hides every device
  State::reset();
  State::kind[0] = Kind::Rc522; State::kind[1] = Kind::Bmx; State::stuckLow = true;
  App::discover();
  CHECK(App::reg.count == 2);

  // ---- the RC522 driver: cards ------------------------------------------------------------------------
  State::reset();
  State::kind[0] = Kind::Rc522; State::kind[3] = Kind::Plain;
  App::discover();
  const RowId rf = rowOn(0);
  CHECK(rf != discover::noRow);
  CardLog::n = 0;
  App::pump();
  CHECK(CardLog::n == 0);                          // no card, nothing emitted
  CHECK(Plain<App>::last == 0x5A);                 // the Fixed device is polled through its own slot

  State::card = mspi::Card{true, {0xDE, 0xAD, 0xBE, 0xEF}, false, false, false};
  App::pump();
  CHECK(CardLog::n == 1 && CardLog::log[0] == 0xDEADBEEFu);
  for (int i = 0; i < 4; ++i) App::pump();         // the card stays: READY after each anticollision, still seen, no repeat
  CHECK(CardLog::n == 1);
  CHECK(App::devState<Rfid>(rf).uid == 0xDEADBEEFu);

  // a held card that misses a poll (a poll is two wake-ups): no departure, and the streak restarts when it answers
  State::card.mute = 2;
  App::pump();
  CHECK(CardLog::n == 1 && App::devState<Rfid>(rf).missStreak == 1);
  App::pump();
  CHECK(CardLog::n == 1 && App::devState<Rfid>(rf).missStreak == 0);
  State::card.mute = 4;                            // two missed polls, then it answers
  App::pump(); App::pump(); App::pump();
  CHECK(CardLog::n == 1 && App::devState<Rfid>(rf).missStreak == 0);

  State::card.present = false;                     // the card left: 3 polls in a row without it
  App::pump(); App::pump();
  CHECK(CardLog::n == 1 && App::devState<Rfid>(rf).missStreak == 2);
  App::pump();
  CHECK(CardLog::n == 2 && CardLog::log[1] == 0 && App::devState<Rfid>(rf).missStreak == 0);
  App::pump();
  CHECK(CardLog::n == 2);                          // no card, no UID held: nothing more

  State::card = mspi::Card{true, {0x01, 0x02, 0x03, 0x04}, true, false, false};
  App::pump();
  CHECK(CardLog::n == 2);                          // a corrupt UID is never emitted
  CHECK(App::devState<Rfid>(rf).bccErrors == 1);

  State::card = mspi::Card{true, {0x01, 0x02, 0x03, 0x04}, false, true, false};
  App::pump();
  CHECK(CardLog::n == 2 && App::devState<Rfid>(rf).collisions == 1);

  State::card.collide = false;
  App::pump();
  CHECK(CardLog::n == 3 && CardLog::log[2] == 0x01020304u);

  // a mode change only when the next device needs another one
  State::clearCounts();
  App::pump(); App::pump();
  CHECK(State::setups <= 4);

  std::printf(failures ? "FAILED (%d)\n" : "OK: discover SPI round 1 native\n", failures);
  return failures != 0;
}
