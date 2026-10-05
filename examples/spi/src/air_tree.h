// What the spi example publishes: the air sensor's codes and published nodes (the reader's are rfid_tree.h's), the wiring, and the description of both
// machines, in one place. The sketch (main.cpp), the simulated device (test/link/tree_device.cpp), the AVR image (test/link/avr_tree.cpp) and the
// description generator (examples/spi/describe.cpp) all build them from here, so the description the generator writes is the one the firmware's hash is
// computed from.
//   M      the machine (bmpm::Machine<W, Criteria, Mode>)
//   Note   Note<Code>::fn: a constexpr pointer to what a published value calls when it changes (the App's: print it, mark it for the link, ...)
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <chips/esp8266/esp8266Pins.h>
#include <oneMachine/discover/manifest.h>
#include "bmp280_machine.h"
#include "rc522.h"
#include "rfid_tree.h"

namespace airTree {

  // the codes, numbered by their position in the description (the link's change records use the numbers)
  struct CodeTemp     { static constexpr uint8_t num = 0; ONEMACHINE_STATE_NAME(name, "temp"); };
  struct CodePress    { static constexpr uint8_t num = 1; ONEMACHINE_STATE_NAME(name, "press"); };
  struct CodeAir      { static constexpr uint8_t num = 2; ONEMACHINE_STATE_NAME(name, "air"); };
  struct CodeConfig   { static constexpr uint8_t num = 3; ONEMACHINE_STATE_NAME(name, "air/config"); };
  struct CodeCtrlMeas { static constexpr uint8_t num = 4; ONEMACHINE_STATE_NAME(name, "air/ctrl_meas"); };
  using rfidTree::CodeCard;   // 5, then rfid/gain 6 and rfid/antenna 7

  // ---- the wiring: the rig's pins, the one place the sketch takes them from (experiment 6) ------------------------------------------------
  // The board's pins and their facts are OneChip's (Esp8266Pins); who drives each line is the drivers' manifests' (rc522::Manifest, bmpm::Manifest)
  // and the buses' (discover::I2cLines, SpiLines). The rules below are the wiring check at compile time; python/onemachine/wiring.py checks a
  // wiring spec by the same facts, which describe.cpp prints from these types.
#ifdef AIRTREE_WIRING
  #include AIRTREE_WIRING    // a Wiring emitted from a wiring spec (python3 -m onemachine.wiring emit): the rules below judge it as they judge this one
#else
  struct Wiring {
    using Board = hw::esp8266::Esp8266Pins;
    static constexpr const char* board = "wemos-d1-mini";
    static constexpr uint8_t sda = Board::D2, scl = Board::D1;                      // I2C
    static constexpr uint8_t sck = Board::D5, miso = Board::D6, mosi = Board::D7;  // SPI (the ESP8266's HSPI pins)
    static constexpr uint8_t rfidCs = Board::D8, emptyCs = Board::D3;              // the SPI slots: the reader, and one declared with nothing on it
    static constexpr uint8_t rfidRst = Board::D4, rfidIrq = Board::D0;
    static constexpr uint8_t airAddr = 0x76;
  };
#endif
  // a line that the part drives (or shares) must not be on a pin the chip samples at reset; the MCU's own lines may be (D3 and D8 as chip selects)
  template<uint8_t G, discover::Drive D> constexpr bool line() {
    using B = Wiring::Board;
    static_assert(B::gpio(G) && !B::flash(G), "wiring: not a GPIO of this board (GPIO6..GPIO11 are the flash)");
    static_assert(discover::safeAtReset<B>(G, D), "wiring: a line the part drives is on a boot strap pin (GPIO0, GPIO2 or GPIO15: D3, D4, D8): "
                  "the part can hold it at reset (the RC522 keeps a pending IRQ across a reset of the board) and the ESP8266 boots into ROM mode");
    return true;
  }
  static_assert(line<Wiring::sda, discover::I2cLines::sda>() && line<Wiring::scl, discover::I2cLines::scl>());
  static_assert(line<Wiring::sck, discover::SpiLines::sck>() && line<Wiring::miso, discover::SpiLines::miso>() && line<Wiring::mosi, discover::SpiLines::mosi>());
  static_assert(line<Wiring::rfidCs, rc522::Manifest::Pins::cs>() && line<Wiring::emptyCs, discover::SpiLines::cs>());
  static_assert(line<Wiring::rfidRst, rc522::Manifest::Pins::rst>() && line<Wiring::rfidIrq, rc522::Manifest::Pins::irq>());

  // the sensor: the machine at the wiring's address (its Criteria), under the App W's failure handling Mode
  template<typename W, typename Mode = bmpm::Plain> using Machine = bmpm::Machine<W, bmpm::Addr<Wiring::airAddr>, Mode>;
  constexpr uint8_t bus = 1;   // the air sensor's bus is the App's second machine: its path codes start with 1

  // temp and press notify on sync; air is the control group, config and ctrl_meas its registers (silent), each by its path in the machine
  template<typename M, template<typename> class Note> using Pubs = hapi::Chain<
    bmpm::PublishedAt<CodeTemp,     bmpm::PathRef<M, 0>,    oneData::OnSync<Note<CodeTemp>::fn>>,
    bmpm::PublishedAt<CodePress,    bmpm::PathRef<M, 1>,    oneData::OnSync<Note<CodePress>::fn>>,
    bmpm::PublishedAt<CodeAir,      bmpm::PathRef<M, 3>>,
    bmpm::PublishedAt<CodeConfig,   bmpm::PathRef<M, 3, 0>>,
    bmpm::PublishedAt<CodeCtrlMeas, bmpm::PathRef<M, 3, 1>>>;

  // ---- the wiring lines of the description (the build output's, by hash: they cost the device nothing) -----------------------------------
  //   wiring wemos-d1-mini
  //     i2c sda 4 shared scl 5 mcu
  //     spi sck 14 mcu miso 12 device mosi 13 mcu
  //     rfid rc522 cs 15 mcu rst 2 mcu irq 16 device
  //     empty cs 0 mcu
  //     air bmp280 at 0x76
  // a part's lines: the GPIO the wiring gives each one, and who drives it (its manifest)
  template<typename P> struct WiringPut {
    P& put;
    constexpr void str(const char* s) { while (*s) put(*s++); }
    constexpr void dec(unsigned v) { char b[4] = {}; unsigned k = 0; do { b[k++] = char('0' + v % 10); v /= 10; } while (v); while (k) put(b[--k]); }
    constexpr void hex(uint8_t v) { const uint8_t h = uint8_t(v >> 4), l = uint8_t(v & 15); str("0x"); put(char(h < 10 ? '0' + h : 'A' + h - 10)); put(char(l < 10 ? '0' + l : 'A' + l - 10)); }
    constexpr void pin(const char* role, uint8_t g, discover::Drive d) { put(' '); str(role); put(' '); dec(g); put(' '); str(discover::driveName(d)); }
  };
  template<typename P> constexpr void wiringLines(P& put) {
    using W = Wiring;
    WiringPut<P> w{put};
    w.str("wiring "); w.str(W::board); put('\n');
    w.str("  i2c"); w.pin("sda", W::sda, discover::I2cLines::sda); w.pin("scl", W::scl, discover::I2cLines::scl); put('\n');
    w.str("  spi"); w.pin("sck", W::sck, discover::SpiLines::sck); w.pin("miso", W::miso, discover::SpiLines::miso); w.pin("mosi", W::mosi, discover::SpiLines::mosi); put('\n');
    w.str("  rfid "); w.str(rc522::Manifest::name);
    using RP = rc522::Manifest::Pins;
    w.pin("cs", W::rfidCs, RP::cs); w.pin("rst", W::rfidRst, RP::rst); w.pin("irq", W::rfidIrq, RP::irq);
    put('\n');
    w.str("  empty"); w.pin("cs", W::emptyCs, discover::SpiLines::cs); put('\n');
    w.str("  air "); w.str(bmpm::Manifest::name); w.str(" at "); w.hex(W::airAddr); put('\n');
  }
  // The App's tree: both machines and what is published of them (the air sensor's codes, then the reader's), then the wiring. Pubs is the list the link's
  // codes are numbered by. describe is the text with the status of each row (the text build); describeStatic is the same without them and with the wiring,
  // a compile-time fold (the description's hash and the build output).
  //   BM, BP  the air sensor's machine and its published nodes     RM, RP  the reader's
  template<typename BM, typename BP, typename RM, typename RP>
  struct Tree {
    using Pubs = typename hapi::ConcatChains<BP, RP>::Type;
    template<typename P> static void describe(P& put) {
      bmpm::Walk<P> b{put}; rc522m::Walk<P> r{put};
      b.template machine<BM>(); r.template machine<RM>();
      b.str("published\n");
      b.template publishedAll<BM>(static_cast<BP*>(nullptr), bus);
      r.template publishedAll<RM>(static_cast<RP*>(nullptr), rfidTree::bus);
    }
    template<typename P> static constexpr void describeStatic(P& put) {
      bmpm::HashWalk<P> b{put}; rc522m::HashWalk<P> r{put};
      b.template machine<BM>(); r.template machine<RM>();
      b.str("published\n");
      b.template publishedAll<BM>(static_cast<BP*>(nullptr), bus);
      r.template publishedAll<RM>(static_cast<RP*>(nullptr), rfidTree::bus);
      wiringLines(put);
    }
  };
}
