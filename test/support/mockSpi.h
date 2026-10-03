// A simulated SPI bus behind oneBus::SpiSlots, for the SPI discovery tests. Four slots, each with a chip select pin
// (MockCs<K>) and whatever sits behind it:
//   Rc522    a register model of an MFRC522: VersionReg, the FIFO, a SoftReset that lands late, RST held low (hold(): reads 0x00,
//            writes ignored, register defaults on release), Transceive with a card in the field (WUPA -> ATQA,
//            cascade-1 anticollision -> UID + BCC), the READY quirk (a card left READY ignores the next wake-up), a
//            corrupt BCC and a collision on request
//   Bmx      a BMP280/BME280 in SPI mode: register 0xD0 reads its chip id, bit 7 of the first byte means read
//   Plain    a device with no ID register (it answers 0x5A to everything): only a Fixed entry can give it a row
//   Empty    nothing: MISO reads `idle` (0xFF or 0x00), or, when `floating`, noise that lands on a valid id once
//            per probe (the RC522's 0x92, then the BMP280's 0x58) and on something else the next time
// `stuckLow` holds MISO at 0 for every slot, whatever is selected (a shorted line).
// The core records the clock and mode in force at each byte, per slot, and counts the bytes each slot saw.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneBus/spi.h>

namespace mspi {

  enum class Kind : uint8_t { Empty, Rc522, Bmx, Plain };

  struct Card { bool present; uint8_t uid[4]; bool badBcc, collide; bool ready; uint8_t mute = 0; };   // mute: wake-ups left to ignore

  struct Rc522Model {
    uint8_t regs[64];
    uint8_t fifo[16]; uint8_t n;
    uint16_t resetReads;  // CommandReg reads PowerDown this many times after a SoftReset (the oscillator starting:
                          // tens of ms on a real part, thousands of SPI reads)
    uint16_t lateBy = 30; // a SoftReset written takes effect this many accesses later (each read or write counts); the
                          // configuration written in between is wiped when it does, as on a real part
    uint16_t pending = 0; // accesses left before the SoftReset applies; 0: none waiting
    uint8_t deaf;         // then this many accesses while the oscillator starts: writes are lost, reads give the
                          // reset value of CommandReg (0x20, PowerDown clear) and 0 elsewhere -- no flag says it is over
    void reset(uint8_t version) {
      for (auto& r : regs) r = 0;
      regs[0x37] = version; regs[0x14] = 0x80; n = 0; resetReads = 0; deaf = 0; pending = 0; held = false;
    }
    bool noStore = false; // the ID answers and nothing written is kept (a part living off its signal pins)
    bool held = false;    // RST low: the part reads 0x00 and ignores writes; released, it comes up at its register defaults (a hard reset, not a SoftReset)
    void hold(bool on) {
      if (on) { held = true; pending = 0; return; }
      if (!held) return;
      held = false;
      const uint8_t ver = regs[0x37];
      reset(ver);
    }
    void tick() {
      if (pending && --pending == 0) { const uint8_t ver = regs[0x37]; reset(ver); resetReads = 3000; deaf = 40; }
    }
    uint8_t read(uint8_t reg) {
      if (held) return 0;
      tick();
      if (deaf) { --deaf; return reg == 0x01 ? 0x20 : 0; }
      if (reg == 0x09) { if (!n) return 0; const uint8_t v = fifo[0]; for (uint8_t i = 1; i < n; ++i) fifo[i - 1] = fifo[i]; --n; return v; }
      if (reg == 0x0A) return n;
      if (reg == 0x01 && resetReads) { --resetReads; return 0x10; }
      return regs[reg & 0x3F];
    }
    void push(uint8_t b) { if (n < 16) fifo[n++] = b; }
    void write(uint8_t reg, uint8_t v, Card& card) {
      if (held || noStore) return;
      tick();
      if (deaf) { --deaf; return; }
      reg &= 0x3F;
      switch (reg) {
        case 0x09: push(v); return;
        case 0x0A: if (v & 0x80) n = 0; return;
        case 0x04: if (v & 0x80) regs[reg] |= (v & 0x7F); else regs[reg] &= uint8_t(~v); return;   // Set1 bit
        case 0x01: regs[reg] = v; if (v == 0x0F) pending = lateBy ? lateBy : 1; return;
        case 0x0D:
          regs[reg] = v;
          if ((v & 0x80) && regs[0x01] == 0x0C) transceive(card);
          return;
        default: regs[reg] = v; return;
      }
    }
    void transceive(Card& card) {
      const uint8_t len = n; uint8_t in[16]; for (uint8_t i = 0; i < len; ++i) in[i] = fifo[i];
      n = 0; regs[0x06] = 0;
      const bool antenna = (regs[0x14] & 0x03) == 0x03;
      if (antenna && card.present && len == 1 && (in[0] == 0x52 || in[0] == 0x26)) {
        if (card.mute) { --card.mute; regs[0x04] |= 0x01; return; }   // a held card that does not answer this wake-up
        if (card.ready) { card.ready = false; regs[0x04] |= 0x01; return; }   // READY + an unexpected command: back to IDLE, silent
        card.ready = true; push(0x04); push(0x00); regs[0x04] |= 0x30; return;
      }
      if (antenna && card.present && card.ready && len == 2 && in[0] == 0x93 && in[1] == 0x20) {
        if (card.collide) { regs[0x06] = 0x08; regs[0x04] |= 0x30; return; }
        uint8_t bcc = uint8_t(card.uid[0] ^ card.uid[1] ^ card.uid[2] ^ card.uid[3]);
        if (card.badBcc) bcc ^= 0x01;
        for (uint8_t b : card.uid) push(b);
        push(bcc); regs[0x04] |= 0x30; return;
      }
      regs[0x04] |= 0x01;   // TimerIRq: nothing answered
    }
  };

  struct State {
    static constexpr uint8_t slots = 4;
    inline static Kind     kind[slots] = {};
    inline static uint8_t  bmxId = 0x58;
    inline static uint8_t  idle = 0xFF;
    inline static bool     floating = false, stuckLow = false;
    inline static uint8_t  noise = 0;
    inline static Rc522Model rc{};
    inline static Card     card{};

    inline static int8_t   sel = -1;      // the selected slot; -1 none, -2 more than one (a fault)
    inline static uint8_t  phase = 0, addr = 0;
    inline static bool     rd = false;

    inline static uint32_t hz = 0;
    inline static uint8_t  mode = 0;
    inline static uint32_t bytes[slots] = {};
    inline static uint8_t  modeAt[slots] = {};   // the mode in force at a slot's last byte
    inline static uint32_t setups = 0;

    static void reset() {
      for (uint8_t k = 0; k < slots; ++k) { kind[k] = Kind::Empty; bytes[k] = 0; modeAt[k] = 0xFF; }
      bmxId = 0x58; idle = 0xFF; floating = stuckLow = false; noise = 0;
      rc.reset(0x92); card = Card{};
      sel = -1; phase = 0; setups = 0; hz = 0; mode = 0;
    }
    static void clearCounts() { for (auto& b : bytes) b = 0; setups = 0; }

    static void cs(uint8_t k, bool low) {
      if (low) { sel = (sel == -1) ? int8_t(k) : int8_t(-2); phase = 0; }
      else if (sel == int8_t(k)) sel = -1;
    }

    static uint8_t exchange(uint8_t b) {
      if (sel < 0) return stuckLow ? 0 : idle;
      const uint8_t k = uint8_t(sel);
      ++bytes[k]; modeAt[k] = mode;
      uint8_t out = idle;
      switch (kind[k]) {
        case Kind::Empty: {
          static constexpr uint8_t walk[8] = {0x00, 0x92, 0x00, 0x41, 0x00, 0x58, 0x00, 0x07};
          out = floating ? walk[noise++ & 7] : idle;
          break;
        }
        case Kind::Plain: out = 0x5A; break;
        case Kind::Bmx:
          if (phase == 0) { addr = b & 0x7F; rd = b & 0x80; out = 0; }
          else { out = (rd && addr == 0x50) ? bmxId : 0; ++addr; }   // 0xD0 with bit 7 cleared is 0x50
          break;
        case Kind::Rc522:
          if (phase == 0) { addr = uint8_t((b >> 1) & 0x3F); rd = b & 0x80; out = 0; }
          else if (rd) { out = rc.read(addr); addr = uint8_t((b >> 1) & 0x3F); }
          else { rc.write(addr, b, card); out = 0; }
          break;
      }
      ++phase;
      return stuckLow ? 0 : out;
    }
  };

  template<uint8_t K> struct MockCs {
    static void begin() {}
    static void on()  { State::cs(K, false); }
    static void off() { State::cs(K, true); }
  };

  struct Core {
    template<typename O>
    struct Part : O {
      static void spi_init(uint32_t h) { State::hz = h; }
      static void spi_setup(uint32_t h, uint8_t m) { State::hz = h; State::mode = m; ++State::setups; }
      [[nodiscard]] static uint8_t spi_transfer(uint8_t b) { return State::exchange(b); }
      static void begin() { O::begin(); }
    };
  };

  using Bus = hapi::APIOf<oneBus::SpiAPI, oneBus::SpiSlots<MockCs<0>, MockCs<1>, MockCs<2>, MockCs<3>>,
                          oneBus::SpiMaster<4000000>, Core>;

}
