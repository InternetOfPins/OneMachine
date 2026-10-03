// An MFRC522 (RC522 RFID reader) driver over SPI: found by its VersionReg, initialised at found() (soft reset, timer,
// 100% ASK, antenna on), polled for a card. A card that arrives emits its UID once; a card that leaves emits 0.
// Cascade level 1 only: a 4-byte UID is the card's whole UID, a 7-byte UID card shows as 0x88 and its first 3 bytes.
// Register access (datasheet 8.1.2.3): address byte (reg << 1) & 0x7E, with bit 7 set for a read.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/discover/spi.h>

namespace rc522 {

  using discover::RowId;

  // a card's UID, cascade level 1, first byte in the high byte; 0 when the card leaves
  struct Card { using Value = uint32_t; static constexpr uint8_t id = 20, decimals = 0; static constexpr const char* name = "card"; };

  enum Reg : uint8_t {
    CommandReg = 0x01, ComIrqReg = 0x04, ErrorReg = 0x06, FIFODataReg = 0x09, FIFOLevelReg = 0x0A,
    ControlReg = 0x0C, BitFramingReg = 0x0D, CollReg = 0x0E, ModeReg = 0x11, TxControlReg = 0x14, TxASKReg = 0x15,
    TModeReg = 0x2A, TPrescalerReg = 0x2B, TReloadRegH = 0x2C, TReloadRegL = 0x2D, VersionReg = 0x37,
  };
  enum Cmd : uint8_t { Idle = 0x00, Transceive = 0x0C, SoftReset = 0x0F };

  template<typename W>
  struct Rc522 : discover::SpiDriverBase<Rc522<W>, W> {
    using B = discover::SpiDriverBase<Rc522<W>, W>;
    using Produces = hapi::Chain<Card>;

    static constexpr uint32_t spiHz   = 4000000;   // the chip takes 10 MHz; 4 is kind to jumper wires
    static constexpr uint8_t  spiMode = 0;
    static constexpr uint8_t  idCmd   = uint8_t((VersionReg << 1) | 0x80);
    using Ids = discover::IdSet<0x91, 0x92, 0x88, 0x12, 0xB2>;   // v1.0, v2.0, then the common clones (FM17522 and others)

    // what the poll found: the card in the field and the errors seen on the way (a BCC mismatch is a corrupt UID)
    // initTries: on which write of the configuration it read back (0: it never did)
    struct DeviceState { uint32_t uid; uint16_t bccErrors, collisions, initTries; };

    static uint8_t rd(RowId row, uint8_t reg) {
      uint8_t io[2] = {uint8_t(((reg << 1) & 0x7E) | 0x80), 0};
      B::xfer(row, io, io, 2);
      return io[1];
    }
    static void wr(RowId row, uint8_t reg, uint8_t v) {
      uint8_t io[2] = {uint8_t((reg << 1) & 0x7E), v};
      B::xfer(row, io, nullptr, 2);
    }

    // the configuration init() writes, read back: only a chip whose oscillator runs keeps what is written
    static bool configured(RowId row) {
      return rd(row, TPrescalerReg) == 0xA9 && rd(row, TReloadRegL) == 0xE8 && (rd(row, TxControlReg) & 0x03) == 0x03;
    }

    // A SoftReset restarts the oscillator. PowerDown reads 1 until it runs, which takes tens of ms (the MFRC522
    // library allows 150): a bound in register reads, as there is no clock here, of 60000 reads (about 0.6 s at
    // 4 MHz on an ESP8266, when the part never wakes). Writes made before it runs are lost, the antenna stays off
    // and no card ever answers, and PowerDown may not show the whole window either: so the configuration is then
    // written until it reads back.
    static void init(RowId row) {
      wr(row, CommandReg, SoftReset);
      for (uint16_t i = 0; i < 60000 && (rd(row, CommandReg) & 0x10); ++i) {}
      for (uint16_t i = 0; i < 1000; ++i) {
        wr(row, TModeReg, 0x80);       // timer starts at the end of a transmission
        wr(row, TPrescalerReg, 0xA9);  // 40 kHz tick
        wr(row, TReloadRegH, 0x03);    // 1000 ticks: a 25 ms receive timeout
        wr(row, TReloadRegL, 0xE8);
        wr(row, TxASKReg, 0x40);       // 100% ASK
        wr(row, ModeReg, 0x3D);        // CRC preset 0x6363
        wr(row, TxControlReg, uint8_t(rd(row, TxControlReg) | 0x03));   // antenna on
        if (configured(row)) { B::dev(row).initTries = uint16_t(i + 1); return; }
      }
    }

    // one exchange with a card: FIFO in, Transceive, wait for the receive (or the timer), FIFO out.
    // Returns the bytes received, 0 when nothing answered, -1 on an error (collision, parity, protocol, overflow).
    static int8_t transceive(RowId row, const uint8_t* tx, uint8_t n, uint8_t* rx, uint8_t max, uint8_t lastBits) {
      wr(row, CommandReg, Idle);
      wr(row, ComIrqReg, 0x7F);             // clear every interrupt request bit
      wr(row, FIFOLevelReg, 0x80);          // flush the FIFO
      for (uint8_t i = 0; i < n; ++i) wr(row, FIFODataReg, tx[i]);
      wr(row, BitFramingReg, lastBits);
      wr(row, CommandReg, Transceive);
      wr(row, BitFramingReg, uint8_t(lastBits | 0x80));   // StartSend
      uint8_t irq = 0;
      for (uint16_t i = 0; i < 2000; ++i) { irq = rd(row, ComIrqReg); if (irq & 0x31) break; }   // RxIRq, IdleIRq, TimerIRq
      wr(row, BitFramingReg, 0x00);
      if (!(irq & 0x30)) return 0;          // the timer ran out (or the loop did): no card answered
      if (rd(row, ErrorReg) & 0x1B) return -1;   // BufferOvfl, CollErr, ParityErr, ProtocolErr
      uint8_t got = rd(row, FIFOLevelReg);
      if (got > max) got = max;
      for (uint8_t i = 0; i < got; ++i) rx[i] = rd(row, FIFODataReg);
      return int8_t(got);
    }

    // WUPA (7 bits): a card in the field answers ATQA. A card left READY by an anticollision without a SELECT ignores
    // the first wake-up and falls back to IDLE, so a silent first try gets one more.
    static bool present(RowId row) {
      const uint8_t wupa = 0x52;
      uint8_t atqa[2];
      for (uint8_t t = 0; t < 2; ++t) if (transceive(row, &wupa, 1, atqa, 2, 0x07) == 2) return true;
      return false;
    }

    static void read(RowId row) {
      auto& st = B::dev(row);
      uint32_t uid = 0;
      if (present(row)) {
        const uint8_t anticoll[2] = {0x93, 0x20};   // cascade level 1, no UID bits known
        uint8_t r[5];
        const int8_t got = transceive(row, anticoll, 2, r, 5, 0x00);
        if (got < 0) { ++st.collisions; return; }
        if (got != 5) return;
        if (uint8_t(r[0] ^ r[1] ^ r[2] ^ r[3]) != r[4]) { ++st.bccErrors; return; }
        uid = (uint32_t(r[0]) << 24) | (uint32_t(r[1]) << 16) | (uint32_t(r[2]) << 8) | r[3];
      }
      if (uid != st.uid) { st.uid = uid; B::template emit<Card>(row, uid); }
    }
  };

}
