// An MFRC522 (RC522 RFID reader) driver over SPI: found by its VersionReg, initialised at found() (soft reset, timer,
// 100% ASK, antenna on), polled for a card. A card that arrives emits its UID once; a card that leaves emits 0.
// Cascade level 1 only: a 4-byte UID is the card's whole UID, a 7-byte UID card shows as 0x88 and its first 3 bytes.
// Register access (datasheet 8.1.2.3): address byte (reg << 1) & 0x7E, with bit 7 set for a read.
// It takes part in failure handling (fail::DevEdge over the SPI access policy): each poll first reads the configuration back, which
// a chip that was reset without the host knowing has lost (Corrupt: the configuration is written again); a chip that answers no ID is
// Absent (retried, then Stale, probed until it answers: its configuration is written again when it does). A card held when the row
// goes Stale is reported as gone.
#pragma once
#include <stdint.h>
#include <hapi/hapi.h>
#include <oneMachine/discover/spi.h>
#include <oneMachine/fail/devedge.h>
#include <oneMachine/fail/spiaccess.h>
#include <oneMachine/fail/health.h>

namespace rc522 {

  using discover::RowId;

  // a card's UID, cascade level 1, first byte in the high byte; 0 when the card leaves
  struct Card { using Value = uint32_t; static constexpr uint8_t id = 20, decimals = 0; static constexpr const char* name = "card"; };

  enum Reg : uint8_t {
    CommandReg = 0x01, ComIEnReg = 0x02, DivIEnReg = 0x03, ComIrqReg = 0x04, ErrorReg = 0x06, FIFODataReg = 0x09, FIFOLevelReg = 0x0A,
    ControlReg = 0x0C, BitFramingReg = 0x0D, CollReg = 0x0E, ModeReg = 0x11, TxControlReg = 0x14, TxASKReg = 0x15,
    TModeReg = 0x2A, TPrescalerReg = 0x2B, TReloadRegH = 0x2C, TReloadRegL = 0x2D, VersionReg = 0x37,
  };
  enum Cmd : uint8_t { Idle = 0x00, Transceive = 0x0C, SoftReset = 0x0F };

  // The health monitor's thresholds for this row. A reader that is half powered flaps Stale/Alive about once a second; against the monitor's 500 ms
  // decay that settles near 80 on the 0..256 flap scale, under the default entry threshold of 96: it would be reported but never quarantined. One
  // isolated fault adds 32 and decays away.
  struct HealthCfg : fail::DefaultHealthCfg {
    static constexpr uint16_t enterQ = 64, exitQ = 32;
    static_assert(enterQ > exitQ, "Health: enter threshold must exceed exit threshold (hysteresis)");
  };

  // ---- the RC522's interrupt part ------------------------------------------------------------------------------
  // A mode that has `using Irq = rc522::Interrupt<Delivery[, Observer[, Check[, Fallback]]]>` makes a command wait on the IRQ line instead of
  // polling ComIrqReg: the poll starts the command and returns, and service() finishes it when the line is asserted, or after Interrupt::timeoutMs.
  // The register values (what is enabled while a command runs, the pin driven push-pull) belong to the chip and are here; the App chooses the pin
  // and how the line reaches the loop, and which of the optional parts it wants:
  //   Delivery   begin(), arm() (before a command starts), ready() (the line is asserted)
  //   Observer   seen(irq, line, outcome): rig diagnostics, after every command (default NoObserver)
  //   Check      the line against the register after a command, as a device failure (LineCheck; default NoCheck: none)
  //   Fallback   what a line fault does to the row (PollOnLineFault: it polls the register from then on; default NoFallback: the fault is the
  //              chip's as far as the failure edge is concerned, and Recover initialises it again)
  // The requests are enabled only while a command runs and cleared after it: the chip keeps its state across a reset of the host, and an
  // enabled pending request would hold the line low at the next boot (a boot strapping pin must be high then).
  struct NoObserver { static void seen(uint8_t, bool, const fail::Outcome&) {} };

  // what the ComIrqReg bits mean for a Transceive: RxIRq or IdleIRq, a card answered; TimerIRq alone, nobody did
  struct IrqMeaning {
    static constexpr uint8_t rxIrq = 0x20, idleIrq = 0x10, timerIrq = 0x01;
    static constexpr bool answered(uint8_t irq) { return (irq & (rxIrq | idleIrq)) != 0; }
    static constexpr bool finished(uint8_t irq) { return (irq & (rxIrq | idleIrq | timerIrq)) != 0; }
  };

  struct NoCheck {
    static constexpr bool on = false;
    static fail::Outcome of(uint8_t, bool) { return fail::Outcome::Ok(); }
  };

  // The line against the register after a command, as a device failure: Fault with detail 1 when the line is asserted and the register shows no
  // request that was enabled (spurious), 2 when it shows one and the line never was (the line is not connected, or not driven); Timeout when
  // neither (the chip's own timer did not end the command).
  struct LineCheck {
    static constexpr bool on = true;
    static constexpr uint8_t spurious = 1, missed = 2;
    static constexpr bool isLineFault(const fail::Outcome& o) { return o.failed() && o.kind() == fail::Kind::Fault && (o.detail == spurious || o.detail == missed); }
    static fail::Outcome of(uint8_t irq, bool line) {
      const bool hit = IrqMeaning::finished(irq);
      if (line && !hit) return fail::Outcome::Fail(fail::Kind::Fault, spurious);
      if (!line && hit) return fail::Outcome::Fail(fail::Kind::Fault, missed);
      if (!line && !hit) return fail::Outcome::Fail(fail::Kind::Timeout, irq);
      return fail::Outcome::Ok();
    }
  };

  struct NoFallback {
    static constexpr bool on = false;
    static constexpr bool handles(const fail::Outcome&) { return false; }
    static fail::Outcome report(const fail::Outcome& o) { return o; }
  };

  // A line fault is the delivery's, not the chip's: the row polls ComIrqReg from then on (until the chip is initialised again, which tests the line
  // again). The failure edge still hears of it, as a kind Recover does not act on (Refused, the line's detail kept): nothing initialises the chip.
  struct PollOnLineFault {
    static constexpr bool on = true;
    static constexpr bool handles(const fail::Outcome& o) { return LineCheck::isLineFault(o); }
    static fail::Outcome report(const fail::Outcome& o) { return fail::Outcome::Fail(fail::Kind::Refused, o.detail); }
  };

  template<typename Delivery, typename Observer = NoObserver, typename Check = NoCheck, typename Fallback = NoFallback>
  struct Interrupt {
    static_assert(!Fallback::on || Check::on, "a fallback acts on what the line check finds: give the interrupt part a check");
    static constexpr bool on = true;
    using FallbackPart = Fallback;
    static constexpr uint8_t enable = 0x80 | IrqMeaning::rxIrq | IrqMeaning::timerIrq;   // ComIEnReg while a command runs: IRqInv, RxIEn, TimerIEn
    static constexpr uint8_t idle = 0x80;                                                  // ComIEnReg between commands: IRqInv only
    static constexpr uint8_t pushPull = 0x80;                                              // DivIEnReg IRQPushPull: the chip drives the line
    static constexpr uint16_t timeoutMs = 40;                                              // the chip's own timer ends a command in 25 ms
    static void begin()         { Delivery::begin(); }
    static void arm()           { Delivery::arm(); }
    static bool line()          { return Delivery::ready(); }
    static fail::Outcome check(uint8_t irq, bool line) {
      const fail::Outcome o = Check::of(irq, line);
      Observer::seen(irq, line, o);
      return o;
    }
  };

  struct NoIrq {
    static constexpr bool on = false;
    static constexpr uint8_t enable = 0, idle = 0, pushPull = 0;
    static constexpr uint16_t timeoutMs = 0;
    using FallbackPart = NoFallback;
    static void begin() {}
    static void arm() {}
    static bool line() { return false; }
    static fail::Outcome check(uint8_t, bool) { return fail::Outcome::Ok(); }
  };
  template<typename...> using Void = void;
  template<typename M, typename = void> struct IrqOf { using type = NoIrq; };
  template<typename M> struct IrqOf<M, Void<typename M::Irq>> { using type = typename M::Irq; };

  // no failure handling: the driver polls and nothing is retried, probed or reported
  struct NoFail {
    static constexpr bool lifecycle = false, returnPath = false, idempotent = true;
    template<typename E> using DevStack = fail::Bare;
    template<typename Impl, typename W> using Access = fail::SpiAccess<Impl, W>;
  };

  template<typename W, typename M = NoFail, uint8_t K = 1>
  struct Rc522 : discover::SpiDriverBase<Rc522<W, M, K>, W>, fail::DevEdge<Rc522<W, M, K>, W, M, K> {
    using B    = discover::SpiDriverBase<Rc522, W>;
    using Edge = fail::DevEdge<Rc522, W, M, K>;
    using Irq  = typename IrqOf<M>::type;
    using Produces = hapi::Chain<Card>;
    static constexpr bool mayIsolate = true;   // a health monitor may quarantine the row (it has no isolate(): nothing cuts its supply)
    static constexpr uint8_t recoverMask = fail::bit(fail::Kind::Corrupt);   // a reset the host did not see: init again

    static constexpr uint32_t spiHz   = 4000000;   // the chip takes 10 MHz; 4 is kind to jumper wires
    static constexpr uint8_t  spiMode = 0;
    static constexpr uint8_t  idCmd   = uint8_t((VersionReg << 1) | 0x80);
    using Ids = discover::IdSet<0x91, 0x92, 0x88, 0x12, 0xB2>;   // v1.0, v2.0, then the common clones (FM17522 and others)

    // what the poll found: the card in the field and the errors seen on the way (a BCC mismatch is a corrupt UID)
    // initTries: on which write of the configuration it read back (0: it never did)
    // missStreak: consecutive polls that found no card while a UID is held; a departure is emitted at missPolls
    // inits: how many times the chip was initialised (the first is at discovery)
    // corrupt: canary failures in a row while the ID still answers
    // The poll in flight, with an interrupt part only: phase (Rest: none), rx (the UID answer), limit (the deadline of the command in flight), fallen
    // (the row polls the register: a line fault with a fallback). It outlives the call that started it, so each row holds one. Without an interrupt
    // part the poll runs inside one call and none of this exists.
    enum Phase : uint8_t { Rest, Wake0, Wake1, Anti };
    struct PollState { fail::Deadline limit; uint8_t phase, rx[5]; bool fallen; };
    struct NoPoll {};
    struct DeviceState : std::conditional_t<Irq::on, PollState, NoPoll> { uint32_t uid; uint16_t bccErrors, collisions, initTries, inits; uint8_t missStreak, corrupt; };
    static PollState& pollOf(RowId row) { return B::dev(row); }   // with an interrupt part
    static constexpr bool serves = Irq::on;                       // has a service(row, now) for the loop (fail::Services)
    static constexpr uint8_t corruptMax = 3;   // Corrupts in a row (initialised again each time) before the chip counts as not there
    static constexpr uint8_t missPolls = 3;

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

    // A SoftReset restarts the oscillator and takes effect some time after the write; there is no clock here, so
    // each wait is a bound in register reads. A marker written before it reads back 0 once the reset has happened
    // (TReloadRegL resets to 0). Then PowerDown reads 1 until the oscillator runs (tens of ms; the MFRC522 library
    // allows 150). Writes made before that are lost and the antenna stays off, and PowerDown may not show the whole
    // window: so the configuration is then written until it reads back.
    static void init(RowId row) {
      auto& st = B::dev(row);
      ++st.inits;
      if constexpr (Irq::on) { auto& p = pollOf(row); p.phase = Rest; p.limit.disarm(); p.fallen = false; }   // the reset ends any command in flight; the line is tried again
      wr(row, TReloadRegL, 0x5A);
      wr(row, CommandReg, SoftReset);
      for (uint16_t i = 0; i < 60000 && rd(row, TReloadRegL) != 0x00; ++i) {}
      for (uint16_t i = 0; i < 60000 && (rd(row, CommandReg) & 0x10); ++i) {}
      for (uint16_t i = 0; i < 1000; ++i) {
        wr(row, TModeReg, 0x80);       // timer starts at the end of a transmission
        wr(row, TPrescalerReg, 0xA9);  // 40 kHz tick
        wr(row, TReloadRegH, 0x03);    // 1000 ticks: a 25 ms receive timeout
        wr(row, TReloadRegL, 0xE8);
        wr(row, TxASKReg, 0x40);       // 100% ASK
        wr(row, ModeReg, 0x3D);        // CRC preset 0x6363
        wr(row, TxControlReg, uint8_t(rd(row, TxControlReg) | 0x03));   // antenna on
        if constexpr (Irq::on) { wr(row, DivIEnReg, Irq::pushPull); wr(row, ComIEnReg, Irq::idle); }   // the IRQ pin driven, nothing enabled
        if (configured(row)) { B::dev(row).initTries = uint16_t(i + 1); return; }
      }
    }

    // One exchange with a card, in two halves. start(): FIFO in, Transceive, StartSend; it returns at once.
    static void start(RowId row, const uint8_t* tx, uint8_t n, uint8_t lastBits) {
      wr(row, CommandReg, Idle);
      wr(row, ComIrqReg, 0x7F);             // clear every interrupt request bit
      if constexpr (Irq::on) { if (!pollOf(row).fallen) { Irq::arm(); wr(row, ComIEnReg, Irq::enable); } }
      wr(row, FIFOLevelReg, 0x80);          // flush the FIFO
      for (uint8_t i = 0; i < n; ++i) wr(row, FIFODataReg, tx[i]);
      wr(row, BitFramingReg, lastBits);
      wr(row, CommandReg, Transceive);
      wr(row, BitFramingReg, uint8_t(lastBits | 0x80));   // StartSend
    }

    // finish(): what the command came to, given the ComIrqReg value read at its end. Returns the bytes received, 0 when nothing
    // answered, -1 on an error (collision, parity, protocol, overflow).
    static int8_t finish(RowId row, uint8_t irq, uint8_t* rx, uint8_t max) {
      wr(row, BitFramingReg, 0x00);
      if (!IrqMeaning::answered(irq)) return 0;   // the timer ran out (or the wait did): no card answered
      if (rd(row, ErrorReg) & 0x1B) return -1;    // BufferOvfl, CollErr, ParityErr, ProtocolErr
      uint8_t got = rd(row, FIFOLevelReg);
      if (got > max) got = max;
      for (uint8_t i = 0; i < got; ++i) rx[i] = rd(row, FIFODataReg);
      return int8_t(got);
    }

    // The whole exchange, waiting for the chip: the register is polled until it says the command is done (no interrupt part).
    static int8_t transceive(RowId row, const uint8_t* tx, uint8_t n, uint8_t* rx, uint8_t max, uint8_t lastBits) {
      start(row, tx, n, lastBits);
      uint8_t irq = 0;
      for (uint16_t i = 0; i < 2000; ++i) { irq = rd(row, ComIrqReg); if (irq & 0x31) break; }   // RxIRq, IdleIRq, TimerIRq
      return finish(row, irq, rx, max);
    }

    // WUPA (7 bits): a card in the field answers ATQA. A card left READY by an anticollision without a SELECT ignores
    // the first wake-up and falls back to IDLE, so a silent first try gets one more.
    static bool present(RowId row) {
      const uint8_t wupa = 0x52;
      uint8_t atqa[2];
      for (uint8_t t = 0; t < 2; ++t) if (transceive(row, &wupa, 1, atqa, 2, 0x07) == 2) return true;
      return false;
    }

    // ---- with an interrupt part: the poll as steps, one command each --------------------------------------------------------
    // a WUPA, a second WUPA when it is silent, then the anticollision (cascade level 1)
    static void startPoll(RowId row) {
      const uint8_t wupa = 0x52;
      pollOf(row).phase = Wake0;
      start(row, &wupa, 1, 0x07);
    }

    // a command finished: the next step of the poll
    static void advance(RowId row, int8_t got) {
      auto& st = B::dev(row);
      auto& p = pollOf(row);
      if (p.phase == Wake0 || p.phase == Wake1) {
        if (got == 2) {                                    // ATQA: a card is there
          st.missStreak = 0; p.phase = Anti;
          const uint8_t anticoll[2] = {0x93, 0x20};        // cascade level 1, no UID bits known
          start(row, anticoll, 2, 0x00);
          return;
        }
        if (p.phase == Wake0) { startPoll(row); p.phase = Wake1; return; }
        p.phase = Rest;                                    // nobody answered twice
        if (!st.uid) return;
        if (++st.missStreak < missPolls) return;           // a card held still can miss a poll: it left only after missPolls
        commit(row, 0);
        return;
      }
      p.phase = Rest;
      if (got < 0) { ++st.collisions; return; }
      if (got != 5) return;
      const uint8_t* r = p.rx;
      if (uint8_t(r[0] ^ r[1] ^ r[2] ^ r[3]) != r[4]) { ++st.bccErrors; return; }
      commit(row, (uint32_t(r[0]) << 24) | (uint32_t(r[1]) << 16) | (uint32_t(r[2]) << 8) | r[3]);
    }

    static void commit(RowId row, uint32_t uid) {
      auto& st = B::dev(row);
      if (uid != st.uid) { st.uid = uid; st.missStreak = 0; B::template emit<Card>(row, uid); }
    }

    // The end of the command in flight: by the line (read before the register, which is then released and cleared), or, for a row that fell
    // back, by polling the register until the chip says it is done.
    static int8_t complete(RowId row, fail::Outcome& check) {
      uint8_t irq = 0;
      check = fail::Outcome::Ok();
      if (!pollOf(row).fallen) {
        const bool line = Irq::line();
        irq = rd(row, ComIrqReg);
        wr(row, ComIEnReg, Irq::idle);      // the line is released before the requests are cleared
        wr(row, ComIrqReg, 0x7F);
        check = Irq::check(irq, line);
      } else {
        for (uint16_t i = 0; i < 2000; ++i) { irq = rd(row, ComIrqReg); if (IrqMeaning::finished(irq)) break; }
      }
      return finish(row, irq, pollOf(row).rx, 5);
    }

    // The loop's step, every iteration (fail::Services): the command in flight is finished when the line is asserted or after the timeout. Nothing
    // waits here. A check that fails ends the poll and is reported to the failure edge as the device's failure; a line fault with a fallback
    // is reported as the delivery's, the row polls the register from then on, and the answer the register gave is used.
    static void service(RowId row, [[maybe_unused]] uint32_t now) {
      if constexpr (Irq::on) {
        auto& p = pollOf(row);
        if (p.phase == Rest) return;
        if (!p.limit.armed) p.limit.arm(now, Irq::timeoutMs);
        if (!Irq::line() && !p.limit.due(now)) return;
        p.limit.disarm();
        W::route(W::reg.rows[row].parent);
        fail::Outcome check;
        const int8_t got = complete(row, check);
        if (!check.isOk()) {
          if (Irq::FallbackPart::handles(check)) {
            p.fallen = true; report(row, Irq::FallbackPart::report(check));
            advance(row, got);
            while (p.phase != Rest) { fail::Outcome ok; advance(row, complete(row, ok)); }   // the rest of this poll, by the register
            return;
          }
          p.phase = Rest; report(row, check); return;
        }
        advance(row, got);
      }
    }

    // a failure found after the poll was accepted: the stack sees it as the poll's own
    static void report(RowId row, fail::Outcome o) {
      Edge::Tab::serve(row, fail::Cause::Fresh, [o]() -> fail::Outcome { return o; });
    }

    static void reinit(RowId row) { init(row); }

    // the row went Stale: what was held about the card is unknown
    static void onStale(RowId row) {
      auto& st = B::dev(row);
      st.missStreak = 0; st.corrupt = 0;
      if constexpr (Irq::on) { pollOf(row).phase = Rest; pollOf(row).limit.disarm(); }
      if (st.uid) { st.uid = 0; B::template emit<Card>(row, 0); }
    }

    static void read(RowId row) { Edge::serve(row, fail::Cause::Fresh); }

    // The canary first: the configuration read back. When it is gone the ID tells a chip that was reset (it still answers: Corrupt) from one that
    // does not answer (Absent); only the first is a reason to write the configuration again at once. A chip that answers its ID but loses the
    // configuration again and again (a supply that is not there, the part living off its signal pins) is not reset, it is gone: Absent after
    // corruptMax Corrupts in a row, which takes the row Stale and probes it at the reprobe interval instead of initialising it every poll.
    static fail::Outcome attempt(RowId row) {
      auto& st = B::dev(row);
      if (!configured(row)) {
        const uint8_t v = rd(row, VersionReg);
        if (Ids::has(v) && st.corrupt < corruptMax) { ++st.corrupt; return fail::Outcome::Fail(fail::Kind::Corrupt, v); }
        return fail::Outcome::Fail(fail::Kind::Absent, v);
      }
      st.corrupt = 0;
      pollCard(row);
      return fail::Outcome::Ok();
    }

    // Without an interrupt part the whole poll runs here. With one it is started and service() carries it on; a row that fell back runs it here too.
    static void pollCard(RowId row) {
      if constexpr (Irq::on) {
        if (pollOf(row).phase != Rest) return;
        startPoll(row);
        if (pollOf(row).fallen) while (pollOf(row).phase != Rest) { fail::Outcome ok; advance(row, complete(row, ok)); }
      } else {
        auto& st = B::dev(row);
        uint32_t uid = 0;
        if (!present(row)) {
          if (!st.uid) return;
          if (++st.missStreak < missPolls) return;   // a card held still can miss a poll: it left only after missPolls
        } else {
          st.missStreak = 0;
          const uint8_t anticoll[2] = {0x93, 0x20};   // cascade level 1, no UID bits known
          uint8_t r[5];
          const int8_t got = transceive(row, anticoll, 2, r, 5, 0x00);
          if (got < 0) { ++st.collisions; return; }
          if (got != 5) return;
          if (uint8_t(r[0] ^ r[1] ^ r[2] ^ r[3]) != r[4]) { ++st.bccErrors; return; }
          uid = (uint32_t(r[0]) << 24) | (uint32_t(r[1]) << 16) | (uint32_t(r[2]) << 8) | r[3];
        }
        if (uid != st.uid) { st.uid = uid; st.missStreak = 0; B::template emit<Card>(row, uid); }
      }
    }
  };

}
