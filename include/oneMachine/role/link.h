#pragma once
// oneMachine/role/link.h -- a machine's roles over a byte stream (a UART, a TCP socket, an MQTT payload pair): request/response.
//
//   request   op (1 byte), length (u16 little-endian), payload
//   response  status (1 byte), length (u16 little-endian), payload
//   op  'm'  the machine description (role/face.h)          'r'  the report description     'c'  the command description
//       'g'  a report frame (state::write of the report)     's'  a command frame: status is state::Status of state::read into the staged
//                                                                  command; Ok stages it for the next take() and arms the quiet deadline
//   anything else goes to App::op(op, payload, n) -> a status (no payload), or Unknown when the app has no such op
//
// role::Link<M, Out, App>: M a role::Machine, Out `static void put(uint8_t)`, App optional. feed(byte, now) as bytes arrive, then at the
// cycle boundary take(cmd) (a new command, if one was accepted) and quiet(now) (true once, when no command was accepted for QuietMs:
// the app applies M::safe then). The request buffer is the command frame size (Cap); a longer request is read to its end and refused.
#include "face.h"
#include <oneMachine/state/wire.h>
#include <oneMachine/fail/deadline.h>

namespace role {
  constexpr uint8_t LinkOk = 0, LinkBadHash = 1, LinkBadLength = 2, LinkBadValue = 3, LinkUnknown = 0x80, LinkTooLong = 0x81;
  struct NoApp { static int op(uint8_t, const uint8_t*, uint16_t) { return -1; } };

  template<class M, class Out, class App = NoApp, unsigned Cap = state::wire_size<typename M::Command>()>
  struct Link {
    using Command = typename M::Command; using Report = typename M::Report;
    const Report& report;
    uint32_t quietMs;
    Command staged{};
    bool fresh = false;
    fail::Deadline silence{};
    uint8_t phase = 0, op = 0; uint16_t len = 0, got = 0; uint8_t buf[Cap]{};

    Link(const Report& r, uint32_t q) : report(r), quietMs(q) {}

    void feed(uint8_t b, uint32_t now) {
      switch (phase) {
        case 0: op = b; phase = 1; return;
        case 1: len = b; phase = 2; return;
        case 2: len = uint16_t(len | (uint16_t(b) << 8)); got = 0; phase = 3; if (len) return; break;
        default: if (got < Cap) buf[got] = b; ++got; if (got < len) return;
      }
      phase = 0; request(now);
    }
    bool take(Command& c) { if (!fresh) return false; c = staged; fresh = false; return true; }
    bool quiet(uint32_t now) { if (!silence.due(now)) return false; silence.disarm(); return true; }

  private:
    // one put type for counting and for sending, so each description is compiled once (a text reply is measured, then sent)
    struct Put { uint16_t n = 0; bool send = false; void operator()(char c) { if (send) Out::put(uint8_t(c)); else ++n; } };
    static void head(uint8_t st, uint16_t n) { Out::put(st); Out::put(uint8_t(n)); Out::put(uint8_t(n >> 8)); }
    template<class F> static void text(F f) { Put p; f(p); head(LinkOk, p.n); p.send = true; f(p); }
    void request(uint32_t now) {
      if (len > Cap) { head(LinkTooLong, 0); return; }
      switch (op) {
        case 'm': text([](Put& p) { describe<M>(p); }); return;
        case 'r': text([](Put& p) { state::describe<Report>(p); }); return;
        case 'c': text([](Put& p) { state::describe<Command>(p); }); return;
        case 'g': { uint8_t f[state::wire_size<Report>()]; state::write(report, f); head(LinkOk, sizeof f); for (uint8_t b : f) Out::put(b); return; }
        case 's': {
          state::Status st = state::read(staged, buf, len);
          if (st == state::Status::Ok) { fresh = true; silence.arm(now, quietMs); }
          head(uint8_t(st), 0); return;
        }
        default: { int st = App::op(op, buf, len); head(st < 0 ? LinkUnknown : uint8_t(st), 0); return; }
      }
    }
  };
}
