#pragma once
// oneMachine/role/link.h -- a machine's roles over a byte stream (a UART, a TCP socket, an MQTT payload pair): request/response.
//
//   request   op (1 byte), length (u16 little-endian), payload
//   response  status (1 byte), length (u16 little-endian), payload
//   op  'm'  the machine description (role/face.h)          'r'  the report description     'c'  the command description
//       'g'  a report frame (state::write of the report)     's'  a command frame: status is state::Status of state::read into the staged
//                                                                  command; Ok stages it for the next take() and arms the quiet deadline
//       when the machine has tuned roles (role::Tuned):
//       'T'  the tuning description  'G'  a tuning frame (the current values)  'S'  a tuning frame: BadHash/BadLength as for 's', BadValue
//                                                                  when a value is outside the firmware's limits (nothing changes); Ok
//                                                                  takes effect at once, and the next take() re-applies the command
//   anything else goes to App::op(op, payload, n) -> a status (no payload), or Unknown when the app has no such op
//   An app whose ops answer with a payload declares `static constexpr bool payload = true`, and then (instead of op()):
//       static void describe(P& put)                                     the text reply of op 'd' (a char sink, called twice: counted, then sent)
//       static void request(uint8_t op, const uint8_t* in, uint16_t n, role::LinkReply<N>& r)    every other op: r.status(st), r.put(byte), r.put32(v)
//   (examples/spi: the machine tree's ops d, v, w, n, f.) An app without it is served exactly as before.
//
// role::Link<M, Out, App>: M a role::Machine, Out `static void put(uint8_t)`, App optional. feed(byte, now) as bytes arrive, then at the
// cycle boundary take(cmd) (a new command, if one was accepted) and quiet(now) (true once, when no command was accepted for QuietMs:
// the app applies M::safe then). The request buffer is the command frame size, or the tuning frame's when larger (Cap); a longer request is read to its end and refused.
#include "face.h"
#include <oneMachine/state/wire.h>
#include <oneMachine/fail/deadline.h>

namespace role {
  constexpr uint8_t LinkOk = 0, LinkBadHash = 1, LinkBadLength = 2, LinkBadValue = 3, LinkUnknown = 0x80, LinkTooLong = 0x81;
  constexpr uint8_t LinkNoValue = 4, LinkReadOnly = 0x82;      // for apps with payload ops: a group has no value; a write to a read-only code
  // what an app's request() answers: a status and up to N payload bytes (more is TooLong)
  template<unsigned N> struct LinkReply {
    uint8_t st = LinkUnknown; uint8_t n = 0; uint8_t data[N]{}; bool over = false;
    void status(uint8_t s) { st = s; }
    void put(uint8_t b) { if (n < N) data[n++] = b; else over = true; }
    void put32(int32_t v) { for (uint8_t i = 0; i < 4; ++i) put(uint8_t(uint32_t(v) >> (8 * i))); }
  };
  template<class A, class = void> struct PayloadOps : std::false_type {};
  template<class A> struct PayloadOps<A, std::void_t<decltype(A::payload)>> : std::bool_constant<A::payload> {};
  struct NoApp { static int op(uint8_t, const uint8_t*, uint16_t) { return -1; } };

  // the longest request a link reads: a command frame, or a tuning frame when the machine has tuned roles
  template<class M> constexpr unsigned link_cap() {
    unsigned c = state::wire_size<typename M::Command>();
    if constexpr (M::tunable) { unsigned t = state::wire_size<typename M::Tuning>(); return t > c ? t : c; } else return c;
  }

  template<class M, class Out, class App = NoApp, unsigned Cap = link_cap<M>(), unsigned ReplyCap = 48>
  struct Link {
    using Command = typename M::Command; using Report = typename M::Report;
    const Report& report;
    uint32_t quietMs;
    Command staged{};
    bool fresh = false, retuned = false;
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
    // true when the command must be applied again: a new one (copied into c), or new tuning (c unchanged)
    bool take(Command& c) { bool r = fresh || retuned; if (fresh) c = staged; fresh = retuned = false; return r; }
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
        case 'T': if constexpr (M::tunable) { text([](Put& p) { state::describe<typename M::Tuning>(p); }); return; } else break;
        case 'G': if constexpr (M::tunable) {
          uint8_t f[state::wire_size<typename M::Tuning>()]; state::write(M::tuning, f); head(LinkOk, sizeof f); for (uint8_t b : f) Out::put(b); return;
        } else break;
        case 'S': if constexpr (M::tunable) {
          typename M::Tuning t = M::tuning;
          state::Status st = state::read(t, buf, len);
          if (st == state::Status::Ok && !M::valid(t)) st = state::Status::BadValue;
          if (st == state::Status::Ok) { M::tuning = t; retuned = true; }
          head(uint8_t(st), 0); return;
        } else break;
        case 'd': if constexpr (PayloadOps<App>::value) { text([](Put& p) { App::describe(p); }); return; } else break;
        default: break;
      }
      if constexpr (PayloadOps<App>::value) {
        LinkReply<ReplyCap> r;
        App::request(op, buf, len, r);
        if (r.over) { head(LinkTooLong, 0); return; }
        head(r.st, r.n);
        for (uint8_t i = 0; i < r.n; ++i) Out::put(r.data[i]);
      } else { int st = App::op(op, buf, len); head(st < 0 ? LinkUnknown : uint8_t(st), 0); }
    }
  };
}
