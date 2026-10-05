#pragma once
// oneMachine/link/frame.h -- the request/response framing of role/link.h, for a device whose ops answer with a payload.
//
//   request   op (1 byte), length (u16 little-endian), payload
//   response  status (1 byte), length (u16 little-endian), payload
//
// The same bytes as role::Link, so a Python StreamLink or CtypesLink talks to it as to any machine (python/onemachine). role::Link carries a
// role machine's command and report frames and its op hook answers a status only; this one hands the request to the app and sends what the
// app answers. Status numbers keep role::Link's where they overlap.
//
// link::Frame<Out, App, Cap>: Out `static void put(uint8_t)`; feed(byte, now) as bytes arrive. App:
//   static void describe(P& put)                       the text reply of op 'd' (a char sink; called twice: counted, then sent, so it changes nothing)
//   static void request(uint8_t op, const uint8_t* in, uint16_t n, Reply& r)   every other op: r.status(st), r.put(byte), at most ReplyCap bytes
// A request longer than Cap is read to its end and refused (TooLong); an op the app does not know answers Unknown.
#include <stdint.h>

namespace link {
  constexpr uint8_t Ok = 0, BadLength = 2, BadValue = 3, NoValue = 4, Unknown = 0x80, TooLong = 0x81, ReadOnly = 0x82;

  template<unsigned N> struct ReplyBuf {
    uint8_t st = Unknown; uint8_t n = 0; uint8_t data[N]{}; bool over = false;
    void status(uint8_t s) { st = s; }
    void put(uint8_t b) { if (n < N) data[n++] = b; else over = true; }
    void put32(int32_t v) { for (uint8_t i = 0; i < 4; ++i) put(uint8_t(uint32_t(v) >> (8 * i))); }
  };

  template<class Out, class App, unsigned Cap = 48, unsigned ReplyCap = 48>
  struct Frame {
    uint8_t phase = 0, op = 0; uint16_t len = 0, got = 0; uint8_t buf[Cap]{};

    void feed(uint8_t b, uint32_t) {
      switch (phase) {
        case 0: op = b; phase = 1; return;
        case 1: len = b; phase = 2; return;
        case 2: len = uint16_t(len | (uint16_t(b) << 8)); got = 0; phase = 3; if (len) return; break;
        default: if (got < Cap) buf[got] = b; ++got; if (got < len) return;
      }
      phase = 0; request();
    }

  private:
    struct Put { uint16_t n = 0; bool send = false; void operator()(char c) { if (send) Out::put(uint8_t(c)); else ++n; } };
    static void head(uint8_t st, uint16_t n) { Out::put(st); Out::put(uint8_t(n)); Out::put(uint8_t(n >> 8)); }
    void request() {
      if (len > Cap) { head(TooLong, 0); return; }
      if (op == 'd') { Put p; App::describe(p); head(Ok, p.n); p.send = true; App::describe(p); return; }
      ReplyBuf<ReplyCap> r;
      App::request(op, buf, len, r);
      if (r.over) { head(TooLong, 0); return; }
      head(r.st, r.n);
      for (uint8_t i = 0; i < r.n; ++i) Out::put(r.data[i]);
    }
  };
}
