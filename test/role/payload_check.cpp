// role::Link with an app whose ops answer with a payload (`payload = true`: describe() for op 'd', request() for the others, role::LinkReply),
// and the same link with an app that answers a status only (op(), as before):
//   - a payload app: the text of 'd' (counted, then sent), a payload reply, a status with no payload, a reply longer than the app's replyCap is TooLong,
//     an op it does not know is Unknown; the link's own ops ('m', 'r', 'c') still answer
//   - an app without it is served exactly as before: op() for every op, 'd' included, and an unknown op is Unknown
// Native only.
#include <stdint.h>
#include <cstdio>
#include <cstring>
#include <oneMachine/role/link.h>

using M = role::Machine<>;                     // a device with no roles: the link's ops are all the app's

struct Out {
  inline static uint8_t buf[512]; inline static unsigned n = 0;
  static void put(uint8_t b) { if (n < sizeof buf) buf[n++] = b; }
};

struct PayloadApp {
  static constexpr bool payload = true;
  static constexpr unsigned replyCap = 96;
  template<class P> static void describe(P& put) { for (const char* s = "tree\n"; *s;) put(*s++); }
  template<class R> static void request(uint8_t op, const uint8_t* in, uint16_t n, R& r) {
    switch (op) {
      case 'e': r.status(role::LinkOk); for (uint16_t i = n; i > 0; --i) r.put(in[i - 1]); return;       // the payload, reversed
      case 'v': r.status(role::LinkOk); r.put32(-2); return;
      case 'q': r.status(role::LinkBadValue); return;
      case 'L': r.status(role::LinkOk); for (int i = 0; i < 200; ++i) r.put(1); return;                   // more than the reply holds
      default: r.status(role::LinkUnknown); return;
    }
  }
};
struct OpApp { inline static int calls = 0; static int op(uint8_t o, const uint8_t*, uint16_t n) { ++calls; return o == 'k' ? int(n) : -1; } };

static int failures = 0;
#define CHECK(...) do { if (!(__VA_ARGS__)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #__VA_ARGS__); } } while (0)

template<class L> static void send(L& l, uint8_t op, const void* p = nullptr, uint16_t n = 0) {
  Out::n = 0;
  l.feed(op, 0); l.feed(uint8_t(n), 0); l.feed(uint8_t(n >> 8), 0);
  for (uint16_t i = 0; i < n; ++i) l.feed(static_cast<const uint8_t*>(p)[i], 0);
}
static uint8_t status() { return Out::buf[0]; }
static unsigned len() { return Out::buf[1] | (Out::buf[2] << 8); }
static const uint8_t* body() { return Out::buf + 3; }

int main() {
  static M::Report rep;
  static role::Link<M, Out, PayloadApp, 48> pl(rep, 0);
  send(pl, 'd');
  CHECK(status() == role::LinkOk && len() == 5 && std::memcmp(body(), "tree\n", 5) == 0 && Out::n == 8);       // the text, sent once
  send(pl, 'e', "abc", 3);
  CHECK(status() == role::LinkOk && len() == 3 && std::memcmp(body(), "cba", 3) == 0);
  send(pl, 'v');
  CHECK(status() == role::LinkOk && len() == 4 && body()[0] == 0xFE && body()[1] == 0xFF && body()[3] == 0xFF);
  send(pl, 'q');
  CHECK(status() == role::LinkBadValue && len() == 0 && Out::n == 3);
  send(pl, 'L');
  CHECK(status() == role::LinkTooLong && len() == 0);                                                          // a reply that does not fit is refused, not cut
  send(pl, 'z');
  CHECK(status() == role::LinkUnknown && len() == 0);
  uint8_t big[60] = {};
  send(pl, 'e', big, sizeof big);                                                                              // a request longer than Cap
  CHECK(status() == role::LinkTooLong && len() == 0);
  send(pl, 'e', "xy", 2);
  CHECK(status() == role::LinkOk && len() == 2);                                                               // and the link carries on
  send(pl, 'm');
  CHECK(status() == role::LinkOk && len() > 10 && std::memcmp(body(), "machine 1\n", 10) == 0);               // the link's own ops still answer
  send(pl, 'r'); CHECK(status() == role::LinkOk);
  send(pl, 'c'); CHECK(status() == role::LinkOk);

  static role::Link<M, Out, OpApp, 48> op(rep, 0);
  send(op, 'k', "abc", 3);
  CHECK(status() == 3 && len() == 0 && OpApp::calls == 1);                                                     // op() answers a status, as before
  send(op, 'd');
  CHECK(status() == role::LinkUnknown && OpApp::calls == 2);                                                   // 'd' is not special without a payload app
  send(op, 'z');
  CHECK(status() == role::LinkUnknown && OpApp::calls == 3);
  send(op, 'm');
  CHECK(status() == role::LinkOk && OpApp::calls == 3);                                                        // the link's own ops never reach the app

  if (failures) { std::printf("FAILED: %d\n", failures); return 1; }
  std::printf("OK: role::Link payload ops native\n");
  return 0;
}
