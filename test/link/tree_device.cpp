// The spi example's air sensor as a device a consumer reaches over the link (role/link.h with payload ops, examples/spi/src/tree_ops.h), on simulated time and a
// simulated BMP280 (test/support/mockBmpTwi.h: the datasheet's worked example at 0x76). Built two ways, as test/role does:
//   a process   the link's bytes on stdin/stdout (check_tree.py over a pipe, as over a serial port)
//   -DSIM_LIB   a shared library exporting onemachine_call / onemachine_cycle (check_tree.py in-process through ctypes)
// Test-only ops (not part of the link):
//   't' u32 ms     advance simulated time in 10 ms steps: the poll every 100 ms, the failure edges, the sync pass of the published nodes
//   'x'            soft-reset the sensor behind the host's back          'u'  unplug it          'p'  plug it in again (reset values)
//   'k' u32        a card arrives (the number is its UID; 0: it leaves): a code that only notifies
//   'z' status     the status the card's row has from now on (0 alive, 1 stale, 2 gone)
//   'R' reg        reply: the register's byte, read from the simulated chip (not through the machine)
//   'W' reg v      write a register of the simulated chip (not through the machine)
//   'S' u32 u32    the raw pressure and temperature the chip converts from now on
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <hapi/hapi.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include <oneMachine/fail/busedge.h>
#include <oneMachine/fail/world.h>
#include "../support/mockBmpTwi.h"
#include "../../examples/spi/src/tree_ops.h"
#include "../../examples/spi/src/air_tree.h"

using discover::RowId;
using hapi::Chain;

struct Mode {
  static constexpr bool checked = true, returnPath = false, idempotent = true, lifecycle = true;
  template<typename E> using BusStack = fail::Controller<E, fail::TickPart<fail::Retry<0>>, fail::Recover, fail::DetectError,
    fail::HoldOp<fail::Coalesce>, fail::Backoff<100, 400>, fail::Status>;
  template<typename E> using DevStack = fail::Controller<E, fail::TickPart<fail::Retry<2>>, fail::Recover, fail::DetectError,
    fail::HoldOp<fail::Coalesce>, fail::Gate<50>, fail::TickPart<fail::Reprobe<500, 120>>, fail::LazyStatus>;
};
struct App;
using M = airTree::Machine<App, Mode>;
using Drivers = Chain<M::Driver>;
struct App : discover::World<App, mockbmp::Twi, Chain<>, M::Entries, 3, discover::I2cScan>, fail::BusEdge<App, Drivers, 1, Mode> {
  static constexpr bool lifecycle = true;
  static void release(RowId) {}
  static void unbindAll() {}
  static void busReset() { mockbmp::Twi::begin(); }
};
using Ticker = fail::Ticks<App, Drivers>;

// the codes, with their numbers; `card` only notifies
using airTree::CodeCard;   // the codes and the published nodes: examples/spi/src/air_tree.h

using Queue = bmpm::ChangeQueue<8>;
template<typename Code> static void note(int32_t v) { Queue::note(Code::num, v); }

template<typename Code> struct Note { static void fn(int32_t v) { note<Code>(v); } };
using Pubs = airTree::Pubs<M, Note>;

struct Extra {   // a code that only notifies: the card (an event with a value, the UID, 0 when it leaves), with the status of its row
  using Codes = Chain<CodeCard>;
  static inline uint8_t cardStatus = 0;
  static uint8_t status(uint8_t) { return cardStatus; }
  template<typename P> static constexpr void describe(P& put) {
    airTree::cardLine(put);
    if constexpr (!bmpm::NoStatus<P>::value) { const char* s = cardStatus == 0 ? " status alive" : cardStatus == 1 ? " status stale" : " status gone"; while (*s) put(*s++); }
    put('\n');
  }
};
using Ops = bmpm::TreeOps<M, Pubs, Extra, 1, 8>;

static uint32_t now = 0;
// The machine's nodes are objects with constructors of their own (the machine's statics are initialised in no defined order), so discovery, which
// writes into them, runs from main or the first call, never from a static initialiser.
static void start() { static bool started = false; if (started) return; started = true; mockbmp::State::reset(); mockbmp::State::c77.unplug(); App::discover(); }

struct LinkApp {
  static constexpr bool payload = true;
  static constexpr unsigned replyCap = 96;
  template<typename P> static void describe(P& put) { Ops::describe(put); }
  template<typename R> static void request(uint8_t op, const uint8_t* in, uint16_t n, R& r) {
    using mockbmp::State;
    switch (op) {
      case 't': {
        if (n != 4) { r.status(role::LinkBadLength); return; }
        const uint32_t ms = uint32_t(in[0]) | uint32_t(in[1]) << 8 | uint32_t(in[2]) << 16 | uint32_t(in[3]) << 24;
        for (uint32_t t = 0; t < ms; t += 10, now += 10) {
          if (now % 100 == 0) { App::pump(); bmpm::PublishAll<Pubs>::sync(); }
          App::tickBuses(now); Ticker::run(now);
          Ops::watch();
        }
        r.status(role::LinkOk); return;
      }
      case 'x': State::c76.softReset(); r.status(role::LinkOk); return;
      case 'u': State::c76.unplug(); r.status(role::LinkOk); return;
      case 'p': State::c76.replug(); r.status(role::LinkOk); return;
      case 'k': {
        if (n != 4) { r.status(role::LinkBadLength); return; }
        Queue::note(CodeCard::num, int32_t(uint32_t(in[0]) | uint32_t(in[1]) << 8 | uint32_t(in[2]) << 16 | uint32_t(in[3]) << 24));
        r.status(role::LinkOk); return;
      }
      case 'z': if (n != 1) { r.status(role::LinkBadLength); return; } Extra::cardStatus = in[0]; r.status(role::LinkOk); return;
      case 'R': if (n != 1) { r.status(role::LinkBadLength); return; } r.status(role::LinkOk); r.put(State::c76.regs[in[0]]); return;
      case 'S': {
        if (n != 8) { r.status(role::LinkBadLength); return; }
        auto u = [&](int i) { return uint32_t(in[i]) | uint32_t(in[i + 1]) << 8 | uint32_t(in[i + 2]) << 16 | uint32_t(in[i + 3]) << 24; };
        State::c76.setSample(u(0), u(4)); r.status(role::LinkOk); return;
      }
      case 'W': if (n != 2) { r.status(role::LinkBadLength); return; } State::c76.regs[in[0]] = in[1]; r.status(role::LinkOk); return;
      default: Ops::request(op, in, n, r); return;
    }
  }
};

#ifdef SIM_LIB
struct BufOut {
  static inline uint8_t* p = nullptr; static inline uint16_t cap = 0, n = 0;
  static void put(uint8_t b) { if (n < cap) p[n] = b; ++n; }
};
static role::Machine<>::Report rep0;
static role::Link<role::Machine<>, BufOut, LinkApp, 48> link(rep0, 0);
extern "C" __attribute__((visibility("default"))) int32_t onemachine_call(uint8_t op, const uint8_t* in, uint16_t n, uint8_t* out, uint16_t cap, uint32_t) {
  start(); BufOut::p = out; BufOut::cap = cap; BufOut::n = 0;
  link.feed(op, now); link.feed(uint8_t(n), now); link.feed(uint8_t(n >> 8), now);
  for (uint16_t i = 0; i < n; ++i) link.feed(in[i], now);
  return BufOut::n <= cap ? int32_t(BufOut::n) : -int32_t(BufOut::n);
}
extern "C" __attribute__((visibility("default"))) void onemachine_cycle(uint32_t) {}
#else
struct StdOut { static void put(uint8_t b) { fputc(b, stdout); } };
int main() {
  start();
  static role::Machine<>::Report rep0;
  static role::Link<role::Machine<>, StdOut, LinkApp, 48> link(rep0, 0);
  int c;
  while ((c = fgetc(stdin)) != EOF) { link.feed(uint8_t(c), now); if (link.phase == 0) fflush(stdout); }
  return 0;
}
#endif
