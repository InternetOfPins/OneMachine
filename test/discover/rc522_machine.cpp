// The RC522 as a static machine of OneMenu ItemDef nodes (examples/spi/src/rc522_machine.h) against the register model (test/support/mockSpi.h), under the
// failure edge:
//   - discovery: the part is found, its VersionReg read, and the registers of `rf` hold the defaults (the init), the antenna on
//   - a set reaches the register, through the node's limits: the gain is a field of RFCfgReg (bits 6..4), the antenna one of TxControlReg (bits 1..0), and
//     a set of a field keeps the other bits as they are wanted
//   - a reset behind the host's back (an RST pulse, a SoftReset that lands late) is Corrupt: the canary reads the registers back, and what comes back is
//     what was set, not the default
//   - a set while the row is Stale is the intent that comes back
//   - another VersionReg is another part: the defaults
//   - the card: an event that queues through ChangeQueue and counts what it refused; with the antenna off no card is seen
//   - the link's ops over the nodes (get, set, the refusals) and the description walk
// Native only. Time is simulated: the poll every 100 ms, the failure edge ticked every 10 ms.
#include <stdint.h>
#include <cstdio>
#include <cstring>
#include <hapi/hapi.h>
#include <oneMachine/discover/spi.h>
#include <oneMachine/fail/world.h>
#include "../support/mockSpi.h"
#include "../../examples/spi/src/tree_ops.h"
#include "../../examples/spi/src/rfid_tree.h"

using discover::RowId;
using discover::Sample;
using discover::Status;
using hapi::Chain;

using Queue = bmpm::ChangeQueue<8>;

// the card, as the App makes it a change the link reads: its UID when it arrives, 0 when it leaves
struct CardNotifier {
  using Accepts = Chain<rc522::Card>;
  inline static uint32_t seen[32];
  inline static uint8_t n = 0;
  template<typename Cap> struct Body {
    template<typename T> struct Part : T {
      using T::T;
      void on(const Sample<Cap>& s) { if (n < 32) seen[n++] = s.value; Queue::note(rfidTree::CodeCard::num, int32_t(s.value)); }
    };
  };
};

struct Mode {
  static constexpr bool lifecycle = true, returnPath = false, idempotent = true;
  template<typename E> using DevStack = fail::Controller<E, fail::TickPart<fail::Retry<2>>, fail::Recover, fail::DetectError,
    fail::HoldOp<fail::Coalesce>, fail::Gate<50>, fail::TickPart<fail::Reprobe<500, 120>>, fail::LazyStatus>;
  template<typename Impl, typename W> using Access = fail::SpiAccess<Impl, W>;
};

struct App;
using M = rfidTree::Machine<App, Mode>;
using Rfid = M::Driver;
struct App : discover::World<App, mspi::Bus, Chain<CardNotifier>, Chain<Rfid>, 3, discover::SpiScan, Chain<discover::SpiSlotIds<4>>> {
  static constexpr bool lifecycle = true;
  static void release(RowId) {}
  static void unbindAll() {}
};
using Ticker = fail::Ticks<App, App::DriverList>;

static int failures = 0;
#define CHECK(...) do { if (!(__VA_ARGS__)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #__VA_ARGS__); } } while (0)

static uint32_t now = 0;
static bool leftAlive = false;   // did the row leave Alive since the last start()

// the rig: one RC522 on slot 0 with a card on it, discovered. The machine's statics outlive a scenario: they start again too.
static void start(bool card = true) {
  mspi::State::reset();
  mspi::State::kind[0] = mspi::Kind::Rc522;
  mspi::State::card = mspi::Card{card, {0xDE, 0xAD, 0xBE, 0xEF}, false, false, false};
  M::Dev::known = false; M::Dev::restored = M::Dev::defaulted = 0; M::Dev::uid = 0; M::reset();
  CardNotifier::n = 0; leftAlive = false; now = 0;
  while (!Queue::q.empty()) Queue::q.pop();
  Queue::seenDrops = Queue::q.status().drops;
  Rfid::resetFail();
  App::discover();
}
static void advance(uint32_t ms) {
  for (const uint32_t end = now + ms; now < end; now += 10) {
    if (now % 100 == 0) App::pump();
    Ticker::run(now);
    if (App::reg.status(1) != Status::Alive) leftAlive = true;
  }
}
static bool runUntil(Status s, uint32_t limitMs) {
  for (uint32_t t = 0; t < limitMs; t += 10) { if (App::reg.status(1) == s) return true; advance(10); }
  return App::reg.status(1) == s;
}
static auto& st() { return App::devState<Rfid>(1); }
static auto& gain() { return M::resolve<2, 6>(); }
static auto& antenna() { return M::resolve<2, 7>(); }
static uint8_t chip(uint8_t reg) { return mspi::State::rc.regs[reg]; }

// ---- the link's ops over the reader's nodes alone (the codes numbered from 0 here) ---------------------------------------------------------
struct CodeCard    { static constexpr uint8_t num = 0; ONEMACHINE_STATE_NAME(name, "card"); };
struct CodeGain    { static constexpr uint8_t num = 1; ONEMACHINE_STATE_NAME(name, "rfid/gain"); };
struct CodeAntenna { static constexpr uint8_t num = 2; ONEMACHINE_STATE_NAME(name, "rfid/antenna"); };
using Pubs = Chain<rc522m::PublishedAt<CodeCard,    rc522m::PathRef<M, 0>>,
                   rc522m::PublishedAt<CodeGain,    rc522m::PathRef<M, 2, 6>>,
                   rc522m::PublishedAt<CodeAntenna, rc522m::PathRef<M, 2, 7>>>;
struct Tree {
  using Pubs = ::Pubs;
  template<typename P> static void describe(P& put) { rc522m::Walk<P> w{put}; w.template machine<M>(); w.str("published\n"); w.template publishedAll<M>(static_cast<Pubs*>(nullptr), 0); }
  template<typename P> static constexpr void describeStatic(P& put) { rc522m::HashWalk<P> w{put}; w.template machine<M>(); w.str("published\n"); w.template publishedAll<M>(static_cast<Pubs*>(nullptr), 0); }
};
using Ops = bmpm::TreeOps<Tree, 8>;
struct Reply {
  uint8_t st = 0xFF, data[96] = {}; uint16_t n = 0;
  void status(uint8_t s) { st = s; }
  void put(uint8_t b) { data[n++] = b; }
  void put32(int32_t v) { for (int i = 0; i < 4; ++i) put(uint8_t(uint32_t(v) >> (8 * i))); }
};
static uint8_t key[5];
static void code(uint8_t num) { constexpr uint32_t h = Ops::hash(); for (int i = 0; i < 4; ++i) key[i] = uint8_t(h >> (8 * i)); key[4] = num; }
static uint8_t get(uint8_t num, int32_t* v = nullptr) {
  code(num); Reply r; Ops::request('v', key, 5, r);
  if (v && r.st == role::LinkOk) *v = int32_t(uint32_t(r.data[1]) | uint32_t(r.data[2]) << 8 | uint32_t(r.data[3]) << 16 | uint32_t(r.data[4]) << 24);
  return r.st;
}
static uint8_t put(uint8_t num, int32_t v) {
  uint8_t in[9]; code(num);
  for (int i = 0; i < 4; ++i) in[i] = uint8_t(uint32_t(v) >> (8 * i));
  std::memcpy(in + 4, key, 5);
  Reply r; Ops::request('w', in, 9, r); return r.st;
}

struct Str { char b[2048]; unsigned n = 0; void operator()(char c) { if (n < sizeof(b) - 1) b[n++] = c; b[n] = 0; } };

int main() {
  using mspi::State;

  // ---- discovery: found, identified, configured with the defaults --------------------------------------------------------------------
  start();
  CHECK(App::reg.count == 2 && M::status() == 0);
  CHECK(M::Dev::slot == 0 && M::Dev::version == 0x92);
  CHECK(M::Dev::defaulted == 1 && M::Dev::restored == 0);
  CHECK(chip(0x2A) == 0x80 && chip(0x2B) == 0xA9 && chip(0x2C) == 0x03 && chip(0x2D) == 0xE8);       // the timer: 25 ms
  CHECK(chip(0x15) == 0x40 && chip(0x11) == 0x3D && chip(0x26) == 0x48 && chip(0x14) == 0x83);       // 100% ASK, CRC preset, gain 4 (33 dB), antenna on
  CHECK(M::holds() && Rfid::configured(1) && st().initTries >= 1 && st().inits == 1);   // written until it read back: the oscillator starts late
  CHECK(gain().get() == 4 && gain().desired() == 0x48 && antenna().get() == 3 && antenna().desired() == 0x83);   // the field, and the whole register
  CHECK(!std::is_same<M::Dev, rc522m::Machine<App, rc522m::Slot<1>, Mode>::Dev>::value);            // another slot is another type, with statics of its own

  // ---- a set reaches the register; the node's limits are the link's refusals ----------------------------------------------------------------
  gain().set(7);
  CHECK(chip(0x26) == 0x78 && gain().get() == 7 && gain().desired() == 0x78 && M::holds());           // bits 6..4; bit 3 (reserved, set in the reset value) is kept
  CHECK(get(1) == role::LinkOk && put(1, 3) == role::LinkOk && chip(0x26) == 0x38);
  CHECK(put(1, 8) == role::LinkBadValue && put(1, -1) == role::LinkBadValue && chip(0x26) == 0x38);    // outside 0..7: refused, nothing written
  CHECK(put(2, 4) == role::LinkBadValue && put(2, -1) == role::LinkBadValue && chip(0x14) == 0x83);    // outside 0..3
  CHECK(put(2, 0) == role::LinkOk && chip(0x14) == 0x80 && antenna().desired() == 0x80);               // both drivers off; bit 7 (InvTx2RFOn) is kept
  CHECK(put(2, 1) == role::LinkOk && chip(0x14) == 0x81);                                              // TX1 only: a valid setting
  CHECK(put(2, 3) == role::LinkOk && chip(0x14) == 0x83);
  CHECK(put(0, 1) == role::LinkReadOnly);                                                              // the card is read-only
  { int32_t v = 0; CHECK(get(1, &v) == role::LinkOk && v == 3); CHECK(get(0) == role::LinkNoValue); } // an event has no value: the status alone
  State::rc.regs[0x26] = 0x0F;                                                                         // the chip holds something else outside the field
  gain().set(2);
  CHECK(chip(0x26) == 0x28 && gain().desired() == 0x28);                                               // the other bits are as they are wanted (0x48's), not as the chip holds them
  gain().set(7);

  // ---- a reset behind the host's back: the canary, and the setting comes back --------------------------------------------------------------
  advance(300);
  CHECK(st().inits == 1);
  State::rc.hold(true); State::rc.hold(false);                                                         // an RST pulse: the ID still answers, the configuration is gone
  CHECK(chip(0x26) == 0x48 && chip(0x14) == 0x80 && !M::holds());
  advance(300);
  CHECK(st().inits == 2 && M::holds() && !leftAlive);                                                  // initialised again at once, the row never left Alive
  CHECK(chip(0x26) == 0x78 && gain().get() == 7);                                                      // the gain that was set, not the default
  CHECK(M::Dev::restored == 1 && M::Dev::defaulted == 1);
  CHECK(Rfid::failStatus(1).recovers == 1);
  antenna().set(0);                                                                                    // two settings: the gain and the antenna off
  State::rc.hold(true); State::rc.hold(false);
  advance(300);
  CHECK(st().inits == 3 && M::holds() && !leftAlive);
  CHECK(chip(0x26) == 0x78 && chip(0x14) == 0x80 && M::Dev::restored == 2);                            // both came back, not 0x48 and 0x83
  antenna().set(3);
  Rfid::wr(1, rc522::CommandReg, rc522::SoftReset);                                                    // a SoftReset that lands late while running: the same
  advance(500);
  CHECK(st().inits == 4 && M::holds() && !leftAlive);
  CHECK(chip(0x26) == 0x78 && chip(0x14) == 0x83 && M::Dev::restored == 3);

  // ---- a set while the row is Stale is the intent that comes back ---------------------------------------------------------------------------
  State::rc.hold(true);                                                                                // the reader vanishes (RST low)
  CHECK(runUntil(Status::Stale, 2000) && M::status() == 1);
  gain().set(5);                                                                                       // the write cannot reach it
  CHECK(gain().desired() == 0x58 && chip(0x26) == 0x78);
  { using PubGain = rc522m::PublishedAt<CodeGain, rc522m::PathRef<M, 2, 6>>; CHECK(PubGain::last() == 5); }   // what a stale code answers
  { int32_t v = 0; CHECK(get(1, &v) == role::LinkOk && v == 5); }
  advance(500);
  State::rc.hold(false);
  CHECK(runUntil(Status::Alive, 2000) && M::status() == 0);
  advance(100);
  CHECK(chip(0x26) == 0x58 && gain().get() == 5 && M::holds());
  CHECK(M::Dev::restored == 4 && M::Dev::defaulted == 1);

  // ---- another VersionReg is another part: the defaults --------------------------------------------------------------------------------------
  gain().set(7);
  State::rc.regs[0x37] = 0x91;                                                                         // a v1.0 part in the slot
  State::rc.hold(true); State::rc.hold(false);
  advance(300);
  CHECK(M::Dev::version == 0x91 && M::Dev::defaulted == 2 && M::Dev::restored == 4);
  CHECK(chip(0x26) == 0x48 && gain().desired() == 0x48 && M::holds());                                 // the defaults, not 7
  State::rc.regs[0x37] = 0x92;
  State::rc.hold(true); State::rc.hold(false);                                                         // and back: a different part again
  advance(300);
  CHECK(M::Dev::version == 0x92 && M::Dev::defaulted == 3);

  // ---- the card: an event, queued; the newest are refused and counted when it is full -------------------------------------------------------
  start();
  advance(500);
  CHECK(CardNotifier::n == 1 && CardNotifier::seen[0] == 0xDEADBEEFu && M::Dev::uid == 0xDEADBEEFu);   // arrived
  CHECK(M::resolve<0>().get() == 0xDEADBEEFu);
  State::card.present = false; advance(500);
  CHECK(CardNotifier::n == 2 && CardNotifier::seen[1] == 0 && M::Dev::uid == 0);                       // left, after 3 polls without it
  start();
  for (uint8_t i = 0; i < 12; ++i) { State::card.uid[3] = uint8_t(i + 1); advance(300); }              // twelve cards, one after the other, nobody reads them
  CHECK(CardNotifier::n == 12 && CardNotifier::seen[0] == 0xDEADBE01u && CardNotifier::seen[11] == 0xDEADBE0Cu);
  CHECK(!Queue::q.empty() && Queue::q.status().drops - Queue::seenDrops == 4);                          // the oldest 8 are kept, the 4 refused are counted
  { uint8_t kept = 0; uint32_t first = 0; int32_t last = 0;
    while (!Queue::q.empty()) { const fail::Rec& e = Queue::q.front(); if (!kept++) first = uint32_t(e.v); last = e.v; Queue::q.pop(); }
    CHECK(kept == 8 && first == 0xDEADBE01u && uint32_t(last) == 0xDEADBE08u); }

  // ---- no antenna, no card ---------------------------------------------------------------------------------------------------------------------
  start();
  antenna().set(0);
  advance(1000);
  CHECK(CardNotifier::n == 0 && M::Dev::uid == 0 && M::holds() && st().inits == 1);                    // off by intent: nothing wrong, no card seen
  antenna().set(3);
  advance(500);
  CHECK(CardNotifier::n == 1 && M::Dev::uid == 0xDEADBEEFu);

  // ---- the walk -----------------------------------------------------------------------------------------------------------------------------
  start();
  { Str s; Tree::describe(s);
    static const char want[] =
      "machine rc522 at slot 0\n"
      "  #0 card ro value u32\n"
      "  #1 version const 0x37 [1]\n"
      "  #2 rf group 8\n"
      "    #0 tmode reg 0x2A default 0x80 rw range 0..255\n"
      "    #1 tprescaler reg 0x2B default 0xA9 rw range 0..255\n"
      "    #2 treload_h reg 0x2C default 0x03 rw range 0..255\n"
      "    #3 treload_l reg 0x2D default 0xE8 rw range 0..255\n"
      "    #4 txask reg 0x15 default 0x40 rw range 0..255\n"
      "    #5 mode reg 0x11 default 0x3D rw range 0..255\n"
      "    #6 rfcfg reg 0x26 default 0x04 field 6..4 rw range 0..7\n"
      "    #7 txcontrol reg 0x14 default 0x03 field 1..0 rw range 0..3\n"
      "published\n"
      "  card -> 0/0/0 notify event ro value u32 status alive\n"
      "  rfid/gain -> 0/0/2/6 silent reg 0x26 default 0x04 field 6..4 rw range 0..7 status alive\n"
      "  rfid/antenna -> 0/0/2/7 silent reg 0x14 default 0x03 field 1..0 rw range 0..3 status alive\n";
    if (std::strcmp(s.b, want) != 0) { ++failures; std::printf("FAIL description:\n%s--- wanted:\n%s", s.b, want); } }
  { Str a; Tree::describe(a);
    Str b; struct Put { Str& s; void operator()(char c) { s(c); } } p{b}; Tree::describeStatic(p);
    // the hash walk is the text walk without the statuses
    std::string stripped; for (const char* c = a.b; *c;) { const char* nl = std::strchr(c, '\n'); std::string line(c, nl); const auto at = line.find(" status "); stripped += (at == std::string::npos ? line : line.substr(0, at)) + "\n"; c = nl + 1; }
    CHECK(stripped == b.b); }
  CHECK(Ops::hash() != 0 && Ops::numCodes == 3);
  CHECK(std::is_same<rfidTree::Pubs<M>::Head::Inner, M::Card>::value);                                // the App's pubs reach the same nodes by the same paths

  if (failures) { std::printf("FAILED: %d\n", failures); return 1; }
  std::printf("OK: RC522 machine native\n");
  return 0;
}
