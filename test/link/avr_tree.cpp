// The spi example's composition on an ATmega328P, for its size: the BMP280 machine of OneMenu ItemDef nodes under the same failure edges as examples/spi,
// on the real AVR TWI core (OneChip), with or without the link, and with the link the RC522 machine on the real AVR SPI core. examples/spi is
// ESP8266-only, so this is the AVR balance of the same parts (test/link/sizes.sh builds every variant and prints the table).
//   (default)    bare: the air sensor's machine, its failure edges, the sync pass of the published nodes into a sink; no link, no tree ops, no change store,
//                no reader
//   -DREADER     and the RFID reader: the plain RC522 driver (rc522.h) on SPI, chip select PB1, under a failure edge; a card goes to a sink
//   -DMACHINE    the reader as a machine (rc522_machine.h): its registers are the configuration the canary reads back
//   -DLINK       role::Link over USART0 with the tree's payload ops (examples/spi/src/tree_ops.h: d v w n) over both machines; implies -DMACHINE, and the
//                card is an event in the link's queue
//   -DNOREADER   with -DLINK: the link over the air sensor's machine alone (no reader, no SPI world): what the link ops cost without the reader
// millis() is Timer0 overflow, as the Arduino core does it; nothing here is the Arduino core.
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <hapi/hapi.h>
#include <chips/avr/avrTwi.h>
#include <chips/avr/avrUart.h>
#if defined(LINK) && !defined(NOREADER)
  #define MACHINE
#endif
#ifdef MACHINE
  #define READER
#endif
#ifdef READER
  #include <chips/avr/avrSpi.h>
  #include <oneMachine/discover/spi.h>
#endif
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include <oneMachine/fail/busedge.h>
#include <oneMachine/fail/world.h>
#ifdef LINK
  #include "../../examples/spi/src/tree_ops.h"
#endif
#include "../../examples/spi/src/air_tree.h"

using discover::RowId;
using hapi::Chain;

using Twi = hw::avr::mega::Twi<100000>;
using Uart = hw::avr::mega::Serial0<115200>;

static volatile uint32_t ms = 0;
ISR(TIMER0_OVF_vect) { ms = ms + 1; }   // 16 MHz / 64 / 256: 1.024 ms, close enough for a size image
uint32_t millis() { uint8_t s = SREG; cli(); const uint32_t m = ms; SREG = s; return m; }

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
struct App : discover::World<App, Twi, Chain<>, M::Entries, 3, discover::I2cScan>, fail::BusEdge<App, Drivers, 1, Mode> {
  static constexpr bool lifecycle = true;
  static void release(RowId) {}
  static void unbindAll() {}
  static void busReset() { Twi::begin(); }
};
using Ticker = fail::Ticks<App, Drivers>;

using airTree::CodeCard;   // the codes and the published nodes: examples/spi/src/air_tree.h

volatile int32_t g_sink;
#ifdef READER
// the reader: slot 0 of an SPI bus (chip select PB1; slot 1 is declared with nothing on it), under a failure edge as in examples/spi
template<uint8_t Bit> struct PinB {
  static void begin() { DDRB |= uint8_t(1u << Bit); }
  static void on()    { PORTB |= uint8_t(1u << Bit); }
  static void off()   { PORTB &= uint8_t(~(1u << Bit)); }
};
using Spi = hapi::APIOf<oneBus::SpiAPI, oneBus::SpiSlots<PinB<1>, PinB<0>>, oneBus::SpiMaster<4000000>, hw::avr::AvrSpiCore<16000000>>;
struct RMode {
  static constexpr bool lifecycle = true, returnPath = false, idempotent = true;
  template<typename E> using DevStack = fail::Controller<E, fail::TickPart<fail::Retry<2>>, fail::Recover, fail::DetectError,
    fail::HoldOp<fail::Coalesce>, fail::Gate<50>, fail::TickPart<fail::Reprobe<500, 120>>, fail::LazyStatus>;
  template<typename Impl, typename W> using Access = fail::SpiAccess<Impl, W>;
};
struct RApp;
#ifdef MACHINE
using R = rfidTree::Machine<RApp, RMode>;
using Rfid = R::Driver;
#else
using Rfid = rc522::Rc522<RApp, RMode, 1>;
#endif
using RDrivers = Chain<Rfid>;
#else
volatile uint32_t g_card;   // a card's UID, written by whatever reads the reader (an RC522 is not part of the bare image)
#endif
#ifdef LINK
#ifndef EVENTS
  #define EVENTS 8   // the event queue's depth (the card)
#endif
using Queue = bmpm::ChangeQueue<EVENTS>;
template<typename Code> static void note(int32_t) { bmpm::StateChanges<>::mark(Code::num); }   // a state code: a pending bit
#else
template<typename Code> static void note(int32_t v) { g_sink = v; }
#endif

template<typename Code> struct Note { static constexpr auto fn = &note<Code>; };
using Pubs = airTree::Pubs<M, Note>;

#ifdef LINK
struct CardNotifier {   // the card, as a change the link can read: its UID when it arrives, 0 when it leaves
  using Accepts = Chain<rc522::Card>;
  template<typename Cap> struct Body {
    template<typename T> struct Part : T {
      using T::T;
      void on(const discover::Sample<Cap>& s) { Queue::note(CodeCard::num, int32_t(s.value)); }
    };
  };
};
using RConsumer = CardNotifier;
#ifdef NOREADER
struct Tree {   // the air sensor's machine alone
  using Pubs = ::Pubs;
  template<typename P> static void describe(P& put) {
    bmpm::Walk<P> b{put}; b.template machine<M>(); b.str("published\n"); b.template publishedAll<M>(static_cast<Pubs*>(nullptr), airTree::bus);
  }
  template<typename P> static constexpr void describeStatic(P& put) {
    bmpm::HashWalk<P> b{put}; b.template machine<M>(); b.str("published\n"); b.template publishedAll<M>(static_cast<Pubs*>(nullptr), airTree::bus);
    airTree::wiringLines(put);
  }
};
#else
using Tree = airTree::Tree<M, Pubs, R, rfidTree::Pubs<R>>;
#endif
using Ops = bmpm::TreeOps<Tree, EVENTS>;
struct UartOut { static void put(uint8_t b) { Uart::putch(b); } };
struct LinkApp {
  static constexpr bool payload = true;
  static constexpr unsigned replyCap = 96;
  template<typename P> static void describe(P& put) { Ops::describe(put); }
  template<typename R> static void request(uint8_t op, const uint8_t* in, uint16_t n, R& r) { Ops::request(op, in, n, r); }
};
static role::Machine<>::Report linkReport;
static role::Link<role::Machine<>, UartOut, LinkApp, 48> link(linkReport, 0);
#endif

#ifdef READER
#ifndef LINK
struct CardSink {   // the card, to a sink
  using Accepts = Chain<rc522::Card>;
  template<typename Cap> struct Body {
    template<typename T> struct Part : T {
      using T::T;
      void on(const discover::Sample<Cap>& s) { g_sink = int32_t(s.value); }
    };
  };
};
using RConsumer = CardSink;
#endif
struct RApp : discover::World<RApp, Spi, Chain<RConsumer>, RDrivers, 3, discover::SpiScan, Chain<discover::SpiSlotIds<2>>> {
  static constexpr bool lifecycle = true;
  static void release(RowId) {}
  static void unbindAll() {}
};
using RTicker = fail::Ticks<RApp, RApp::DriverList>;
#endif

int main() {
  TCCR0B = (1 << CS01) | (1 << CS00); TIMSK0 = (1 << TOIE0); sei();
  Twi::begin();
  Uart::begin();
#ifdef READER
  Spi::begin();
#endif
  App::discover();
#ifdef READER
  RApp::discover();
  uint32_t nextAir = 0, nextCard = 0;
#else
  uint32_t nextAir = 0, lastCard = 0;
#endif
  for (;;) {
    const uint32_t now = millis();
#ifdef LINK
    while (Uart::available()) link.feed(Uart::getch(), now);
    Ops::watch();
#endif
#ifdef READER
    if (int32_t(now - nextCard) >= 0) { nextCard = now + 100; RApp::pump(); }
    RTicker::run(now);
#else
    if (g_card != lastCard) { lastCard = g_card; g_sink = int32_t(lastCard); }
#endif
    App::tickBuses(now);
    Ticker::run(now);
    if (int32_t(now - nextAir) >= 0) { nextAir = now + 1000; App::pump(); bmpm::PublishAll<Pubs>::sync(); }
  }
}
