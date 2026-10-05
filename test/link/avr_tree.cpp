// The spi example's air sensor composition on an ATmega328P, for its size: the BMP280 machine of OneMenu ItemDef nodes under the same failure edges
// as examples/spi, on the real AVR TWI core (OneChip), with or without the link. examples/spi is ESP8266-only, so this is the AVR balance of the
// same parts (test/link/sizes.sh builds every variant and prints the table).
//   (default)    bare: the machine, its failure edges, the sync pass of the published nodes into a sink; no link, no tree ops, no change store
//   -DLINK       Round 6: role::Link over USART0 with the tree's payload ops (examples/spi/src/tree_ops.h: d v w n), the card as an event
// millis() is Timer0 overflow, as the Arduino core does it; nothing here is the Arduino core.
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <hapi/hapi.h>
#include <chips/avr/avrTwi.h>
#include <chips/avr/avrUart.h>
#include <oneMachine/discover/registry.h>
#include <oneMachine/discover/identify.h>
#include <oneMachine/fail/busedge.h>
#include <oneMachine/fail/world.h>
#ifdef LINK
  #include "../../examples/spi/src/tree_ops.h"
#else
  #include "../../examples/spi/src/bmp280_machine.h"
#endif

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
using M = bmpm::Machine<App, bmpm::Addr<0x76>, Mode>;
using Drivers = Chain<M::Driver>;
struct App : discover::World<App, Twi, Chain<>, M::Entries, 3, discover::I2cScan>, fail::BusEdge<App, Drivers, 1, Mode> {
  static constexpr bool lifecycle = true;
  static void release(RowId) {}
  static void unbindAll() {}
  static void busReset() { Twi::begin(); }
};
using Ticker = fail::Ticks<App, Drivers>;

struct CodeTemp     { static constexpr uint8_t num = 0; ONEMACHINE_STATE_NAME(name, "temp"); };
struct CodePress    { static constexpr uint8_t num = 1; ONEMACHINE_STATE_NAME(name, "press"); };
struct CodeAir      { static constexpr uint8_t num = 2; ONEMACHINE_STATE_NAME(name, "air"); };
struct CodeConfig   { static constexpr uint8_t num = 3; ONEMACHINE_STATE_NAME(name, "air/config"); };
struct CodeCtrlMeas { static constexpr uint8_t num = 4; ONEMACHINE_STATE_NAME(name, "air/ctrl_meas"); };
struct CodeCard     { static constexpr uint8_t num = 5; ONEMACHINE_STATE_NAME(name, "card"); };

volatile int32_t g_sink;
volatile uint32_t g_card;   // a card's UID, written by whatever reads the reader (an RC522 is not part of this image)
#ifdef LINK
#ifndef EVENTS
  #define EVENTS 8   // the event queue's depth (the card)
#endif
using Queue = bmpm::ChangeQueue<EVENTS>;
template<typename Code> static void note(int32_t) { bmpm::StateChanges<>::mark(Code::num); }   // a state code: a pending bit
#else
template<typename Code> static void note(int32_t v) { g_sink = v; }
#endif

using PubTemp     = bmpm::PublishedAt<CodeTemp,     bmpm::PathRef<M, 0>,    oneData::OnSync<&note<CodeTemp>>>;
using PubPress    = bmpm::PublishedAt<CodePress,    bmpm::PathRef<M, 1>,    oneData::OnSync<&note<CodePress>>>;
using PubAir      = bmpm::PublishedAt<CodeAir,      bmpm::PathRef<M, 3>>;
using PubConfig   = bmpm::PublishedAt<CodeConfig,   bmpm::PathRef<M, 3, 0>>;
using PubCtrlMeas = bmpm::PublishedAt<CodeCtrlMeas, bmpm::PathRef<M, 3, 1>>;
using Pubs = Chain<PubTemp, PubPress, PubAir, PubConfig, PubCtrlMeas>;

#ifdef LINK
struct Extra {
  using Codes = Chain<CodeCard>;
  static uint8_t status(uint8_t) { return 0; }
  template<typename P> static void describe(P& put) { const char* s = "  card -> 0/1 notify event ro value u32 status alive\n"; while (*s) put(*s++); }
};
using Ops = bmpm::TreeOps<M, Pubs, Extra, 1, EVENTS>;
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

int main() {
  TCCR0B = (1 << CS01) | (1 << CS00); TIMSK0 = (1 << TOIE0); sei();
  Twi::begin();
  Uart::begin();
  App::discover();
  uint32_t nextAir = 0, lastCard = 0;
  for (;;) {
    const uint32_t now = millis();
#ifdef LINK
    while (Uart::available()) link.feed(Uart::getch(), now);
    Ops::watch();
    if (g_card != lastCard) { lastCard = g_card; Queue::note(CodeCard::num, int32_t(lastCard)); }
#else
    if (g_card != lastCard) { lastCard = g_card; g_sink = int32_t(lastCard); }
#endif
    App::tickBuses(now);
    Ticker::run(now);
    if (int32_t(now - nextAir) >= 0) { nextAir = now + 1000; App::pump(); bmpm::PublishAll<Pubs>::sync(); }
  }
}
