// The cost of role routing on an ATmega328P (avr-g++ -Os, simavr at 16 MHz): one M::apply of six light roles on a PCA9685 behind channel 2
// of a mux, the bus transfer a stub (the same noinline function in every variant), so the numbers are the role layer and its routing.
//   VARIANT=0  Fixed endpoints: the device's place written into the type (the floor)
//   VARIANT=1  Found endpoints: bound when discovery finds the device, a liveness check and W::route per request
//   VARIANT=2  Pinned endpoints: looked up by path after discovery (pin), then the same per request
//   VARIANT=3  Found, plus the consumer side: role::Link and the three descriptions (flash only; not called in the timing)
// Prints "APPLY <cycles>" (and "BIND <cycles>" for the lookup after a scan) over the UART, then END.
#include <oneMachine/role/role.h>
#include <oneMachine/role/kinds.h>
#include <oneMachine/role/found.h>
#include <oneMachine/role/route.h>
#if VARIANT == 3
#include <oneMachine/role/link.h>
#endif
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

volatile uint8_t sink;
static uint8_t muxAt = 0xFF;                                   // a bridge driver keeps its current channel: a select is a write only on change
static inline void muxSelect(uint8_t ch) { if (muxAt != ch) { muxAt = ch; sink = 0x70; sink = uint8_t(1u << ch); } }
static uint16_t last[16];
struct Pca {
  static constexpr uint16_t top = 4095;
  ONEMACHINE_STATE_NAME(name, "pca9685");
  __attribute__((noinline)) static void set(uint8_t a, uint8_t u, uint16_t v) { sink = a; sink = uint8_t(6 + 4 * u); sink = 0; sink = 0; sink = uint8_t(v); sink = uint8_t(v >> 8); last[u & 15] = v; }
  static uint16_t get(uint8_t, uint8_t u) { return last[u & 15]; }
};
struct World {
  static inline discover::Registry<uint8_t, 12> reg;
  static void route(discover::RowId bus) { const auto& b = reg.rows[bus]; if (bus != discover::rootRow && b.isBus) muxSelect(uint8_t(b.busId)); }
};
template<uint8_t U> struct Fixed {
  static constexpr uint16_t top = 4095;
  static bool live() { return true; }
  static void set(uint16_t v) { muxSelect(2); Pca::set(0x40, U, v); }
  static uint16_t get() { muxSelect(2); return Pca::get(0x40, U); }
  template<class P> static void where(P& put) { put('f'); }
};
#if VARIANT == 0
template<uint8_t U> using At = Fixed<U>;
#elif VARIANT == 2
template<uint8_t U> using At = role::Pinned<World, role::path(0x70, 2, 0x40), Pca, U>;
#else
template<uint8_t U> using At = role::Found<World, Pca, 0x40, discover::Behind<0x70, 2>, U>;
#endif
struct L0 { ONEMACHINE_STATE_NAME(name, "l0"); }; struct L1 { ONEMACHINE_STATE_NAME(name, "l1"); }; struct L2 { ONEMACHINE_STATE_NAME(name, "l2"); };
struct L3 { ONEMACHINE_STATE_NAME(name, "l3"); }; struct L4 { ONEMACHINE_STATE_NAME(name, "l4"); }; struct L5 { ONEMACHINE_STATE_NAME(name, "l5"); };
using K = role::Light<4095>;
using M = role::Machine<role::Role<L0, K, At<0>>, role::Role<L1, K, At<1>>, role::Role<L2, K, At<2>>,
                        role::Role<L3, K, At<3>>, role::Role<L4, K, At<4>>, role::Role<L5, K, At<5>>>;

static M::Command cmd{};
static M::Report rep{};
__attribute__((noinline)) void applyAll() { M::apply(cmd, rep); }

static void scan() {                                           // what discover() leaves on the pilot bus, with found()'s binder call per device
  auto& reg = World::reg; reg.reset(); M::unbind();
  M::bind(reg.add(0x68, nullptr, discover::rootRow, false), (void*)nullptr);   // an RTC (a driver no role wants)
  M::bind(reg.add(0x41, nullptr, discover::rootRow, false), (Pca*)nullptr);    // another PCA9685 on the root bus
  discover::RowId bridge = reg.add(0x70, nullptr, discover::rootRow, false);
  discover::RowId ch0 = reg.count; for (uint8_t c = 0; c < 4; c++) reg.add(c, nullptr, bridge, true);
  M::bind(reg.add(0x40, nullptr, discover::RowId(ch0 + 3), false), (Pca*)nullptr);   // a PCA9685 at 0x40 behind channel 3: not ours
  M::bind(reg.add(0x40, nullptr, discover::RowId(ch0 + 2), false), (Pca*)nullptr);   // ours
}

#if VARIANT == 3
struct Out { static void put(uint8_t b) { while (!(UCSR0A & (1 << UDRE0))) {} UDR0 = b; } };
static role::Link<M, Out> link(rep, 500);
#endif
static void uputc(char c) { while (!(UCSR0A & (1 << UDRE0))) {} UDR0 = c; }
static void puts_(const char* s) { while (*s) uputc(*s++); }
static void putu(uint16_t v) { char b[6]; uint8_t k = 0; do { b[k++] = char('0' + v % 10); v /= 10; } while (v); while (k) uputc(b[--k]); }

int main() {
  UBRR0 = 8; UCSR0B = (1 << TXEN0) | (1 << RXEN0); UCSR0C = 3 << UCSZ00;
  TCCR1A = 0; TCCR1B = 1 << CS10;                              // Timer1 at the CPU clock
  TCNT1 = 0; scan(); M::pin(); uint16_t b = TCNT1;
  state::get<L0>(cmd).level = 100; state::get<L1>(cmd).level = 200; state::get<L2>(cmd).level = 300;
  state::get<L3>(cmd).level = 400; state::get<L4>(cmd).level = 500; state::get<L5>(cmd).level = 4095;
  muxAt = 0xFF; TCNT1 = 0; applyAll(); uint16_t a = TCNT1;
  puts_("BIND "); putu(b); uputc('\n'); puts_("APPLY "); putu(a); uputc('\n');
  bool ok = state::get<L5>(rep).live && state::get<L5>(rep).level == 4095 && last[5] == 4095;
  puts_(ok ? "OK\n" : "WRONG\n");
#if VARIANT == 3
  while (UCSR0A & (1 << RXC0)) link.feed(UDR0, 0);             // keep the link in the image; nothing arrives in the timing run
#endif
  puts_("END\n");
  cli(); sleep_cpu();
}
