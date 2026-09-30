// The AVR peer (ATmega328p, simavr): describes itself, reads the frames of wire_in.h, steps, writes a frame, prints hex over the UART.
// The test scaffolding here (strings, hex) is not the library; the library's cost is measured separately (avr_cost.cpp).
#include "net2.h"
#include <oneMachine/state/wire.h>
#include <oneMachine/state/face.h>
#include "wire_in.h"
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <string.h>

static void uputc(char c) { while (!(UCSR0A & (1 << UDRE0))) {} UDR0 = c; }
struct Uart { void operator()(char c) { uputc(c); } };
static void uinit() { UBRR0 = 8; UCSR0B = (1 << TXEN0); UCSR0C = 3 << UCSZ00; }
static void puts_(const char* s) { while (*s) uputc(*s++); }
static void hexo(const uint8_t* b, unsigned n) { for (unsigned i = 0; i < n; i++) { uputc("0123456789abcdef"[b[i] >> 4]); uputc("0123456789abcdef"[b[i] & 15]); } uputc('\n'); }

static uint8_t buf[state::wire_size<Net2>() + 1];
static void scenario(const char* tag, const uint8_t* rom, unsigned n, bool run) {
  memcpy_P(buf, rom, IN_N);
  Net2 s{};                                                    // a zero state: a refused frame must leave exactly this
  state::Status st = state::read(s, buf, n);
  uint8_t out[state::wire_size<Net2>()];
  puts_(tag); puts_(" status="); uputc(char('0' + int(st))); uputc('\n');
  state::write(s, out); puts_(tag); puts_(" READ "); hexo(out, sizeof out);
  if (run && st == state::Status::Ok) {
    Net2 nx{};
    for (int i = 0; i < STEPS; i++) { nx.step(s); s = nx; }
    state::write(s, out); puts_(tag); puts_(" OUT "); hexo(out, sizeof out);
  }
}
int main() {
  uinit();
  Uart u; state::describe<Net2>(u);
  scenario("GOOD", IN_GOOD, IN_N, true);
  scenario("BADHASH", IN_BADHASH, IN_N, false);
  scenario("BADBOOL", IN_BADBOOL, IN_N, false);
  scenario("SHORT", IN_GOOD, IN_N - 1, false);
  scenario("LONG", IN_GOOD, IN_N + 1, false);
  puts_("END\n");
  cli(); sleep_cpu();
}
