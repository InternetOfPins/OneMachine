// The array net on the AVR image (simavr): describes itself, reads a native frame, steps 5 times, writes a frame, prints hex over the UART.
#include "arraynet.h"
#include <oneMachine/state/wire.h>
#include <oneMachine/state/face.h>
#include "array_in.h"
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <string.h>
static void uputc(char c) { while (!(UCSR0A & (1 << UDRE0))) {} UDR0 = c; }
struct Uart { void operator()(char c) { uputc(c); } };
static void puts_(const char* s) { while (*s) uputc(*s++); }
static void hexo(const uint8_t* b, unsigned n) { for (unsigned i = 0; i < n; i++) { uputc("0123456789abcdef"[b[i] >> 4]); uputc("0123456789abcdef"[b[i] & 15]); } uputc('\n'); }
static uint8_t buf[state::wire_size<ArrNet>()];
int main() {
  UBRR0 = 8; UCSR0B = (1 << TXEN0); UCSR0C = 3 << UCSZ00;
  Uart u; state::describe<ArrNet>(u);
  memcpy_P(buf, ARR_IN, sizeof buf);
  ArrNet s{}, nx{}; state::Status st = state::read(s, buf, sizeof buf);
  puts_("status="); uputc(char('0' + int(st))); uputc('\n');
  for (int i = 0; i < 5; i++) { nx.step(s); s = nx; }
  state::write(s, buf); puts_("OUT "); hexo(buf, sizeof buf); puts_("END\n");
  cli(); sleep_cpu();
}
