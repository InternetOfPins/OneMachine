// What the wire format and the face cost on AVR, on the fib net (7 bytes of state, 11 of frame), against hand-written twins.
//   -DHAND_BASE the same net, hand-indexed, nothing added
//   (none)   the net stepped in a loop, its fields published                 -DINC     the three headers included, nothing called
//   -DWRITE  + one typed frame written            -DHAND_WRITE  + the same frame written by hand (header bytes, then the state bytes)
//   -DREAD   + one typed frame read (checks, then stores)     -DHAND_READ  + the same read by hand (header, length, copy)
//   -DDESCRIBE  + the self-description put to a sink
#include "net.h"
#include "flat.h"
#ifdef INC
  #include <oneMachine/state/wire.h>
  #include <oneMachine/state/face.h>
#endif
#if defined(WRITE) || defined(READ) || defined(DESCRIBE)
  #include <oneMachine/state/wire.h>
  #include <oneMachine/state/face.h>
#endif
volatile int16_t in_a, in_b; volatile int16_t out_a, out_b; volatile uint16_t out_steps; volatile uint8_t out_odd;
volatile uint8_t sink; volatile uint8_t inbyte;
static const uint32_t HASH = 0xa47d01d3u;
#if defined(HAND_WRITE) || defined(HAND_READ) || defined(HAND_BASE)
  #define HANDNET
#endif
#ifdef HANDNET
// portable and straight-line: every field through its value, little-endian by shifts, in frame order (b, a, steps, odd)
static void put16(uint8_t* o, uint16_t x) { o[0] = uint8_t(x); o[1] = uint8_t(x >> 8); }
static void hand_write(const Flat& f, uint8_t* o) {
  o[0] = uint8_t(HASH); o[1] = uint8_t(HASH >> 8); o[2] = uint8_t(HASH >> 16); o[3] = uint8_t(HASH >> 24);
  put16(o + 4, uint16_t(rd16(f.v + F_B))); put16(o + 6, uint16_t(rd16(f.v + F_A))); put16(o + 8, rdu16(f.v + F_STEPS)); o[10] = f.v[F_ODD]; }
static bool hand_read(Flat& f, const uint8_t* in, unsigned n) {
  if (n < 4) return false;
  if (in[0] != uint8_t(HASH) || in[1] != uint8_t(HASH >> 8) || in[2] != uint8_t(HASH >> 16) || in[3] != uint8_t(HASH >> 24)) return false;
  if (n != 11) return false;
  wr16(f.v + F_B, int16_t(uint16_t(in[4] | (in[5] << 8)))); wr16(f.v + F_A, int16_t(uint16_t(in[6] | (in[7] << 8))));
  wru16(f.v + F_STEPS, uint16_t(in[8] | (in[9] << 8))); f.v[F_ODD] = in[10];
  return true; }
#endif
#ifdef DESCRIBE
struct Sink { void operator()(char c) { sink = uint8_t(c); } };
#endif
int main() {
#ifdef HANDNET
  Flat p{}, n{}; wr16(p.v + F_A, in_a); wr16(p.v + F_B, in_b);
  for (;;) { flat_step(p, n); p = n; out_a = rd16(p.v + F_A); out_b = rd16(p.v + F_B); out_steps = rdu16(p.v + F_STEPS); out_odd = p.v[F_ODD];
  #ifdef HAND_WRITE
    uint8_t b[11]; hand_write(p, b); for (uint8_t i = 0; i < 11; i++) sink = b[i];
  #endif
  #ifdef HAND_READ
    uint8_t b[11]; for (uint8_t i = 0; i < 11; i++) b[i] = inbyte; if (hand_read(p, b, 11)) sink = 1;
  #endif
  }
#else
  Net p{}, n{}; state::get<FibA>(p).a = in_a; state::get<FibB>(p).b = in_b;
  for (;;) { n.step(p); p = n; out_a = state::get<FibA>(p).a; out_b = state::get<FibB>(p).b; out_steps = state::get<Watch>(p).steps; out_odd = state::get<Watch>(p).odd;
  #ifdef WRITE
    uint8_t b[state::wire_size<Net>()]; state::write(p, b); for (uint8_t i = 0; i < sizeof b; i++) sink = b[i];
  #endif
  #ifdef READ
    uint8_t b[state::wire_size<Net>()]; for (uint8_t i = 0; i < sizeof b; i++) b[i] = inbyte; if (state::read(p, b, sizeof b) == state::Status::Ok) sink = 1;
  #endif
  #ifdef DESCRIBE
    Sink s; state::describe<Net>(s);
  #endif
  }
#endif
}
