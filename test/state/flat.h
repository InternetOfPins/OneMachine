// The same net, hand-indexed: one byte array, offsets written by hand, prev and next as two arrays.
#pragma once
#include <stdint.h>
struct Flat { uint8_t v[7]; };                       // [0,1] b   [2,3] a   [4,5] steps   [6] odd
enum : unsigned { F_B = 0, F_A = 2, F_STEPS = 4, F_ODD = 6 };
#ifdef FLAT_CAST
  typedef int16_t  __attribute__((may_alias)) i16a;
  typedef uint16_t __attribute__((may_alias)) u16a;
  static inline int16_t  rd16(const uint8_t* p)  { return *reinterpret_cast<const i16a*>(p); }
  static inline uint16_t rdu16(const uint8_t* p) { return *reinterpret_cast<const u16a*>(p); }
  static inline void wr16(uint8_t* p, int16_t x)   { *reinterpret_cast<i16a*>(p) = x; }
  static inline void wru16(uint8_t* p, uint16_t x) { *reinterpret_cast<u16a*>(p) = x; }
#else
  static inline int16_t  rd16(const uint8_t* p)  { return int16_t(uint16_t(p[0] | (p[1] << 8))); }
  static inline uint16_t rdu16(const uint8_t* p) { return uint16_t(p[0] | (p[1] << 8)); }
  static inline void wr16(uint8_t* p, int16_t x)   { p[0] = uint8_t(x); p[1] = uint8_t(uint16_t(x) >> 8); }
  static inline void wru16(uint8_t* p, uint16_t x) { p[0] = uint8_t(x); p[1] = uint8_t(x >> 8); }
#endif
static inline void flat_step(const Flat& p, Flat& n) {      // last layer first: b, a, then the observer
  wr16(n.v + F_B, int16_t(rd16(p.v + F_A) + rd16(p.v + F_B)));
  wr16(n.v + F_A, rd16(p.v + F_B));
  wru16(n.v + F_STEPS, uint16_t(rdu16(p.v + F_STEPS) + 1));
  n.v[F_ODD] = uint8_t((rd16(n.v + F_A) + rd16(n.v + F_B)) & 1);
}
