#pragma once
// oneMachine/state/wire.h -- binary frames of a state composition.
//
// Frame = schema hash (u32) then every field, in chain order (the last-listed layer first), each at its declared width, little-endian
// (bool is one byte, 0 or 1). Nothing depends on the machine's int width, padding or endianness.
//   state::wire_size<R>()             bytes of a frame, constexpr
//   state::write(res, out)            writes a frame, returns the end pointer
//   state::read(res, in, n)           Status::Ok and res replaced, or a refusal with res untouched:
//                                  BadLength (n < 4, or n is not the frame size of R), BadHash (another composition), BadValue (a bool byte other than 0 or 1)
// Included but not called, nothing is emitted.
#include "state.h"

namespace state {
  enum class Status : uint8_t { Ok = 0, BadHash, BadLength, BadValue };

  template<unsigned N> struct UInt;
  template<> struct UInt<1> { using type = uint8_t; };
  template<> struct UInt<2> { using type = uint16_t; };
  template<> struct UInt<4> { using type = uint32_t; };
  template<> struct UInt<8> { using type = uint64_t; };

  struct Sizer {
    unsigned n = 0;
    constexpr void layer(Name) {}
    template<class T> constexpr void operator()(Name, const T&) { n += sizeof(T); }
  };
  template<class R> constexpr unsigned wire_size() { R r{}; Sizer s; r.each(s); return 4 + s.n; }

  struct Writer {
    uint8_t* p;
    void layer(Name) {}
    template<class E> void one(const E& v) {
      typename UInt<sizeof(E)>::type u = typename UInt<sizeof(E)>::type(v);
      for (unsigned i = 0; i < sizeof(E); i++) { *p++ = uint8_t(u); u = typename UInt<sizeof(E)>::type(u >> 8); }
    }
    template<class T> void operator()(Name, const T& v) {
      if constexpr (Extent<T>::array) { for (unsigned i = 0; i < Extent<T>::n; i++) one(v[i]); }
      else one(v);
    }
  };
  template<class R> uint8_t* write(const R& r, uint8_t* out) {
    Writer w{out};
    uint32_t h = schema_v<R>;
    for (unsigned i = 0; i < 4; i++) { *w.p++ = uint8_t(h); h >>= 8; }
    r.each(w);
    return w.p;
  }

  // two passes: the first only looks at the bool bytes (and vanishes for a composition without a bool), the second stores
  template<bool Apply> struct Reader {
    const uint8_t* p; bool ok;
    void layer(Name) {}
    template<class E> void one(E& v) {
      if constexpr (!Apply) {
        if constexpr (std::is_same<E, bool>::value) { if (*p > 1) ok = false; }
        p += sizeof(E);
      } else {
        typename UInt<sizeof(E)>::type u = 0;
        for (unsigned i = sizeof(E); i-- > 0; ) u = typename UInt<sizeof(E)>::type((u << 8) | p[i]);   // constant shifts: cheap on 8-bit targets
        p += sizeof(E);
        if constexpr (std::is_same<E, bool>::value) v = (u != 0); else v = E(u);
      }
    }
    template<class T> void operator()(Name, T& v) {
      if constexpr (Extent<T>::array) { for (unsigned i = 0; i < Extent<T>::n; i++) one(v[i]); }
      else one(v);
    }
  };
  template<class R> Status read(R& r, const uint8_t* in, unsigned n) {
    if (n < 4) return Status::BadLength;
    uint32_t h = 0; for (unsigned i = 4; i-- > 0; ) h = (h << 8) | in[i];
    if (h != schema_v<R>) return Status::BadHash;
    if (n != wire_size<R>()) return Status::BadLength;
    Reader<false> check{in + 4, true}; r.each(check);
    if (!check.ok) return Status::BadValue;
    Reader<true> store{in + 4, true}; r.each(store);
    return Status::Ok;
  }
}
