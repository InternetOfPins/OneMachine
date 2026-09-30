#pragma once
// oneMachine/state/face.h -- text faces of a state composition; the only place the names are read at run time.
//   state::describe<R>(put)   the self-description: what a peer needs to read a frame (and to compute its hash)
//   state::json(res, put)     the state as one JSON object {"layer":{"field":value,...},...}
// `put` is any callable taking a char. On AVR the names and the fixed text are read from flash.
//
// describe:            state 1
//                      hash a47d01d3            schema_v, 8 hex digits
//                      size 11                  frame size in bytes, the 4 hash bytes included
//                      layer fibB               one per layer, chain order
//                      fibB/b i16               one per field: layer/field type; types: bool u8 i8 u16 i16 u32 i32 u64 i64, an array as u8[60]
#include "state.h"
#include "wire.h"   // describe reports the frame size

namespace state {
  template<class P> void put_name(P& put, Name n) { for (unsigned i = 0; ; i++) { char c = n.rom(i); if (!c) break; put(c); } }
#define ONEMACHINE_STATE_TEXT(id, text) static constexpr char id##_s[] ONEMACHINE_STATE_ROM = text
  template<class P, class V> void put_dec(P& put, V v) {
    char b[20]; unsigned k = 0; do { b[k++] = char('0' + v % 10); v /= 10; } while (v);
    while (k) put(b[--k]);
  }
  template<class P> void put_type(P& put, uint8_t code) {     // from the schema's type byte
    if (code == 1) { put('b'); put('o'); put('o'); put('l'); return; }
    put(code & 1 ? 'i' : 'u');
    switch (code >> 1) { case 1: put('8'); break; case 2: put('1'); put('6'); break; case 4: put('3'); put('2'); break; default: put('6'); put('4'); }
  }

  template<class P> struct Describe {
    ONEMACHINE_STATE_TEXT(kLayer, "layer ");
    P& put; Name cur{nullptr};
    void layer(Name n) { put_name(put, Name(kLayer_s)); put_name(put, n); put('\n'); cur = n; }
    template<class T> void operator()(Name n, const T&) {
      put_name(put, cur); put('/'); put_name(put, n); put(' ');
      put_type(put, wcode<typename Extent<T>::Elem>());
      if constexpr (Extent<T>::array) { put('['); put_dec(put, unsigned(Extent<T>::n)); put(']'); }
      put('\n'); }
  };

  template<class R, class P> void describe(P& put) {
    ONEMACHINE_STATE_TEXT(kHead, "state 1\nhash ");
    ONEMACHINE_STATE_TEXT(kSize, "\nsize ");
    put_name(put, Name(kHead_s));
    uint32_t h = schema_v<R>;
    for (int i = 28; i >= 0; i -= 4) { uint8_t d = uint8_t((h >> i) & 15); put(char(d < 10 ? '0' + d : 'a' + d - 10)); }
    put_name(put, Name(kSize_s)); put_dec(put, wire_size<R>()); put('\n');
    static constexpr R r{};
    Describe<P> d{put};
    r.each(d);
  }

  template<class P> struct Json {
    P& put; bool inLayer = false, firstLayer = true, firstField = true;
    void quoted(Name n) { put('"'); put_name(put, n); put('"'); }
    void layer(Name n) { if (inLayer) put('}'); if (!firstLayer) put(','); firstLayer = false; inLayer = true; firstField = true; quoted(n); put(':'); put('{'); }
    template<class E> void number(const E& v) {
      if constexpr (std::is_same<E, bool>::value) { const char* t = v ? "true" : "false"; while (*t) put(*t++); }
      else {
        uint64_t m = uint64_t(v);
        if constexpr (wcode<E>() & 1) { if (v < 0) { put('-'); m = uint64_t(0) - m; } }
        put_dec(put, m);
      }
    }
    template<class T> void operator()(Name n, const T& v) {
      if (!firstField) { put(','); }
      firstField = false; quoted(n); put(':');
      if constexpr (Extent<T>::array) { put('['); for (unsigned i = 0; i < Extent<T>::n; i++) { if (i) put(','); number(v[i]); } put(']'); }
      else number(v);
    }
  };
  template<class R, class P> void json(const R& r, P& put) { Json<P> j{put}; put('{'); r.each(j); if (j.inLayer) put('}'); put('}'); }
}
#undef ONEMACHINE_STATE_TEXT
