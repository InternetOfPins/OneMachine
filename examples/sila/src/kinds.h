// The example's kinds, written in the shape of role::Switch and role::Light: what the library calls on a kind is name(), params(v), Command and
// Report with each(), apply<E>, sense<E> and safe (role/role.h, role/face.h). Tune and valid are for tuned roles only. params(v) may call
// v(name, int) for a parameter and v.value(raw[, label]), v.scale(field, num, den), v.unit(field, symbol) for what the values mean.
#pragma once
#include <oneMachine/role/role.h>

namespace extra {
  // a field that takes one of a compile-time list of values; a value outside the list is ignored by the device (the previous one stays, the report shows it)
  template<int... Vs> struct Discrete {
    static_assert(sizeof...(Vs) > 0 && sizeof...(Vs) <= 100, "extra: a Discrete has 1 to 100 values");
    static_assert(((Vs >= 0 && Vs <= 65535) && ...), "extra: a Discrete value is a u16");
    ONEMACHINE_STATE_NAME(name, "discrete");
    struct Command { uint16_t value; ONEMACHINE_STATE_NAME(n_value, "value");
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_value(), s.value); } };
    struct Report  { uint16_t value; ONEMACHINE_STATE_NAME(n_value, "value");
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_value(), s.value); } };
    template<class V> static void params(V& v) { (v.value(Vs), ...); }
    static constexpr bool allowed(uint16_t x) { return ((int(x) == Vs) || ...); }
    template<class E> static void apply(const Command& c, Report& r) { if (allowed(c.value)) E::set(c.value); r.value = E::get(); }
    template<class E> static void sense(Report& r) { r.value = E::get(); }
    static void safe(Command& c, const Report& r) { c.value = r.value; }          // hold where it is
  };

  // a field that takes one of a list of labelled values 0, 1, ...; an index outside the list is ignored by the device (the previous one stays, the report shows it)
  template<class... Ls> struct Select {
    static_assert(sizeof...(Ls) > 0 && sizeof...(Ls) <= 100, "extra: a Select has 1 to 100 labels");
    // a label is 1 to 32 of A-Z a-z 0-9 _ - : one word on its description line (python/onemachine refuses any other)
    static constexpr bool label_ok(::state::Name n) {
      unsigned i = 0;
      for (; n.at(i); i++) { char c = n.at(i); if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '_' || c == '-')) return false; }
      return i >= 1 && i <= 32;
    }
    static_assert((label_ok(Ls::name()) && ...), "extra: a Select label is 1 to 32 of A-Z a-z 0-9 _ -");
    ONEMACHINE_STATE_NAME(name, "select");
    struct Command { uint8_t index; ONEMACHINE_STATE_NAME(n_index, "index");
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_index(), s.index); } };
    struct Report  { uint8_t index; ONEMACHINE_STATE_NAME(n_index, "index");
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_index(), s.index); } };
    template<unsigned I, class... Rest> struct Put { template<class V> static void go(V&) {} };
    template<unsigned I, class First, class... Rest> struct Put<I, First, Rest...> {
      template<class V> static void go(V& v) { v.value(int32_t(I), First::name()); Put<I + 1, Rest...>::go(v); } };
    template<class V> static void params(V& v) { Put<0, Ls...>::go(v); }
    template<class E> static void apply(const Command& c, Report& r) { if (c.index < sizeof...(Ls)) E::set(c.index); r.index = E::get(); }
    template<class E> static void sense(Report& r) { r.index = E::get(); }
    static void safe(Command& c, const Report& r) { c.index = r.index; }          // hold where it is
  };

  // a value read from the device, never written: the report is the raw reading; the description gives its scale (presented = raw * Num / Den) and unit symbol
  template<int Num, int Den, class Unit> struct Analog {
    static_assert(Den != 0, "extra: an Analog's denominator is 0");
    ONEMACHINE_STATE_NAME(name, "analog");
    struct Command { template<class Self, class F> static constexpr void each(Self&, F&) {} };
    struct Report  { uint16_t raw; ONEMACHINE_STATE_NAME(n_raw, "raw");
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_raw(), s.raw); } };
    ONEMACHINE_STATE_NAME(n_raw, "raw");
    template<class V> static void params(V& v) { v.scale(n_raw(), Num, Den); v.unit(n_raw(), Unit::name()); }
    template<class E> static void apply(const Command&, Report& r) { r.raw = E::get(); }
    template<class E> static void sense(Report& r) { r.raw = E::get(); }
    static void safe(Command&, const Report&) {}
  };

  // printable ASCII text in a fixed buffer of N bytes, NUL-padded (a text of N characters has no NUL). A command whose buffer is not printable bytes
  // followed only by NULs is ignored by the device (the previous text stays, the report shows it).
  template<unsigned N> struct Text {
    static_assert(N >= 1 && N <= 60, "extra: a Text buffer is 1 to 60 bytes");
    ONEMACHINE_STATE_NAME(name, "text");
    struct Command { uint8_t text[N]; ONEMACHINE_STATE_NAME(n_text, "text");
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_text(), s.text); } };
    struct Report  { uint8_t text[N]; ONEMACHINE_STATE_NAME(n_text, "text");
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_text(), s.text); } };
    template<class V> static void params(V&) {}
    static bool valid(const uint8_t* t) { unsigned i = 0; while (i < N && t[i] >= 0x20 && t[i] <= 0x7e) i++; for (; i < N; i++) if (t[i]) return false; return true; }
    template<class E> static void apply(const Command& c, Report& r) { if (valid(c.text)) E::set(c.text); E::get(r.text); }
    template<class E> static void sense(Report& r) { E::get(r.text); }
    static void safe(Command& c, const Report& r) { for (unsigned i = 0; i < N; i++) c.text[i] = r.text[i]; }   // hold where it is
  };

  // a raw u16 0..Max whose meaning is presented = raw * Num / Den in Unit (the description's scale and unit lines); above Max the device ignores the command
  template<unsigned Max, int Num, int Den, class Unit> struct Scaled {
    static_assert(Den != 0 && Max <= 65535, "extra: a Scaled needs a nonzero denominator and a u16 maximum");
    ONEMACHINE_STATE_NAME(name, "scaled");
    struct Command { uint16_t raw; ONEMACHINE_STATE_NAME(n_raw, "raw");
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_raw(), s.raw); } };
    struct Report  { uint16_t raw; ONEMACHINE_STATE_NAME(n_raw, "raw");
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_raw(), s.raw); } };
    ONEMACHINE_STATE_NAME(n_raw, "raw"); ONEMACHINE_STATE_NAME(p_max, "max");
    template<class V> static void params(V& v) { v(p_max(), int32_t(Max)); v.scale(n_raw(), Num, Den); v.unit(n_raw(), Unit::name()); }
    template<class E> static void apply(const Command& c, Report& r) { if (c.raw <= Max) E::set(c.raw); r.raw = E::get(); }
    template<class E> static void sense(Report& r) { r.raw = E::get(); }
    static void safe(Command& c, const Report& r) { c.raw = r.raw; }                // hold where it is
  };

  // an action: the device fires on every command with `fire` set; nothing is stored or compared. The report is how many times it ran since boot
  // (a u16: it wraps to 0 after 65535).
  struct Action {
    ONEMACHINE_STATE_NAME(name, "action");
    struct Command { uint8_t fire; ONEMACHINE_STATE_NAME(n_fire, "fire");
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_fire(), s.fire); } };
    struct Report  { uint16_t fired; ONEMACHINE_STATE_NAME(n_fired, "fired");
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_fired(), s.fired); } };
    template<class V> static void params(V&) {}
    template<class E> static void apply(const Command& c, Report& r) { if (c.fire) E::fire(); r.fired = E::count(); }
    template<class E> static void sense(Report& r) { r.fired = E::count(); }
    static void safe(Command& c, const Report&) { c.fire = 0; }
  };

  // endpoints with no wiring: variables (one per Id: two roles cannot share an endpoint type), and a counter that an action increments
  template<int Id> struct Var {
    static inline uint16_t v = 0;
    static bool live() { return true; }
    static void set(uint16_t x) { v = x; } static uint16_t get() { return v; }
    template<class Put> static void where(Put& put) { put('r'); put('a'); put('m'); }
  };
  template<unsigned N> struct Buf {
    static inline uint8_t b[N] = {};
    static bool live() { return true; }
    static void set(const uint8_t* t) { for (unsigned i = 0; i < N; i++) b[i] = t[i]; } static void get(uint8_t* t) { for (unsigned i = 0; i < N; i++) t[i] = b[i]; }
    template<class Put> static void where(Put& put) { put('r'); put('a'); put('m'); }
  };
  struct Counter {
    static inline uint16_t n = 0;
    static bool live() { return true; }
    static void fire() { ++n; } static uint16_t count() { return n; }
    template<class Put> static void where(Put& put) { put('r'); put('a'); put('m'); }
  };
}
