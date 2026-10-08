#pragma once
// oneMachine/role/face.h -- the machine description: what the roles mean. A consumer reads it once, with the command and report
// descriptions (state::describe), and needs nothing else.
//
//   machine 1
//   hash 1f2e3d4c                   FNV-1a over the bytes of every line below except the `at` lines
//   ref https://...                 role/ref.h, in item order
//   role x axis                     one per role: name, kind
//   param x steps_mm 80             one per kind parameter, signed decimal: the firmware's value (for a tuned role, its limit)
//   value x 50 half                 a kind may print: a value the role takes, with an optional label (v.value(raw, label))
//   scale x level 5 1023            presented = raw * num / den, for a field of the role (v.scale(field, num, den))
//   unit x level V                  the unit symbol of a field of the role (v.unit(field, symbol))
//   tune x                          the role is role::Tuned: its parameters' current values are the link's tuning state (ops T, G, S)
//   at x sim.stepdir(0)             where the device routes it: for a person reading the description, not for a consumer.
//                                   Not in the hash: rewiring changes nothing a consumer depends on.
#include <oneMachine/state/face.h>
#include "role.h"
#include "ref.h"

namespace role {
  template<class P> void put_int(P& put, int32_t v) {
    if (v < 0) { put('-'); state::put_dec(put, uint32_t(0) - uint32_t(v)); } else state::put_dec(put, uint32_t(v)); }
  template<class P> void put_str(P& put, const char* s) { while (*s) put(*s++); }   // RAM text (a RefFrom string)
#define ONEMACHINE_ROLE_TEXT(id, text) static constexpr char id##_s[] ONEMACHINE_STATE_ROM = text
#define ONEMACHINE_ROLE_PUT(put, id) state::put_name(put, state::Name(id##_s))

  template<class T> struct IsRef : std::false_type {};
  template<class T> struct IsRef<Ref<T>> : std::true_type {};
  template<class T> struct IsRefFrom : std::false_type {};
  template<class T> struct IsRefFrom<RefFrom<T>> : std::true_type {};

  template<class P> struct Params {
    P& put; state::Name role;
    void operator()(state::Name n, int32_t v) {
      ONEMACHINE_ROLE_TEXT(kParam, "param "); ONEMACHINE_ROLE_PUT(put, kParam); state::put_name(put, role); put(' '); state::put_name(put, n); put(' '); put_int(put, v); put('\n'); }
    // a value the role takes, with an optional label: presented to a person or a consumer as the label
    void value(int32_t raw) {
      ONEMACHINE_ROLE_TEXT(kValue, "value "); ONEMACHINE_ROLE_PUT(put, kValue); state::put_name(put, role); put(' '); put_int(put, raw); put('\n'); }
    void value(int32_t raw, state::Name label) {
      ONEMACHINE_ROLE_TEXT(kValue, "value "); ONEMACHINE_ROLE_PUT(put, kValue); state::put_name(put, role); put(' '); put_int(put, raw); put(' '); state::put_name(put, label); put('\n'); }
    // presented = raw * num / den, for a field of the role
    void scale(state::Name field, int32_t num, int32_t den) {
      ONEMACHINE_ROLE_TEXT(kScale, "scale "); ONEMACHINE_ROLE_PUT(put, kScale); state::put_name(put, role); put(' '); state::put_name(put, field); put(' '); put_int(put, num); put(' '); put_int(put, den); put('\n'); }
    // the unit symbol of a field of the role
    void unit(state::Name field, state::Name symbol) {
      ONEMACHINE_ROLE_TEXT(kUnit, "unit "); ONEMACHINE_ROLE_PUT(put, kUnit); state::put_name(put, role); put(' '); state::put_name(put, field); put(' '); state::put_name(put, symbol); put('\n'); }
  };
  template<class P> struct Lines {
    P& put; bool at;
    template<class I> void item() {
      if constexpr (IsRole<I>::value) {
        state::Name r = I::tag::name();
        ONEMACHINE_ROLE_TEXT(kRole, "role "); ONEMACHINE_ROLE_PUT(put, kRole); state::put_name(put, r); put(' '); state::put_name(put, I::kind::name()); put('\n');
        Params<P> ps{put, r}; I::kind::params(ps);
        if constexpr (IsTunedRole<I>::value) { ONEMACHINE_ROLE_TEXT(kTune, "tune "); ONEMACHINE_ROLE_PUT(put, kTune); state::put_name(put, r); put('\n'); }
        if (at) { ONEMACHINE_ROLE_TEXT(kAt, "at "); ONEMACHINE_ROLE_PUT(put, kAt); state::put_name(put, r); put(' '); I::endpoint::where(put); put('\n'); }
      } else if constexpr (IsRef<I>::value) {
        using T = typename Shape<I>::text; ONEMACHINE_ROLE_TEXT(kRef, "ref "); ONEMACHINE_ROLE_PUT(put, kRef); state::put_name(put, T::name()); put('\n');
      } else if constexpr (IsRefFrom<I>::value) {
        const char* s = Shape<I>::src::ref();
        if (s && *s) { ONEMACHINE_ROLE_TEXT(kRef, "ref "); ONEMACHINE_ROLE_PUT(put, kRef); put_str(put, s); put('\n'); }
      }
    }
    template<class T> struct Shape;
    template<class T> struct Shape<Ref<T>> { using text = T; };
    template<class T> struct Shape<RefFrom<T>> { using src = T; };
  };
  // every line is written through one sink type (a function pointer and its context), so the walk is compiled once for the hash and once
  // for the text, whatever the caller's put is
  struct Sink { void (*f)(void*, char); void* ctx; void operator()(char c) { f(ctx, c); } };
  template<class P> Sink sink(P& put) { return Sink{[](void* x, char c) { (*static_cast<P*>(x))(c); }, &put}; }
  struct Fnv { uint32_t h = 2166136261u; void operator()(char c) { h = state::fnv(h, uint8_t(c)); } };

  template<class M> uint32_t machine_hash() { Fnv f; Sink s = sink(f); Lines<Sink> l{s, false}; M::each(l); return f.h; }

  template<class M, class P> void describe(P& p) {
    Sink put = sink(p);
    uint32_t h = machine_hash<M>();
    ONEMACHINE_ROLE_TEXT(kHead, "machine 1\nhash "); ONEMACHINE_ROLE_PUT(put, kHead);
    for (int i = 28; i >= 0; i -= 4) { uint8_t d = uint8_t((h >> i) & 15); put(char(d < 10 ? '0' + d : 'a' + d - 10)); }
    put('\n');
    Lines<Sink> l{put, true}; M::each(l);
  }
}
#undef ONEMACHINE_ROLE_TEXT
#undef ONEMACHINE_ROLE_PUT
