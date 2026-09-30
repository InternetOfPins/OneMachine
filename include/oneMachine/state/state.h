#pragma once
// oneMachine/state/state.h -- per-layer typed state for HAPI compositions.
//
// A layer is state::Layer<Tag, Slot, Body>:
//   Tag   a type declaring `ONEMACHINE_STATE_NAME(name, "text")` (a constexpr function `name()`; the text is a flash array, no object): the layer's identity (unique among the layers of one composition).
//   Slot  a plain struct of fixed-width fields, with one `each` naming them:
//           ONEMACHINE_STATE_NAME(n_x, "x"); ONEMACHINE_STATE_NAME(n_y, "y");
//           template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_x(), s.x); f(n_y(), s.y); }
//         Field types are fixed-width integers or bool.
//   Body  `static Slot run(const Below& below, const Prev& prev)`: returns the layer's new slot. `below` is the next state of the layers
//         listed after this one (already computed), `prev` the whole previous state; the layer's own next is not in either.
//         Omitted (void): the layer holds its previous value.
//
// The slots are hapi::Slot (hapi/slots.h); this header is the contract on top of them: names, the step, the schema hash.
// hapi::APIOf<state::API, L1, L2, ...>::Res is the whole state: one object, statically sized, no heap. An empty slot costs nothing.
//   get<Tag>(res)        the slot of a layer
//   next.step(prev)      runs the layers last-listed first; each returns its own slot, the framework stores it.
//                        Effects belong after the step, from the finished `next`.
//   res.each(visitor)    visitor.layer(name), then visitor(field name, field) for every field, in chain order; names are state::Name
//   ONEMACHINE_STATE_PIN(Res, hash)    compile-time check of schema_v against a pinned value, on every target that builds the composition
//   schema_v<Res>        32-bit FNV-1a over layer names, field names and field types (width, signedness, bool), in chain order, at compile time
//
// Names live in flash on AVR (ONEMACHINE_STATE_ROM = PROGMEM) and cost nothing unless a face reads them (Name::rom).
#ifdef ONEMACHINE_STATE_EACH_INLINE
  #define HAPI_SLOT_EACH_INLINE ONEMACHINE_STATE_EACH_INLINE
#endif
#include <hapi/hapi.h>
#include <stdint.h>
#ifdef __AVR__
  #include <avr/pgmspace.h>
  #ifndef ONEMACHINE_STATE_ROM
    #define ONEMACHINE_STATE_ROM PROGMEM
  #endif
#else
  #ifndef ONEMACHINE_STATE_ROM
    #define ONEMACHINE_STATE_ROM
  #endif
#endif

#ifndef ONEMACHINE_STATE_EACH_INLINE
  #define ONEMACHINE_STATE_EACH_INLINE   // e.g. [[gnu::always_inline]]: a size policy for the walks (read, write, describe): larger code, fewer calls
#endif
#ifndef ONEMACHINE_STATE_MAX_NAMES
  #define ONEMACHINE_STATE_MAX_NAMES 32   // layers per composition, and fields per layer, the name checks can hold
#endif

namespace state {
  // a name: a pointer to a constexpr char array (in flash on AVR). `at` reads it in a constant expression, `rom` at run time.
  struct Name {
    const char* p;
    constexpr explicit Name(const char* s) : p(s) {}
    constexpr char at(unsigned i) const { return p[i]; }
    char rom(unsigned i) const {
#ifdef __AVR__
      return char(pgm_read_byte(p + i));
#else
      return p[i];
#endif
    }
  };
#define ONEMACHINE_STATE_NAME(id, text) static constexpr char id##_s[] ONEMACHINE_STATE_ROM = text; static constexpr ::state::Name id() { return ::state::Name(id##_s); }

  template<class Tag, class R> using HasTag = hapi::HasSlot<Tag, R>;
  template<class T, class = void> struct HasName : std::false_type {};
  template<class T> struct HasName<T, std::void_t<decltype(T::name())>> : std::is_same<std::remove_cv_t<decltype(T::name())>, Name> {};

  template<class Tag, class R> constexpr decltype(auto) get(R& r) {
    static_assert(HasTag<Tag,R>::value, "state: no layer with that tag in this state (a layer sees itself and the layers below it, in next)");
    return hapi::slot<Tag>(r);
  }

  constexpr bool streq(Name a, Name b) { unsigned i = 0; while (a.at(i) && a.at(i) == b.at(i)) ++i; return a.at(i) == b.at(i); }
  struct Names {
    const char* n[ONEMACHINE_STATE_MAX_NAMES]{}; unsigned k = 0; bool dup = false, over = false;
    constexpr bool has(Name s) const { for (unsigned i = 0; i < k; i++) if (streq(Name(n[i]), s)) return true; return false; }
    constexpr void add(Name s) { if (has(s)) dup = true; if (k < ONEMACHINE_STATE_MAX_NAMES) n[k++] = s.p; else over = true; }
  };
  struct FieldNames : Names {
    constexpr void layer(Name) {}
    template<class T> constexpr void operator()(Name s, T&) { add(s); }
  };
  // one walk over the layers below: is a name already taken, and how many layers are there (linear in the layers; done once per layer)
  struct LayerScan { Name target; bool found = false; unsigned count = 0;
    constexpr void layer(Name n) { if (streq(n, target)) found = true; ++count; }
    template<class T> constexpr void operator()(Name, T&) {} };
  template<class R> constexpr LayerScan layer_scan(Name s) { R r{}; LayerScan v{s}; r.each(v); return v; }
  template<class R, class Tag> inline constexpr LayerScan layer_scan_v = layer_scan<R>(Tag::name());
  template<class S> constexpr bool fields_fit() { S s{}; FieldNames v; S::each(s, v); return !v.over; }
  template<class S> constexpr bool fields_unique() { S s{}; FieldNames v; S::each(s, v); return !v.dup; }

  constexpr uint32_t fnv(uint32_t h, uint8_t b) { return (h ^ b) * 16777619u; }
  constexpr uint32_t fnvs(uint32_t h, Name s) { for (unsigned i = 0; s.at(i); i++) h = fnv(h, uint8_t(s.at(i))); return h; }
  // a field is a scalar or a fixed-size array of one (`uint8_t band[60]`)
  template<class T> struct Extent { static constexpr bool array = false; using Elem = T; static constexpr unsigned n = 1; };
  template<class T, decltype(sizeof(0)) N> struct Extent<T[N]> { static constexpr bool array = true; using Elem = T; static constexpr unsigned n = unsigned(N); };

  // the type byte of the schema: bool 1, otherwise 2 * width in bytes + 1 if signed (u8 2, i8 3, u16 4, i16 5, u32 8, i32 9, u64 16, i64 17);
  // an array of one of these has the byte with 0x80 added, and its element count follows in the hash
  template<class T> constexpr uint8_t wcode_() {
    using U = std::remove_cv_t<T>;
    static_assert(!std::is_same<U, char>::value, "state: use int8_t/uint8_t: the signedness of char differs between machines");
    static_assert(sizeof(U) == 1 || sizeof(U) == 2 || sizeof(U) == 4 || sizeof(U) == 8, "state: a field must be a 1, 2, 4 or 8 byte integer or bool");
    static_assert(std::is_same<U, bool>::value || U(1) / U(2) == U(0), "state: a field must be an integer or bool (no floating point)");
    return std::is_same<U, bool>::value ? uint8_t(1) : uint8_t(sizeof(U) * 2 + (U(-1) < U(0) ? 1 : 0));
  }
  template<class T> constexpr uint8_t wcode() {
    if constexpr (Extent<T>::array) return uint8_t(0x80 | wcode<typename Extent<T>::Elem>());
    else return wcode_<T>();
  }
  struct Hasher {
    uint32_t h;
    constexpr void layer(Name n) { h = fnv(fnvs(h, n), 0xFF); }
    template<class T> constexpr void operator()(Name n, T&) {
      h = fnv(fnv(fnvs(h, n), 0), wcode<T>());
      if constexpr (Extent<T>::array) h = fnv(fnv(h, uint8_t(Extent<T>::n)), uint8_t(Extent<T>::n >> 8));
    }
  };
  struct TypeCheck {
    constexpr void layer(Name) {}
    template<class T> constexpr void operator()(Name, T&) { (void)wcode<T>(); }
  };
  template<class S> constexpr bool fields_typed() { S s{}; TypeCheck v{}; S::each(s, v); return true; }
  template<class R> constexpr uint32_t schema() { R r{}; Hasher h{2166136261u}; r.each(h); return h.h; }
  template<class R> inline constexpr uint32_t schema_v = schema<R>();
// ONEMACHINE_STATE_PIN(Res, 0x...): the composition's schema hash as this target compiles it. Put it in the header that defines the composition, so every
// target's build checks it: a field that is not the same width on every target (a plain `int`) then fails the build of the target that differs.
#define ONEMACHINE_STATE_PIN(R, hash) static_assert(::state::schema_v<R> == (hash), "state: the schema of " #R " is not the pinned one on this target: a field, a name, the order, or the width of a type (plain int, long) changed")

  struct Root : hapi::SlotRoot {
    template<class F> void step(const F&) {}
  };
  struct API { using Res = Root; };

  // the step of a layer, added to its Res: the layers below first, then this layer's own slot
  template<class Tag, class S, class Body>
  struct StepContract {
    template<class Below, class R> struct Type : Below {
      template<class F> void step(const F& prev) {
        Below::step(prev);
        R& self = static_cast<R&>(*this);
        if constexpr (std::is_same<Body, void>::value) self.me() = hapi::slot<Tag>(prev);
        else self.me() = Body::run(static_cast<const Below&>(self), prev);
      }
    };
  };

  template<class Tag, class S, class Body = void>
  struct Layer {
    template<class O> struct Part : hapi::Slot<Tag, S, StepContract<Tag,S,Body>::template Type>::template Part<O> {
      static_assert(HasName<Tag>::value, "state: a layer tag needs a `static constexpr const char* name`");
      static_assert(!HasTag<Tag, typename O::Res>::value, "state: two layers claim the same tag");
      static_assert(fields_typed<S>(), "state: a field type is not wire-representable");
      static_assert(fields_fit<S>(), "state: a layer has more fields than ONEMACHINE_STATE_MAX_NAMES");
      static_assert(fields_unique<S>(), "state: two fields of one layer have the same name");
      static_assert(layer_scan_v<typename O::Res, Tag>.count < ONEMACHINE_STATE_MAX_NAMES, "state: more layers than ONEMACHINE_STATE_MAX_NAMES");
      static_assert(!layer_scan_v<typename O::Res, Tag>.found, "state: two layers have the same name");
    };
  };
}
