#pragma once
// oneMachine/role/call.h -- role::Link as a function call: the same request and response bytes, no stream. For a consumer in the same
// image or process: Python loading a host build with ctypes, Rust firmware calling into the C++ over its C ABI (HAPI's rust_stm32_bridge
// pattern). The consumer implements the protocol, as it does over a serial port; nothing here is specific to a language.
//
// role::Call<M, App> holds a machine's command, report and link:
//   call(op, in, n, out, cap, now)   one request (op, payload) -> the response (status, u16 length, payload) in out; returns its length,
//                                    or minus the length it needs when cap is too small. Only the text and frame replies are long, and
//                                    they change nothing: ask again with more room. Every reply that changes something fits in 3 bytes.
//   cycle(now)                       the cycle boundary: a command accepted since the last cycle is applied, a quiet supervisor gets the
//                                    safe command, the report is refreshed. The owner of the loop calls it (firmware main loop, a host
//                                    thread, or the consumer itself between calls).
// ONEMACHINE_CALL_EXPORT(prefix, obj) gives it a C ABI:
//   int32_t <prefix>_call(uint8_t op, const uint8_t* in, uint16_t n, uint8_t* out, uint16_t cap, uint32_t now_ms)
//   void    <prefix>_cycle(uint32_t now_ms)
// One call at a time: the reply buffer is shared by every Call of one type (there is one machine per image).
// ONEMACHINE_CALL_EXPORT(onemachine, dev) is the conventional name (python/onemachine's CtypesLink looks for it by default).
#include "link.h"

namespace role {
  template<class Tag> struct BufOut {
    static inline uint8_t* p = nullptr;
    static inline uint16_t cap = 0, n = 0;
    static void put(uint8_t b) { if (n < cap) p[n] = b; ++n; }
  };

  template<class M, class App = NoApp, unsigned Cap = link_cap<M>()>
  struct Call {
    using Out = BufOut<Call>;
    typename M::Command cmd{};
    typename M::Report rep{};
    Link<M, Out, App, Cap> link;
    explicit Call(uint32_t quietMs) : link(rep, quietMs) { M::apply(cmd, rep); }

    int32_t call(uint8_t op, const uint8_t* in, uint16_t n, uint8_t* out, uint16_t cap, uint32_t now) {
      Out::p = out; Out::cap = cap; Out::n = 0;
      link.feed(op, now); link.feed(uint8_t(n), now); link.feed(uint8_t(n >> 8), now);
      for (uint16_t i = 0; i < n; i++) link.feed(in[i], now);
      return Out::n <= cap ? int32_t(Out::n) : -int32_t(Out::n);
    }
    void cycle(uint32_t now) {
      if (link.take(cmd)) M::apply(cmd, rep);
      if (link.quiet(now)) { M::safe(cmd, rep); M::apply(cmd, rep); }
      M::sense(rep);
    }
  };
}

#if defined(_WIN32)
  #define ONEMACHINE_CALL_API __declspec(dllexport)
#elif defined(__GNUC__)
  #define ONEMACHINE_CALL_API __attribute__((visibility("default")))
#else
  #define ONEMACHINE_CALL_API
#endif
#define ONEMACHINE_CALL_EXPORT(prefix, obj) \
  extern "C" ONEMACHINE_CALL_API int32_t prefix##_call(uint8_t op, const uint8_t* in, uint16_t n, uint8_t* out, uint16_t cap, uint32_t now_ms) { \
    return (obj).call(op, in, n, out, cap, now_ms); } \
  extern "C" ONEMACHINE_CALL_API void prefix##_cycle(uint32_t now_ms) { (obj).cycle(now_ms); }
