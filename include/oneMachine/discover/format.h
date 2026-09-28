// Generic client: sample -> Format -> Transport, each an independent type.
//   Format:    static constexpr const char* prefix;  template<Cap> static uint8_t encode(char* out, RowId, Cap::Value)
//   Transport: static bool send(const char* topic, const uint8_t* payload, uint8_t len)
// GenericClient<Fmt,Tx,...> is a fan-out consumer; N formats and M transports are N+M types, not N*M.
// Numbers are printed without division (AVR: no divide helper), magnitude below 65536, `decimals` fractional digits.
#pragma once
#include <stdint.h>
#include <string.h>
#include <hapi/hapi.h>
#include "capability.h"

namespace discover { namespace fmt {

  inline uint8_t putStr(char* o, const char* s) {
    uint8_t n = 0;
    while (s[n]) { o[n] = s[n]; ++n; }
    return n;
  }

  // v with `decimals` fractional digits: subtract powers of ten, leading zeros dropped but at least decimals+1 digits
  inline uint8_t putFixed(char* o, int32_t v, uint8_t decimals) {
    static const uint16_t P[5] = {10000, 1000, 100, 10, 1};
    uint8_t n = 0;
    uint16_t u;
    if (v < 0) { o[n++] = '-'; u = uint16_t(-v); } else u = uint16_t(v);
    bool started = false;
    for (uint8_t i = 0; i < 5; ++i) {
      const uint8_t left = uint8_t(5 - i);   // digits still to come, this one included
      uint8_t d = 0;
      while (u >= P[i]) { u = uint16_t(u - P[i]); ++d; }
      if (d || started || left <= decimals + 1) {
        started = true;
        o[n++] = char('0' + d);
        if (decimals && left == decimals + 1) o[n++] = '.';
      }
    }
    return n;
  }

  // "T5=18.7": a short label for a text display
  inline uint8_t putLabel(char* o, char tag, RowId row, int32_t v, uint8_t decimals) {
    uint8_t n = 0;
    o[n++] = tag;
    n = uint8_t(n + putFixed(o + n, row, 0));
    o[n++] = '=';
    n = uint8_t(n + putFixed(o + n, v, decimals));
    o[n] = 0;
    return n;
  }

  struct Json {
    static constexpr const char* prefix = "iop/json/";
    template<typename Cap>
    static uint8_t encode(char* o, RowId row, typename Cap::Value v) {
      uint8_t n = putStr(o, "{\"row\":");
      n = uint8_t(n + putFixed(o + n, row, 0));
      n = uint8_t(n + putStr(o + n, ",\"value\":"));
      n = uint8_t(n + putFixed(o + n, int32_t(v), Cap::decimals));
      o[n++] = '}';
      return n;
    }
  };

  struct Csv {
    static constexpr const char* prefix = "iop/csv/";
    template<typename Cap>
    static uint8_t encode(char* o, RowId row, typename Cap::Value v) {
      uint8_t n = putFixed(o, row, 0);
      o[n++] = ',';
      n = uint8_t(n + putFixed(o + n, int32_t(v), Cap::decimals));
      return n;
    }
  };

}

  // topic = format prefix + capability name
  template<typename Fmt, typename Cap>
  inline void topicOf(char* out) {
    uint8_t n = fmt::putStr(out, Fmt::prefix);
    n = uint8_t(n + fmt::putStr(out + n, Cap::name));
    out[n] = 0;
  }

  template<typename Fmt, typename Tx, bool On, typename... Caps>
  struct GenericClient {
    using Accepts = hapi::Chain<Caps...>;
    template<typename Cap> struct Body {
      template<typename T> struct Part : T {
        using T::T;
        void on(const Sample<Cap>& s) {
          if constexpr (On) {
            char topic[32], pay[32];
            topicOf<Fmt, Cap>(topic);
            const uint8_t n = Fmt::template encode<Cap>(pay, s.row, s.value);
            (void)Tx::send(topic, reinterpret_cast<const uint8_t*>(pay), n);
          } else (void)s;
        }
      };
    };
  };

  // in-memory transport: {topicLen, payloadLen, topic, payload}* in one static buffer
  struct MemSink {
    inline static uint8_t  buf[320] = {};
    inline static uint16_t n = 0;
    inline static uint8_t  msgs = 0, dropped = 0;
    static bool send(const char* topic, const uint8_t* p, uint8_t len) {
      const uint8_t tl = uint8_t(strlen(topic));
      if (unsigned(n) + 2u + tl + len > sizeof buf) { ++dropped; return false; }
      buf[n++] = tl; buf[n++] = len;
      for (uint8_t i = 0; i < tl; ++i) buf[n++] = uint8_t(topic[i]);
      for (uint8_t i = 0; i < len; ++i) buf[n++] = p[i];
      ++msgs;
      return true;
    }
    static void reset() { n = msgs = dropped = 0; }
  };

}
