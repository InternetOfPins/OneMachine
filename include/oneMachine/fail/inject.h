// What a real core would report and the mock TwiAPI cannot (it has no ACK or error path): the device operation's
// Outcome. NoFaults is the real-bus edge as it is today (always Ok, folds away); Injected is scripted scaffolding.
#pragma once
#include <stdint.h>
#include "outcome.h"

namespace fail {

  struct NoFaults {
    static constexpr Outcome after(RowId, uint8_t*) { return Outcome::Ok(); }
  };

  // One scripted step per operation attempt on the target row; rows other than the target always pass.
  struct Step {
    enum : uint8_t { Pass, Nack, Hang, Slow, Error, Refuse, Overflow, BadData };
    uint8_t code, arg;
    static constexpr Step pass()            { return {Pass, 0}; }
    static constexpr Step nack()            { return {Nack, 0x20}; }       // SLA+W not acknowledged
    static constexpr Step hang()            { return {Hang, 0}; }          // never ready: polls exhausted
    static constexpr Step slow(uint8_t n)   { return {Slow, n}; }          // ready after n polls
    static constexpr Step error()           { return {Error, 0x08}; }      // hardware error flag
    static constexpr Step refuse(uint8_t r) { return {Refuse, r}; }        // the device answered "no"
    static constexpr Step overflow()        { return {Overflow, 1}; }      // local buffer full
    static constexpr Step badData()         { return {BadData, 0}; }       // a byte of the reading is wrong
  };

  struct Injected {
    static constexpr uint8_t maxPolls = 8;          // bounded ready-poll, like a core's spin count
    static constexpr uint8_t cap      = 16;
    inline static Step     script[cap];
    inline static uint8_t  len = 0, pos = 0;
    inline static RowId    target = 0xFF;
    inline static uint16_t attempts = 0;            // operation attempts on the target row

    static void load(RowId row, const Step* s, uint8_t n) {
      target = row; len = n > cap ? cap : n; pos = 0;
      for (uint8_t i = 0; i < len; ++i) script[i] = s[i];
    }
    static void clear() { target = 0xFF; len = pos = 0; attempts = 0; }

    // b: the two bytes of a temperature reading; the edge's own integrity check is a plausibility range
    static Outcome after(RowId row, uint8_t* b) {
      Step s = Step::pass();
      if (row == target) { ++attempts; if (pos < len) s = script[pos++]; }
      if      (s.code == Step::Nack)     return Outcome::Fail(Kind::Absent,   s.arg);
      else if (s.code == Step::Hang)     return Outcome::Fail(Kind::Timeout,  maxPolls);
      else if (s.code == Step::Slow)   { if (s.arg > maxPolls) return Outcome::Fail(Kind::Timeout, maxPolls); }
      else if (s.code == Step::Error)    return Outcome::Fail(Kind::Fault,    s.arg);
      else if (s.code == Step::Refuse)   return Outcome::Fail(Kind::Refused,  s.arg);
      else if (s.code == Step::Overflow) return Outcome::Fail(Kind::Overflow, s.arg);
      else if (s.code == Step::BadData)  b[0] |= 0x40;
      const int16_t v = int16_t(uint16_t((uint16_t(b[0]) << 8) | b[1]));
      if (v < -400 || v > 1250) return Outcome::Fail(Kind::Corrupt, b[0]);
      return Outcome::Ok();
    }
  };

}
