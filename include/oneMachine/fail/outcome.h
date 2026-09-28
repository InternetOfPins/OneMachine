// The inner face of a failure edge: what an operation below a failure component returns.
#pragma once
#include <stdint.h>

namespace fail {

  using RowId = uint8_t;
  using Id    = uint8_t;     // correlates an operation in flight (the detail of its Pending); single-slot operations use 0

  // Unknown: a failure whose cause the core cannot tell (OneBus TwiCause::Unknown). Not Absent, not Refused: it says nothing
  // about the device, and it never triggers Recover.
  enum class Kind : uint8_t { Absent = 1, Timeout, Refused, Overflow, Corrupt, Fault, Unknown };

  constexpr uint8_t bit(Kind k) { return uint8_t(1u << (uint8_t(k) - 1)); }

  template<Kind... K> struct KindSet { static constexpr uint8_t mask = uint8_t((0u | ... | bit(K))); };

  // Ok / Pending / Blocked / Fail(kind), plus a raw detail byte only the edge author reads. A Pending that starts a correlated
  // operation carries its Id in `detail`. Idle (code 2, detail 0xFF) is "nothing to report": never issued, or already read.
  // Blocked: the operation did not complete because the bus below it failed, which the bus edge owns; it is not this
  // device's failure and no layer counts it against the device.
  struct Outcome {
  private:
    uint8_t code_ = 0;    // 0 Ok, 1 Pending, 2 Blocked (detail 0xFF: Idle), 3.. Fail(Kind(code - 2)); read through the predicates
    constexpr Outcome(uint8_t c, uint8_t d) : code_(c), detail(d) {}
  public:
    uint8_t detail = 0;
    constexpr Outcome() = default;
    static constexpr Outcome Ok(uint8_t d = 0)             { return Outcome(0, d); }             // d: what the operation returned, when it returns a byte
    static constexpr Outcome Pending(uint8_t id = 0)       { return Outcome(1, id); }
    static constexpr Outcome Idle()                        { return Outcome(2, 0xFF); }
    static constexpr Outcome Blocked()                     { return Outcome(2, 0); }
    static constexpr Outcome Fail(Kind k, uint8_t d = 0)   { return Outcome(uint8_t(uint8_t(k) + 2), d); }
    constexpr bool isOk()      const { return code_ == 0; }
    constexpr bool isPending() const { return code_ == 1; }
    constexpr bool isIdle()    const { return code_ == 2 && detail == 0xFF; }
    constexpr bool isBlocked() const { return code_ == 2 && detail != 0xFF; }
    constexpr bool isDone()    const { return code_ != 1 && !isIdle(); }      // Ok, Blocked or a failure: something to report
    constexpr bool failed()    const { return code_ >= 3; }
    constexpr Kind kind()      const { return Kind(code_ - 2); }
    // the code itself, for checksums and logs; comparisons go through the predicates
    constexpr uint8_t raw()    const { return code_; }
  };

  // Fresh: a new operation from the data path. Reissue: the stored operation, again, from tick.
  enum class Cause : uint8_t { Fresh, Reissue };

  // Same numbering as discover::Status, so the edge can write it into the registry row unchanged.
  enum class RowState : uint8_t { Alive, Stale, Gone };

  // The outer face's status query: plain data, cannot fail.
  struct FailStatus { uint8_t retries, recovers, drops, fails, lastKind, lastDetail; };

}
