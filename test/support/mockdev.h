// Mock consumers for F3, on mock time: a display that is busy while it refreshes, an SD card that can be busy, pulled and put back,
// and the far side of a ring that can stall. Fixed arrays only, so the same code builds for AVR.
// Every sink charges the time it takes to Cost, in the phase the test has set: `fan` while the fan-out runs, else tick.
#pragma once
#include <stdint.h>
#include <oneMachine/fail/delivery.h>

namespace mockdev {

  using fail::Rec;
  using fail::Outcome;
  using fail::Kind;

  struct Cost {
    inline static uint32_t inFan = 0, inTick = 0;
    inline static bool     fan = false;
    static void charge(uint32_t ms) { (fan ? inFan : inTick) += ms; }
    static void reset()             { inFan = inTick = 0; fan = false; }
  };

  // takes one record per updateMs; busy (Blocked) in between
  struct Display {
    inline static uint32_t busyUntil = 0, updateMs = 0;
    inline static Rec      shown{};
    inline static uint16_t updates = 0;
    static Outcome deliver(const Rec& r, uint32_t now) {
      if (now < busyUntil) return Outcome::Blocked();
      shown = r; ++updates; busyUntil = now + updateMs;
      Cost::charge(1);
      return Outcome::Ok();
    }
    static void reset() { busyUntil = updateMs = 0; shown = Rec{}; updates = 0; }
  };

  // mounted by rebegin(); a pulled card unmounts; a record is written only to a mounted card that is there
  struct Sd {
#ifdef __AVR__
    static constexpr uint8_t kLog = 16;
#else
    static constexpr uint8_t kLog = 64;
#endif
    inline static bool     present = true, mounted = false;
    inline static uint32_t busyUntil = 0;
    inline static Rec      log[kLog];
    inline static uint8_t  n = 0;
    inline static uint16_t begins = 0, mounts = 0, refusedWrites = 0, poisonWrites = 0;
    inline static int32_t  poison = 0x7FFFFFFF;                          // a record value the sink refuses outright (bad data, not a card fault); default: none

    static bool rebegin() {
      ++begins; Cost::charge(5);
      mounted = present;
      if (mounted) ++mounts;
      return mounted;
    }
    static void pull()   {
      present = false;
#ifndef F3_NEG_LATCHED_READY
      mounted = false;                                                  // the volume goes with the card
#endif
    }
    static void insert() { present = true; }                            // it has to be mounted again: rebegin()
    static bool ready()  { return mounted && present; }                 // what the card is now, not what begin() once said

    static Outcome deliver(const Rec& r, uint32_t now) {
#ifndef F3_NEG_NO_POISON_CHECK
      if (r.v == poison) { ++poisonWrites; Cost::charge(8); return Outcome::Fail(Kind::Refused, 0); }   // the card is fine; this one record is not writable
#endif
#ifdef F3_NEG_LATCHED_READY
      const bool up = mounted;                                          // ready() latched at begin: a pulled card still reads ready
#else
      const bool up = ready();
#endif
      if (!up) { ++refusedWrites; Cost::charge(20); return Outcome::Fail(Kind::Fault, mounted ? 2 : 1); }   // 1: not mounted, 2: gone under a mounted volume
      if (now < busyUntil) return Outcome::Blocked();
      Cost::charge(8);
      if (present && n < kLog) log[n++] = r;                            // (a latched card that is gone loses the record here)
      return Outcome::Ok();
    }
    static void reset() { present = true; mounted = false; busyUntil = 0; n = 0; begins = mounts = refusedWrites = poisonWrites = 0; poison = 0x7FFFFFFF; }
  };

  // the far side of a ring: takes records until it stalls, then is busy
  struct Ring {
#ifdef __AVR__
    static constexpr uint8_t kGot = 16;
#else
    static constexpr uint8_t kGot = 64;
#endif
    inline static bool     stalled = false;
    inline static Rec      got[kGot];
    inline static uint8_t  n = 0;
    static Outcome deliver(const Rec& r, uint32_t) {
      if (stalled) return Outcome::Blocked();
      if (n < kGot) got[n++] = r;
      Cost::charge(1);
      return Outcome::Ok();
    }
    static void reset() { stalled = false; n = 0; }
  };

}
