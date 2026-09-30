#pragma once
// oneMachine/role/sim.h -- simulated endpoints, to run a machine's roles on a host (tests, a Python-driven simulation) with nothing attached.
//   SimPin<Id>             a digital output: Switch
//   SimPwm<Id, Top>        a PWM output with its own top: Light
//   SimStepDir<Id, Rate>   a step/dir driver that moves Rate steps toward its target per tick(): Axis
//   SimI2c<Mux, Dev>       a bus with one bridge: devices at (bridge channel or `direct`, address); `select`, and a PCA9685-shaped driver
//                          `Pca` for Found and Pinned. Every write counts on its device (`writes`); a write that reaches no device counts in
//                          `misrouted`.
#include <oneMachine/state/face.h>

namespace role {
  template<class P> void put_sim(P& put, const char* what, unsigned id) {
    put('s'); put('i'); put('m'); put('.'); while (*what) put(*what++); put('('); state::put_dec(put, id); put(')'); }

  template<uint8_t Id> struct SimPin {
    static inline bool v = false;
    static bool live() { return true; }
    static void set(bool x) { v = x; } static bool get() { return v; }
    template<class P> static void where(P& put) { put_sim(put, "pin", Id); }
  };
  template<uint8_t Id, uint16_t Top> struct SimPwm {
    static inline uint16_t v = 0; static constexpr uint16_t top = Top;
    static bool live() { return true; }
    static void set(uint16_t x) { v = x > Top ? Top : x; } static uint16_t get() { return v; }
    template<class P> static void where(P& put) { put_sim(put, "pwm", Id); }
  };
  template<uint8_t Id, uint16_t Rate> struct SimStepDir {
    static inline int32_t pos = 0, target = 0;
    static bool live() { return true; }
    static void to(int32_t t) { target = t; } static int32_t at() { return pos; }
    static void tick() { int32_t d = target - pos; pos += d > int32_t(Rate) ? int32_t(Rate) : d < -int32_t(Rate) ? -int32_t(Rate) : d; }
    template<class P> static void where(P& put) { put_sim(put, "stepdir", Id); }
  };

  // Mux: bridge channels (0..Mux-1), Dev: device slots
  template<uint8_t Mux, uint8_t Dev> struct SimI2c {
    static constexpr uint8_t direct = 0xFF;                  // a device on the root bus, not behind the bridge
    struct Device { uint8_t mux = 0, addr = 0; bool present = false; uint16_t ch[16] = {}; unsigned writes = 0; };
    static inline Device dev[Dev]{};
    static inline uint8_t selected = direct;
    static inline unsigned misrouted = 0;
    static void select(uint8_t c) { selected = c; }
    static Device* at(uint8_t addr) {                        // what an address reaches now: root devices, and those behind the selected channel
      for (auto& d : dev) if (d.present && d.addr == addr && (d.mux == direct || d.mux == selected)) return &d;
      return nullptr;
    }
    struct Pca {
      static constexpr uint16_t top = 4095;
      ONEMACHINE_STATE_NAME(name, "pca9685");
      static void set(uint8_t addr, uint8_t unit, uint16_t v) { Device* d = at(addr); if (d) { d->ch[unit & 15] = v > top ? top : v; ++d->writes; } else ++misrouted; }
      static uint16_t get(uint8_t addr, uint8_t unit) { Device* d = at(addr); if (d) return d->ch[unit & 15]; ++misrouted; return 0; }
    };
  };
}
