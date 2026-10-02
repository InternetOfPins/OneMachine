#pragma once
// oneMachine/role/kinds.h -- the kinds of output a role can be. A kind gives:
//   name()                        its word in the machine description ("light")
//   Command, Report               state slots (fixed-width fields, one `each`); the report's `live` is added by role::Machine
//   params(v)                     v(name, int32 value) per parameter: its units and limits, in the machine description
//   apply<E>(cmd, rep)            write the command to endpoint E, read back what E actually does
//   sense<E>(rep)                 read back only
//   safe(cmd, rep)                the command that makes the output harmless (a per-role choice where there is one, as a parameter)
//   Tune, valid(tune)             the parameters as a state slot, for role::Tuned<Kind> (role/tune.h): each defaults to the template argument,
//                                 and valid() keeps a run-time value inside what the template allows. apply/sense/safe have an overload
//                                 taking the Tune; the plain ones above use the template arguments, so a kind that is not tuned pays nothing.
// An endpoint serves the kinds whose calls it has: Switch set/get, Light set/get + `top`, Axis to/at.
#include "role.h"

namespace role {
  // on/off; Safe is the state it goes to when the supervisor is quiet
  template<bool Safe = false>
  struct Switch {
    ONEMACHINE_STATE_NAME(name, "switch");
    struct Command { bool on; ONEMACHINE_STATE_NAME(n_on, "on");
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_on(), s.on); } };
    struct Report  { bool on; ONEMACHINE_STATE_NAME(n_on, "on");
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_on(), s.on); } };
    ONEMACHINE_STATE_NAME(p_safe, "safe");
    template<class V> static void params(V& v) { v(p_safe(), int32_t(Safe)); }
    struct Tune { bool safe = Safe;
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(p_safe(), s.safe); } };
    static constexpr bool valid(const Tune&) { return true; }
    template<class E> static void apply(const Command& c, Report& r) { E::set(c.on); r.on = E::get(); }
    template<class E> static void apply(const Command& c, Report& r, const Tune&) { apply<E>(c, r); }
    template<class E> static void sense(Report& r) { r.on = E::get(); }
    template<class E> static void sense(Report& r, const Tune&) { sense<E>(r); }
    static void safe(Command& c, const Report&) { c.on = Safe; }
    static void safe(Command& c, const Report&, const Tune& t) { c.on = t.safe; }
  };

  // a level 0..Max (Max at most the endpoint's own top); a command above is clamped, and the report says so
  template<uint16_t Max, uint16_t Safe = 0>
  struct Light {
    static_assert(Safe <= Max, "role: a light's safe level is above its max");
    ONEMACHINE_STATE_NAME(name, "light");
    struct Command { uint16_t level; ONEMACHINE_STATE_NAME(n_level, "level");
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_level(), s.level); } };
    struct Report  { uint16_t level; bool clamped; ONEMACHINE_STATE_NAME(n_level, "level"); ONEMACHINE_STATE_NAME(n_clamped, "clamped");
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_level(), s.level); f(n_clamped(), s.clamped); } };
    ONEMACHINE_STATE_NAME(p_max, "max"); ONEMACHINE_STATE_NAME(p_safe, "safe");
    template<class V> static void params(V& v) { v(p_max(), int32_t(Max)); v(p_safe(), int32_t(Safe)); }
    struct Tune { uint16_t max = Max; uint16_t safe = Safe;
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(p_max(), s.max); f(p_safe(), s.safe); } };
    static constexpr bool valid(const Tune& t) { return t.max <= Max && t.safe <= t.max; }
    template<class E> static void apply(const Command& c, Report& r) { apply<E>(c, r, Tune{}); }
    template<class E> static void apply(const Command& c, Report& r, const Tune& t) {
      static_assert(Max <= E::top, "role: a light's max is above its endpoint's top");
      uint16_t l = c.level > t.max ? t.max : c.level;
      E::set(l); r.level = E::get(); r.clamped = l != c.level;
    }
    template<class E> static void sense(Report& r) { r.level = E::get(); }
    template<class E> static void sense(Report& r, const Tune&) { sense<E>(r); }
    static void safe(Command& c, const Report&) { c.level = Safe; }
    static void safe(Command& c, const Report&, const Tune& t) { c.level = t.safe; }
  };

  // a linear axis in micrometres, StepsPerMm on the motor side; a target outside MinUm..MaxUm is clamped. Safe: hold where it is.
  template<uint16_t StepsPerMm, int32_t MinUm, int32_t MaxUm>
  struct Axis {
    static_assert(StepsPerMm > 0 && MinUm < MaxUm, "role: an axis needs steps/mm > 0 and min < max");
    ONEMACHINE_STATE_NAME(name, "axis");
    struct Command { int32_t target_um; ONEMACHINE_STATE_NAME(n_target, "target_um");
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_target(), s.target_um); } };
    struct Report  { int32_t pos_um; bool clamped; ONEMACHINE_STATE_NAME(n_pos, "pos_um"); ONEMACHINE_STATE_NAME(n_clamped, "clamped");
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_pos(), s.pos_um); f(n_clamped(), s.clamped); } };
    ONEMACHINE_STATE_NAME(p_steps, "steps_mm"); ONEMACHINE_STATE_NAME(p_min, "min_um"); ONEMACHINE_STATE_NAME(p_max, "max_um");
    template<class V> static void params(V& v) { v(p_steps(), int32_t(StepsPerMm)); v(p_min(), MinUm); v(p_max(), MaxUm); }
    struct Tune { uint16_t steps_mm = StepsPerMm; int32_t min_um = MinUm, max_um = MaxUm;
      template<class Self, class F> static constexpr void each(Self& s, F& f) { f(p_steps(), s.steps_mm); f(p_min(), s.min_um); f(p_max(), s.max_um); } };
    // steps/mm is a calibration, any value above 0; the travel stays inside the firmware's MinUm..MaxUm
    static constexpr bool valid(const Tune& t) { return t.steps_mm > 0 && MinUm <= t.min_um && t.min_um < t.max_um && t.max_um <= MaxUm; }
    static int32_t steps(int32_t um, uint16_t spm = StepsPerMm) { return int32_t(int64_t(um) * spm / 1000); }
    static int32_t um(int32_t st, uint16_t spm = StepsPerMm) { return int32_t(int64_t(st) * 1000 / spm); }
    template<class E> static void apply(const Command& c, Report& r) { apply<E>(c, r, Tune{}); }
    template<class E> static void apply(const Command& c, Report& r, const Tune& u) {
      int32_t t = c.target_um < u.min_um ? u.min_um : c.target_um > u.max_um ? u.max_um : c.target_um;
      E::to(steps(t, u.steps_mm)); r.pos_um = um(E::at(), u.steps_mm); r.clamped = t != c.target_um;
    }
    template<class E> static void sense(Report& r) { r.pos_um = um(E::at()); }
    template<class E> static void sense(Report& r, const Tune& u) { r.pos_um = um(E::at(), u.steps_mm); }
    static void safe(Command& c, const Report& r) { c.target_um = r.pos_um; }
    static void safe(Command& c, const Report& r, const Tune&) { safe(c, r); }
  };
}
