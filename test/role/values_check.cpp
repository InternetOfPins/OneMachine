// The description lines a kind may print besides `param`: value, scale, unit (role/face.h, Params). Kinds here are test kinds:
// a select with labels, and an analog input (report only: its Command has no fields) with a scale and a unit. `values_check <file>` writes the
// machine description there for values_check.py.
#include <cstdio>
#include <string>
#include <oneMachine/role/role.h>
#include <oneMachine/role/kinds.h>
#include <oneMachine/role/face.h>

struct Mode { ONEMACHINE_STATE_NAME(name, "mode"); };
struct Vin  { ONEMACHINE_STATE_NAME(name, "vin"); };
struct Raw  { ONEMACHINE_STATE_NAME(name, "raw"); };
struct Led  { ONEMACHINE_STATE_NAME(name, "led"); };
struct E { static bool live() { return true; } template<class P> static void where(P& put) { put('e'); } };
struct E2 { static bool live() { return true; } template<class P> static void where(P& put) { put('f'); } };
struct E3 { static bool live() { return true; } static void set(bool) {} static bool get() { return false; } template<class P> static void where(P& put) { put('g'); } };

struct Select {
  ONEMACHINE_STATE_NAME(name, "select");
  struct Command { uint8_t index; ONEMACHINE_STATE_NAME(n_index, "index");
    template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_index(), s.index); } };
  struct Report  { uint8_t index; ONEMACHINE_STATE_NAME(n_index, "index");
    template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_index(), s.index); } };
  ONEMACHINE_STATE_NAME(l_off, "off"); ONEMACHINE_STATE_NAME(l_high, "high");
  template<class V> static void params(V& v) { v.value(0, l_off()); v.value(2); v.value(7, l_high()); }
  template<class E> static void apply(const Command& c, Report& r) { r.index = c.index; }
  template<class E> static void sense(Report&) {}
  static void safe(Command& c, const Report&) { c.index = 0; }
};
struct Analog {                                           // report only: Command has no fields
  ONEMACHINE_STATE_NAME(name, "analog");
  struct Command { template<class Self, class F> static constexpr void each(Self&, F&) {} };
  struct Report  { uint16_t raw; ONEMACHINE_STATE_NAME(n_raw, "raw");
    template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_raw(), s.raw); } };
  ONEMACHINE_STATE_NAME(n_raw, "raw"); ONEMACHINE_STATE_NAME(u_v, "V");
  template<class V> static void params(V& v) { v.scale(n_raw(), 5, 1023); v.unit(n_raw(), u_v()); }
  template<class E> static void apply(const Command&, Report& r) { r.raw = 1023; }
  template<class E> static void sense(Report& r) { r.raw = 1023; }
  static void safe(Command&, const Report&) {}
};

using M = role::Machine<role::Role<Mode, Select, E>, role::Role<Vin, Analog, E2>, role::Role<Led, role::Switch<true>, E3>>;

int fails = 0;
void check(const char* what, bool ok) { printf("  %-5s %s\n", ok ? "ok" : "FAIL", what); if (!ok) fails++; }

int main(int argc, char** argv) {
  std::string d; auto put = [&](char c) { d += c; };
  role::describe<M>(put);
  if (argc > 1) { FILE* f = fopen(argv[1], "w"); fputs(d.c_str(), f); fclose(f); }
  auto has = [&](const char* s) { return d.find(s) != std::string::npos; };
  check("value with a label", has("value mode 0 off\n"));
  check("value without a label", has("value mode 2\n"));
  check("value, second label", has("value mode 7 high\n"));
  check("scale", has("scale vin raw 5 1023\n"));
  check("unit", has("unit vin raw V\n"));
  check("a kind that prints none of them prints exactly its param line", has("role led switch\nparam led safe 1\nat led g\n"));
  check("the new lines come after their role line and before the next role", d.find("role mode select\nvalue mode 0 off\nvalue mode 2\nvalue mode 7 high\nat mode e\nrole vin analog\nscale vin raw 5 1023\nunit vin raw V\n") != std::string::npos);
  M::Command c{}; M::Report r{}; M::apply(c, r); M::sense(r); M::safe(c, r);
  std::string cd, rd; auto pc = [&](char ch) { cd += ch; }; auto pr = [&](char ch) { rd += ch; };
  state::describe<M::Command>(pc); state::describe<M::Report>(pr);
  check("report-only kind: its report has the field", rd.find("vin/raw u16") != std::string::npos && state::get<Vin>(r).raw == 1023);
  check("report-only kind: an empty layer in the command description", cd.find("layer vin\nlayer mode\n") != std::string::npos);
  printf("%s\n", fails ? "FAILED" : "ok");
  return fails ? 1 : 0;
}
