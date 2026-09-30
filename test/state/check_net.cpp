// Native checks of the typed net (host, ASan+UBSan). Every line is `CHECK <name>: ok|FAIL`; the exit status is the number of failures.
// The names are what build.sh's mutations expect to see fail.
#include "net.h"
#include "flat.h"
#include "golden.h"
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

static int failures = 0;
static void check(const char* name, bool ok, const char* why = "") {
  printf("CHECK %s: %s%s%s\n", name, ok ? "ok" : "FAIL", ok ? "" : " -- ", ok ? "" : why); if (!ok) failures++;
}

// the state as a list of numbers, and as `layer/field:type` paths (a stand-in for a face, host only)
struct Values { std::vector<uint32_t> v; void layer(state::Name) {} template<class T> void operator()(state::Name, T& x) { v.push_back(uint32_t(x)); } };
static std::string str(state::Name n) { std::string r; for (unsigned i = 0; n.at(i); i++) r += n.at(i); return r; }
struct Paths {
  std::string s, cur;
  void layer(state::Name n) { cur = str(n); }
  template<class T> void operator()(state::Name n, T&) { s += cur + "/" + str(n) + ":" + (T(-1) < T(0) ? "i" : "u") + std::to_string(sizeof(T) * 8) + " "; }
};
struct Extent { std::vector<std::pair<const char*, const char*>> r;   // address range of every field
  void layer(state::Name) {} template<class T> void operator()(state::Name, T& x) { r.push_back({(const char*)&x, (const char*)&x + sizeof(T)}); } };
template<class R> static std::vector<uint32_t> values(const R& r) { Values v; r.each(v); return v.v; }

struct HoldTag { ONEMACHINE_STATE_NAME(name, "hold"); };
struct SlotHold { int16_t x; ONEMACHINE_STATE_NAME(n_x, "x"); template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_x(), s.x); } };
static Net make(int16_t a, int16_t b) { Net n{}; state::get<FibA>(n).a = a; state::get<FibB>(n).b = b; return n; }
static void advance(Net& p, Net& n) { n.step(p); p = n; }                                  // ADVANCE
static Flat mkflat(int16_t a, int16_t b) { Flat f{}; wr16(f.v + F_A, a); wr16(f.v + F_B, b); return f; }

int main() {
  const int16_t starts[][2] = {{0, 1}, {1, 1}, {32767, 1}, {-32768, -1}, {12345, -321}};
  bool okVals = true, okHand = true, okWatch = true, okPure = true, okTwice = true;
  for (auto& st : starts) {
    Net p = make(st[0], st[1]), n{}; Flat fp = mkflat(st[0], st[1]), fn{};
    int a = st[0], b = st[1], steps = 0;
    for (int i = 0; i < 300; i++) {
      // reference: the recurrence, on wide integers, wrapped to 16 bits
      int na = b, nb = int16_t((a + b) & 0xFFFF); a = na; b = nb; steps++;
      Net poisoned; memset(&poisoned, 0xA5, sizeof poisoned);                       // next holds garbage: the step must not read its own next
      poisoned.step(p);
      Net twice{}; twice.step(p);
      okPure  &= values(poisoned) == values(twice);
      okTwice &= values(twice) == values(poisoned);
      advance(p, n);
      flat_step(fp, fn); fp = fn;
      okVals  &= state::get<FibA>(p).a == int16_t(a) && state::get<FibB>(p).b == int16_t(b);
      okWatch &= state::get<Watch>(p).steps == steps && state::get<Watch>(p).odd == ((int16_t(a) + int16_t(b)) & 1);
      okHand  &= rd16(fp.v + F_A) == state::get<FibA>(p).a && rd16(fp.v + F_B) == state::get<FibB>(p).b
              && rdu16(fp.v + F_STEPS) == state::get<Watch>(p).steps && fp.v[F_ODD] == state::get<Watch>(p).odd;
    }
  }
  check("values", okVals, "the state differs from the recurrence");
  check("watch", okWatch, "the observer's steps/odd differ from the model");
  check("hand", okHand, "the typed net differs from the hand-indexed one");
  check("pure-next", okPure, "the result depends on what `next` held before the step");
  check("pure-repeat", okTwice, "two steps from the same prev differ");
  { Net p = make(3, 5), n{}; auto before = values(p); n.step(p); check("prev-unchanged", values(p) == before, "step wrote prev"); }

  { using Hold = hapi::APIOf<state::API, state::Layer<HoldTag,SlotHold>>::Res;              // no Body: the layer holds its previous value
    Hold p{}, n{}; state::get<HoldTag>(p).x = 77; memset(&n, 0xA5, sizeof n); n.step(p);
    check("hold", state::get<HoldTag>(n).x == 77, "a layer without a body must carry its previous value"); }

  { Net r{}; Extent e; r.each(e);                                                        // the slots are disjoint and inside the object
    bool ok = true; const char* lo = (const char*)&r; const char* hi = lo + sizeof r;
    for (size_t i = 0; i < e.r.size(); i++) { ok &= e.r[i].first >= lo && e.r[i].second <= hi;
      for (size_t j = i + 1; j < e.r.size(); j++) ok &= e.r[i].second <= e.r[j].first || e.r[j].second <= e.r[i].first; }
    check("disjoint", ok, "two fields share bytes, or one is outside the object"); }

  { Paths p; Net r{}; r.each(p);
    check("golden-paths", p.s == GOLDEN_PATHS, (std::string("got ") + p.s).c_str());
    char buf[64]; snprintf(buf, sizeof buf, "got 0x%08x", (unsigned)state::schema_v<Net>);
    check("golden-hash", state::schema_v<Net> == GOLDEN_HASH, buf); }
  printf("%d failed\n", failures);
  return failures;
}
