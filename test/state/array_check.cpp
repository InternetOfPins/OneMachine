// Array fields of state.h (host, ASan+UBSan). No argument: CHECK lines. With an argument: the peer output for the python oracle:
//   describe | values [n] | frame [n]
#include "arraynet.h"
#include <oneMachine/state/wire.h>
#include <oneMachine/state/face.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
static int failures = 0;
static void check(const char* name, bool ok, const char* why = "") { printf("CHECK %s: %s%s%s\n", name, ok ? "ok" : "FAIL", ok ? "" : " -- ", ok ? "" : why); if (!ok) failures++; }
struct Out { void operator()(char c) { putchar(c); } };
static std::string str(state::Name n) { std::string r; for (unsigned i = 0; n.at(i); i++) r += n.at(i); return r; }
struct Values { std::string cur; void layer(state::Name n) { cur = str(n); }
  template<class T> void operator()(state::Name n, const T& v) {
    printf("%s/%s=", cur.c_str(), str(n).c_str());
    print(v); putchar('\n'); }
  template<class E> void one(const E& e) { if constexpr (std::is_same<E, bool>::value) printf("%d", int(e)); else if constexpr (state::wcode<E>() & 1) printf("%lld", (long long)e); else printf("%llu", (unsigned long long)e); }
  template<class T> void print(const T& v) { if constexpr (state::Extent<T>::array) { for (unsigned i = 0; i < state::Extent<T>::n; i++) { if (i) putchar(','); one(v[i]); } } else one(v); } };
struct QTag { ONEMACHINE_STATE_NAME(name, "arr"); };
struct S4 { uint8_t v[4]; ONEMACHINE_STATE_NAME(n_v, "v"); template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_v(), s.v); } };
struct S5 { uint8_t v[5]; ONEMACHINE_STATE_NAME(n_v, "v"); template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_v(), s.v); } };
struct S1 { uint8_t v;    ONEMACHINE_STATE_NAME(n_v, "v"); template<class Self, class F> static constexpr void each(Self& s, F& f) { f(n_v(), s.v); } };
static ArrNet stepped(int n) { ArrNet s = arrInitial(), t{}; for (int i = 0; i < n; i++) { t.step(s); s = t; } return s; }
static std::vector<uint8_t> frame(const ArrNet& s) { uint8_t b[state::wire_size<ArrNet>()]; uint8_t* e = state::write(s, b); return {b, e}; }
static void hex(const std::vector<uint8_t>& v) { for (uint8_t b : v) printf("%02x", b); putchar('\n'); }
struct Locate { const char* want; unsigned off = 0; bool found = false; void layer(state::Name) {}
  template<class T> void operator()(state::Name n, const T&) { if (!found && str(n) == want) found = true; else if (!found) off += sizeof(T); } };

int main(int argc, char** argv) {
  std::string c = argc > 1 ? argv[1] : ""; int n = argc > 2 ? atoi(argv[2]) : 0; Out o;
  if (c == "describe") { state::describe<ArrNet>(o); return 0; }
  if (c == "values") { Values v; stepped(n).each(v); return 0; }
  if (c == "frame") { hex(frame(stepped(n))); return 0; }
  const unsigned N = state::wire_size<ArrNet>();
  check("size", N == 4 + 4 + 6 + 2 + 16 + 2 + 2, "an array's frame bytes are its elements' widths times the count");
  check("sizeof", sizeof(ArrNet) >= N - 4, "the state is smaller than its frame");
  auto good = frame(arrInitial());
  { ArrNet got{}; state::Status st = state::read(got, good.data(), N); check("roundtrip", st == state::Status::Ok && frame(got) == good, "an array did not survive a frame"); }
  { ArrNet s = stepped(40); auto f = frame(s); ArrNet got{}; check("roundtrip-stepped", state::read(got, f.data(), N) == state::Status::Ok && frame(got) == f, "a stepped array state does not survive a frame"); }
  { Locate l{"f"}; arrInitial().each(l); auto f = good; f[4 + l.off + 1] = 2; ArrNet got{};                 // the second element of the bool array
    check("refuse-bool-element", state::read(got, f.data(), N) == state::Status::BadValue, "a bool element of 2 was accepted");
    check("atomic", frame(got) == frame(ArrNet{}), "a refused frame changed the state"); }
  { using R4 = hapi::APIOf<state::API, state::Layer<QTag,S4>>::Res; using R5 = hapi::APIOf<state::API, state::Layer<QTag,S5>>::Res; using R1 = hapi::APIOf<state::API, state::Layer<QTag,S1>>::Res;
    check("hash-count", state::schema_v<R4> != state::schema_v<R5>, "u8[4] and u8[5] hash the same");          // the count is part of the schema
    check("hash-array-vs-scalar", state::schema_v<R4> != state::schema_v<R1>, "u8[4] and u8 hash the same"); }
  { std::string json; struct J { std::string& s; void operator()(char ch) { s += ch; } } j{json}; state::json(arrInitial(), j);
    check("json", json.find("\"band\":[1,200,3,255]") != std::string::npos && json.find("\"w\":[-1,4660,-32768]") != std::string::npos && json.find("\"f\":[true,false]") != std::string::npos, json.c_str()); }
  printf("%d failed\n", failures); return failures;
}
