// Native checks of wire.h on the wire net (net2.h) (host, ASan+UBSan). `CHECK <name>: ok|FAIL`; the exit status is the number of failures.
#include "net2.h"
#include <oneMachine/state/wire.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

static int failures = 0;
static void check(const char* name, bool ok, const char* why = "") {
  printf("CHECK %s: %s%s%s\n", name, ok ? "ok" : "FAIL", ok ? "" : " -- ", ok ? "" : why); if (!ok) failures++;
}
static std::string str(state::Name n) { std::string r; for (unsigned i = 0; n.at(i); i++) r += n.at(i); return r; }
struct Locate { const char* want; unsigned off = 0; bool found = false;
  void layer(state::Name) {}
  template<class T> void operator()(state::Name n, const T&) { if (!found && str(n) == want) found = true; else if (!found) off += sizeof(T); } };
using Frame = std::vector<uint8_t>;
static Frame frameOf(const Net2& s) { Frame f(state::wire_size<Net2>()); uint8_t* e = state::write(s, f.data()); f.resize(size_t(e - f.data())); return f; }
static unsigned rnd() { static uint32_t x = 2463534242u; x ^= x << 13; x ^= x >> 17; x ^= x << 5; return x; }

int main() {
  const unsigned N = state::wire_size<Net2>();
  Net2 src = initial(); Frame good = frameOf(src);
  check("size", good.size() == N && N == 4 + 2 + 2 + 2 + 1 + 1 + 1 + 4 + 4 + 8 + 8, "wire_size differs from the bytes written and from the sum of the field widths");

  { Net2 got{}; state::Status st = state::read(got, good.data(), N);
    check("roundtrip", st == state::Status::Ok && frameOf(got) == good, "read(write(s)) is not s"); }
  { Net2 s = initial(), t{}; for (int i = 0; i < 50; i++) { t.step(s); s = t; }
    Frame f = frameOf(s); Net2 got{}; state::Status st = state::read(got, f.data(), N);
    check("roundtrip-stepped", st == state::Status::Ok && frameOf(got) == f, "a stepped state does not survive a frame"); }

  Frame zero = frameOf(Net2{});
  { Frame f = good; f[2] ^= 0x40; Net2 got{}; state::Status st = state::read(got, f.data(), N);
    check("refuse-hash", st == state::Status::BadHash, "a frame of another composition was accepted");
    check("atomic-hash", frameOf(got) == zero, "a refused frame changed the state"); }
  { Net2 got{}; state::Status a = state::read(got, good.data(), N - 1), b = state::Status::Ok, c = state::Status::Ok; Frame longer = good; longer.push_back(0);
    b = state::read(got, longer.data(), N + 1); c = state::read(got, good.data(), 3);
    check("refuse-length", a == state::Status::BadLength && b == state::Status::BadLength && c == state::Status::BadLength, "a frame of the wrong length was accepted");
    check("atomic-length", frameOf(got) == zero, "a refused frame changed the state"); }
  { Locate l{"flag"}; src.each(l); Frame f = good; f[4 + l.off] = 2; Net2 got{}; state::Status st = state::read(got, f.data(), N);
    check("refuse-bool", st == state::Status::BadValue, "a bool byte of 2 was accepted");
    check("atomic-bool", frameOf(got) == zero, "a frame refused for its last-checked field had already changed the state");
    f[4 + l.off] = 0; Net2 g2{}; check("accept-bool0", state::read(g2, f.data(), N) == state::Status::Ok, "a bool byte of 0 was refused"); }

  { bool ok = true; int accepted = 0, refused = 0; Locate l{"flag"}; src.each(l);     // every frame with the right header: accepted and canonical, or refused for the bool only
    for (int i = 0; i < 200000 && ok; i++) {
      Frame f = good; for (unsigned k = 4; k < N; k++) f[k] = uint8_t(rnd());
      if (rnd() & 1) f[4 + l.off] = uint8_t(rnd() & 1);
      Net2 got{}; state::Status st = state::read(got, f.data(), N);
      if (st == state::Status::Ok) { accepted++; ok &= frameOf(got) == f; }
      else { refused++; ok &= st == state::Status::BadValue && f[4 + l.off] > 1 && frameOf(got) == zero; }
    }
    printf("      (%d accepted, %d refused)\n", accepted, refused);
    check("canonical", ok && accepted > 1000 && refused > 1000, "accepted frames are not canonical, or a frame was refused for another reason"); }
  printf("%d failed\n", failures);
  return failures;
}
