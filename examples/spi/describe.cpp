// The description of the spi example's link, written at build time from the same types the firmware is built from (src/air_tree.h): the device
// answers 'd' with the hash only, and the consumer reads the text from here (python: Tree(link, descriptions=<dir>)).
//   describe <dir>    writes <dir>/<hash>.txt (the text without the status) and <dir>/facts.txt, and prints the first path
// facts.txt is what a wiring tool needs, from the same C++ (python/onemachine/wiring.py reads it): the board's pins and their facts (OneChip's
// Esp8266Pins), who drives the buses' lines, and each driver's manifest (bus, addresses, ids, its lines and who drives them, its nodes or event).
// The hash is FNV-1a over the text, folded at compile time by the walk the firmware folds (Tree::describeStatic); this program checks the two agree.
// Built on the host: g++ -std=c++17 -I <the libraries' include dirs> describe.cpp (examples/spi/describe.py does it for PlatformIO).
#include <stdio.h>
#include <string>
#include "src/air_tree.h"

struct NoApp {};   // the description needs the machine's types only: no bus, no registry
using M = airTree::Machine<NoApp>;
using R = rfidTree::Machine<NoApp>;
static void noNote(int32_t) {}
template<typename Code> struct NoNote { static constexpr auto fn = &noNote; };
using Tree = airTree::Tree<M, airTree::Pubs<M, NoNote>, R, rfidTree::Pubs<R>>;
struct Text { std::string s; void operator()(char c) { s += c; } };

template<typename P> constexpr void describeAll(P& put) { Tree::describeStatic(put); }
constexpr uint32_t folded() { bmpm::Fnv f; describeAll(f); return f.h; }

// ---- facts.txt ------------------------------------------------------------------------------------------------------------------
using Board = airTree::Wiring::Board;
static void pinFact(std::string& o, const char* name, uint8_t g) {
  o += std::string("pin ") + name + " " + std::to_string(g);
  if (Board::strap(g)) o += Board::bootLevel(g) ? " strap high" : " strap low";
  if (!Board::hasIrq(g)) o += " noirq";
  o += "\n";
}
static std::string hex2(unsigned v) { char b[8]; snprintf(b, sizeof b, "0x%02X", v); return b; }
static std::string nm(state::Name n) { std::string r; for (unsigned i = 0; n.at(i); ++i) r += n.at(i); return r; }
template<typename N> static std::string kind() {
  if constexpr (rc522m::IsEvent<N>::value) return "event";
  else if constexpr (bmpm::IsGroup<N>::value) return "group";
  else if constexpr (bmpm::HasReg<N>::value) return "reg";
  else if constexpr (bmpm::HasConst<N>::value) return "const";
  else if constexpr (bmpm::HasDecimals<N>::value) return "value " + std::to_string(N::decimals);
  else return "value 0";
}
template<typename... C> static void children(std::string& o, const char* drv, const std::string& parent, unsigned at, hapi::Chain<C...>*) {
  unsigned i = 0; ((o += std::string("node ") + drv + " " + parent + "/" + nm(C::label()) + " " + std::to_string(at) + "/" + std::to_string(i++) + " " + kind<C>() + "\n"), ...);
}
template<typename... N> static void nodes(std::string& o, const char* drv, hapi::Chain<N...>*) {
  unsigned i = 0;
  ((o += std::string("node ") + drv + " " + nm(N::label()) + " " + std::to_string(i) + " " + kind<N>() + "\n",
    [&] { if constexpr (bmpm::IsGroup<N>::value) children(o, drv, nm(N::label()), i, static_cast<typename N::Body::Types*>(nullptr)); }(), ++i), ...);
}
static std::string facts() {
  std::string o = std::string("board ") + airTree::Wiring::board + "\n";
  const char* names[] = {"D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8"};
  const uint8_t gpios[] = {Board::D0, Board::D1, Board::D2, Board::D3, Board::D4, Board::D5, Board::D6, Board::D7, Board::D8};
  for (unsigned i = 0; i < 9; ++i) pinFact(o, names[i], gpios[i]);
  using discover::driveName; using I = discover::I2cLines; using S = discover::SpiLines;
  o += std::string("bus i2c sda ") + driveName(I::sda) + " scl " + driveName(I::scl) + "\n";
  o += std::string("bus spi sck ") + driveName(S::sck) + " miso " + driveName(S::miso) + " mosi " + driveName(S::mosi) + " cs " + driveName(S::cs) + "\n";
  { using F = bmpm::Manifest;
    o += std::string("driver ") + F::name + " " + F::bus + " addrs";
    for (uint8_t a : F::addrs) o += " " + hex2(a);
    o += " ids"; for (uint8_t i : F::ids) o += " " + hex2(i);
    o += "\n";
    nodes(o, F::name, static_cast<M::Nodes*>(nullptr)); }
  { using F = rc522::Manifest;
    o += std::string("driver ") + F::name + " " + F::bus + " ids";
    for (unsigned i = 0; i < F::Id::count; ++i) o += " " + hex2(F::Id::list[i]);
    o += "\n";
    F::pins([&](const char* role, discover::Drive d) { o += std::string("line ") + F::name + " " + role + " " + driveName(d) + "\n"; });
    nodes(o, F::name, static_cast<R::Nodes*>(nullptr)); }
  return o;
}

int main(int argc, char** argv) {
  if (argc != 2) { fprintf(stderr, "usage: describe <dir>\n"); return 2; }
  Text t; describeAll(t);
  uint32_t h = 2166136261u; for (char c : t.s) h = (h ^ uint8_t(c)) * 16777619u;
  constexpr uint32_t want = folded();
  if (h != want) { fprintf(stderr, "describe: the text hashes to %08x, the compile-time fold to %08x\n", unsigned(h), unsigned(want)); return 1; }
  char path[4096]; snprintf(path, sizeof path, "%s/%08x.txt", argv[1], unsigned(h));
  FILE* f = fopen(path, "w"); if (!f) { perror(path); return 1; }
  fwrite(t.s.data(), 1, t.s.size(), f); fclose(f);
  char fpath[4096]; snprintf(fpath, sizeof fpath, "%s/facts.txt", argv[1]);
  const std::string fx = facts();
  FILE* g = fopen(fpath, "w"); if (!g) { perror(fpath); return 1; }
  fwrite(fx.data(), 1, fx.size(), g); fclose(g);
  printf("%s\n", path);
  return 0;
}
