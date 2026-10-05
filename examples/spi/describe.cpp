// The description of the spi example's link, written at build time from the same types the firmware is built from (src/air_tree.h): the device
// answers 'd' with the hash only, and the consumer reads the text from here (python: Tree(link, descriptions=<dir>)).
//   describe <dir>    writes <dir>/<hash>.txt (the text without the status) and prints its path
// The hash is FNV-1a over the text, folded at compile time by the same walk the firmware folds (bmpm::Fnv); this program checks the two agree.
// Built on the host: g++ -std=c++17 -I <the libraries' include dirs> describe.cpp (examples/spi/describe.py does it for PlatformIO).
#include <stdio.h>
#include <string>
#include "src/air_tree.h"

struct NoApp {};   // the description needs the machine's types only: no bus, no registry
using M = airTree::Machine<NoApp>;
template<typename Code> struct NoNote { static void fn(int32_t) {} };
using Pubs = airTree::Pubs<M, NoNote>;
struct Text { static constexpr bool noStatus = true; std::string s; void operator()(char c) { s += c; } };

template<typename P> constexpr void describeAll(P& put) { bmpm::describe<M, Pubs>(put, airTree::bus); airTree::cardLine(put); put('\n'); }
constexpr uint32_t folded() { bmpm::Fnv f; describeAll(f); return f.h; }

int main(int argc, char** argv) {
  if (argc != 2) { fprintf(stderr, "usage: describe <dir>\n"); return 2; }
  Text t; describeAll(t);
  uint32_t h = 2166136261u; for (char c : t.s) h = (h ^ uint8_t(c)) * 16777619u;
  constexpr uint32_t want = folded();
  if (h != want) { fprintf(stderr, "describe: the text hashes to %08x, the compile-time fold to %08x\n", unsigned(h), unsigned(want)); return 1; }
  char path[4096]; snprintf(path, sizeof path, "%s/%08x.txt", argv[1], unsigned(h));
  FILE* f = fopen(path, "w"); if (!f) { perror(path); return 1; }
  fwrite(t.s.data(), 1, t.s.size(), f); fclose(f);
  printf("%s\n", path);
  return 0;
}
