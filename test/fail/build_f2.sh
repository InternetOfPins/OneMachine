#!/usr/bin/env bash
# failCompose F2 verification: the return path (_f up the layers, _serve at the top), and the policies that are layers on it
# (Reply, Within, Overall, Backoff, Coalesce). Unit level and inside discoverCompose's world; native and AVR.
set -e
cd "$(dirname "$0")"
INC="-I ../../include -I ../../../HAPI/include -I ../../../OneBus/include"
INCABS="-I $(cd ../../include && pwd) -I $(cd ../../../HAPI/include && pwd) -I $(cd ../../../OneBus/include && pwd)"
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
rc=0
ok()   { echo "OK: $*"; }
bad()  { echo "FAIL: $*"; rc=1; }
skip() { echo "SKIP: $*"; }
F5B_COMMIT=94d9be8   # re-baselined at the C1 registry/identify split (2026-09-28): every image in this file moved by a small, explained amount then

echo "=== F2.1 unit level: return path, async, correlation, R-5, coalescing, back-off (mock time) ==="
g++ -std=c++17 -O2 -Wall -Wextra -I ../../include -I ../../../HAPI/include unit_f2.cpp -o "$OUT/u2" 2> "$OUT/u2.txt" || { bad "unit_f2 does not compile"; cat "$OUT/u2.txt"; }
[ "$(grep -c warning "$OUT/u2.txt" || true)" = 0 ] && "$OUT/u2" | tail -1 | grep -q '^OK' && ok "g++ $(g++ -dumpversion) -O2, 0 warnings" || { bad "unit_f2"; "$OUT/u2" | tail -3; }
g++ -std=c++17 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -I ../../include -I ../../../HAPI/include unit_f2.cpp -o "$OUT/u2s"
"$OUT/u2s" | tail -1 | grep -q '^OK' && ok "ASan+UBSan" || bad "unit_f2 under sanitizers"
if command -v clang++ >/dev/null; then
  clang++ -std=c++17 -O2 -Wall -Wextra -I ../../include -I ../../../HAPI/include unit_f2.cpp -o "$OUT/u2c" && "$OUT/u2c" | tail -1 | grep -q '^OK' && ok "clang++ $(clang++ -dumpversion)" || bad "unit_f2 under clang"
fi

echo; echo "=== F2.2 compile-fail: each case is rejected with its own message ==="
for pair in \
  "NEG_CAUSE_HIDES:a layer defines _f without being a CausePart" \
  "NEG_CAUSE_BODY:a CausePart body defines onCause(outcome, id), not _f(id)" \
  "NEG_ASYNC_NO_DEADLINE:R-2: an operation that can stay Pending needs a deadline" \
  "NEG_REPLY_NO_RETURNPATH:the edge must declare returnPath" \
  "NEG_OVERALL_BELOW_RETRY:an overall deadline bounds the retry loop: place it above Retry" \
  "NEG_OVERALL_NO_RETRY:an overall deadline bounds the retry loop: place it above Retry" \
  "NEG_COALESCE_NOT_IDEMPOTENT:Coalesce is for idempotent operations"; do
  def=${pair%%:*}; msg=${pair#*:}
  if g++ -std=c++17 -D$def -I ../../include -I ../../../HAPI/include -fsyntax-only unit_f2.cpp 2>&1 | grep -qF "$msg"; then ok "-D$def: $msg"; else bad "-D$def was not rejected with '$msg'"; fi
done

echo; echo "=== F2.3 mutation checks (unit level): each broken component must make the tests fail ==="
mutate2() {  # name file sed-expression
  rm -rf "$OUT/mut"; mkdir -p "$OUT/mut/oneMachine/fail"; cp ../../include/oneMachine/fail/*.h "$OUT/mut/oneMachine/fail/"
  sed -i "$3" "$OUT/mut/oneMachine/fail/$2"
  if cmp -s "../../include/oneMachine/fail/$2" "$OUT/mut/oneMachine/fail/$2"; then bad "mutation '$1' did not apply"; return; fi
  if g++ -std=c++17 -O2 -I "$OUT/mut" -I ../../include -I ../../../HAPI/include unit_f2.cpp -o "$OUT/mut/m" 2>/dev/null; then
    n=$("$OUT/mut/m" | grep -c '^FAIL' || true)
    [ "$n" -gt 0 ] && ok "$1 -> $n checks fail" || bad "mutation '$1' went undetected"
  else bad "$1 -> the mutated tree does not compile: a harness error, not a catch"; fi
}
mutate2 "a layer absorbs without asking the layer below (_f not delegated)" layers.h 's|Outcome _f(Id id) { return Self::onCause(T::_f(id), id); }|Outcome _f(Id id) { return Self::onCause(Outcome::Idle(), id); }|'
mutate2 "a read-once outcome can be read twice"                              layers.h 's|        if constexpr (ReadOnce) consumed = uint8_t(consumed \| b);|        |'
mutate2 "back-off never resets"                                              layers.h 's|      void gateStop()                   { next.disarm(); cur = 0; }|      void gateStop()                   { next.disarm(); }|'
mutate2 "back-off does not grow"                                             layers.h 's|        cur = i > MaxMs / 2 ? uint16_t(MaxMs) : uint16_t(i << 1);|        cur = i;|'
mutate2 "the deadline never fires (Within)"                                  layers.h 's|        if (o.isPending() \&\& (expired \& b)) {|        if (false) {|'
mutate2 "the overall deadline does not abort the loop"                       layers.h 's|        this->abortLoop();|        |'
mutate2 "coalescing still refuses and counts"                                layers.h 's|        if constexpr (T::coalesces) { if (c == Cause::Fresh \&\& this->holding()) return Outcome::Pending(); }|        if constexpr (T::coalesces) { if (c == Cause::Fresh \&\& this->holding()) { this->noteDrop(); return Outcome::Pending(); } }|'
mutate2 "an unknown id is answered as if it were one of the operations"      layers.h 's|        if (id >= N) return Outcome::Fail(Kind::Refused, id);|        if (id >= N) return Outcome::Ok();|'

echo; echo "=== F2.4 the full F2 composition inside discoverCompose's world (native, mock time) ==="
NATF="-std=c++17 -Wall -Wextra -DF5_STEP=10 -DF5_COUNT -DF5_TAP $INC"
g++ $NATF -O2 roundF5.cpp -o "$OUT/f2_full" 2> "$OUT/cc_f2.txt" || { bad "F2 composition does not compile"; cat "$OUT/cc_f2.txt"; }
w=$(grep -c warning "$OUT/cc_f2.txt" || true)
"$OUT/f2_full" > "$OUT/f2_full.txt"; sed 's/^/  /' "$OUT/f2_full.txt"
tail -1 "$OUT/f2_full.txt" | grep -q '^OK' && [ "$w" = 0 ] && ok "g++ -O2, 0 warnings: F5/F5b scenarios (F2's changed expectations listed in HANDOFF) + back-off on a stuck root and a stuck channel (Stale, not Gone; probes at 100/200/400 ms; interval resets), the return path at the top, coalescing" || bad "F2 composition (warnings=$w)"
g++ -std=c++17 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -DF5_STEP=10 -DF5_COUNT -DF5_TAP $INC roundF5.cpp -o "$OUT/f2_san"
"$OUT/f2_san" | tail -1 | grep -q '^OK' && ok "ASan+UBSan" || bad "F2 composition under sanitizers"
if command -v clang++ >/dev/null; then
  clang++ -std=c++17 -O2 -Wall -Wextra -DF5_STEP=10 -DF5_COUNT -DF5_TAP $INC roundF5.cpp -o "$OUT/f2_clang" && "$OUT/f2_clang" | tail -1 | grep -q '^OK' && ok "clang++ $(clang++ -dumpversion)" || bad "F2 composition under clang"
fi
for st in 7 8 9; do
  g++ -std=c++17 -O2 -Wall -Wextra -DF5_STEP=$st $INC roundF5.cpp -o "$OUT/f2_$st" 2> "$OUT/cc_f2_$st.txt" || { bad "F5_STEP=$st does not compile"; continue; }
  [ "$(grep -c warning "$OUT/cc_f2_$st.txt" || true)" = 0 ] && "$OUT/f2_$st" | tail -1 | grep -q '^OK' && ok "F5_STEP=$st (0 warnings): no faults -> R3's checksum" || bad "F5_STEP=$st"
done
g++ -std=c++17 -O2 -Wall -Wextra -DF5_STEP=10 -DF5_COUNT -DF5_TAP -DF5_SMALL_K $INC roundF5.cpp -o "$OUT/f2_small" 2> "$OUT/cc_small.txt"
"$OUT/f2_small" | tail -1 | grep -q '^OK' && [ "$(grep -c warning "$OUT/cc_small.txt" || true)" = 0 ] && ok "K overflow is loud: the row past the last slot reports Fail(Overflow) once through _serve(), and the counter stays" || bad "capacity test with the return path"
g++ -std=c++17 -O2 -Wall -Wextra -DF5_STEP=10 -DF5_COUNT -DF5_TAP -DF5_OVERALL_MS=60 $INC roundF5.cpp -o "$OUT/f2_over" 2> "$OUT/cc_over.txt"
"$OUT/f2_over" | tail -1 | grep -q '^OK' && [ "$(grep -c warning "$OUT/cc_over.txt" || true)" = 0 ] && ok "an overall deadline shorter than the loop cuts a device's retries (2 operations, not 3), reported as Timeout" || bad "overall deadline on a device"

echo; echo "=== F2.5 mutation checks (in the app): each broken policy must make the scenarios fail ==="
mutate5() {  # name file sed-expression [extra g++ flags]
  # file is either a fail:: header name (patched in a copy the -I order shadows the real one with) or roundF5.cpp
  # itself (patched in a copy, compiled instead of the original). -I . lets its own quote-includes ("../support/...")
  # fall back to this directory unchanged, wherever the copy sits.
  rm -rf "$OUT/mut"; mkdir -p "$OUT/mut/oneMachine/fail"; cp ../../include/oneMachine/fail/*.h "$OUT/mut/oneMachine/fail/"
  src="roundF5.cpp"
  if [ -n "$2" ]; then
    if [ "$2" = roundF5.cpp ]; then
      cp roundF5.cpp "$OUT/mut/roundF5.cpp"; src="$OUT/mut/roundF5.cpp"
      sed -i "$3" "$src"; if cmp -s roundF5.cpp "$src"; then bad "mutation '$1' did not apply"; return; fi
    else
      sed -i "$3" "$OUT/mut/oneMachine/fail/$2"
      if cmp -s "../../include/oneMachine/fail/$2" "$OUT/mut/oneMachine/fail/$2"; then bad "mutation '$1' did not apply"; return; fi
    fi
  fi
  if g++ -std=c++17 -O2 -DF5_STEP=10 -DF5_COUNT -DF5_TAP $4 -I "$OUT/mut" -I . $INCABS "$src" -o "$OUT/mut/m" 2>/dev/null; then
    n=$("$OUT/mut/m" | grep -c '^FAIL' || true)
    [ "$n" -gt 0 ] && ok "$1 -> $n checks fail" || bad "mutation '$1' went undetected"
  else bad "$1 -> the mutated tree does not compile: a harness error, not a catch"; fi
}
mutate5 "the bus gives up after M (Gone) instead of staying Stale on its back-off" roundF5.cpp 's|#define BUS_BACKOFF_RETRY fail::TickPart<fail::Retry<0>>|#define BUS_BACKOFF_RETRY fail::TickPart<fail::Retry<4>>|'
mutate5 "the bus back-off never resets"           layers.h   's|      void gateStop()                   { next.disarm(); cur = 0; }|      void gateStop()                   { next.disarm(); }|'
mutate5 "coalescing refuses and counts"           layers.h   's|        if constexpr (T::coalesces) { if (c == Cause::Fresh \&\& this->holding()) return Outcome::Pending(); }|        if constexpr (T::coalesces) { if (c == Cause::Fresh \&\& this->holding()) { this->noteDrop(); return Outcome::Pending(); } }|'
mutate5 "Retry does not answer Pending while it holds (the failure passes)" layers.h 's|if constexpr (T::returnPath) return Outcome::Pending(); }|}|'
mutate5 "K overflow is not reported through _serve()" ""       ""  "-DF5_SMALL_K -DF5_NEG_OVERFLOW_QUIET"

echo; echo "=== F2.6 AVR (avr-g++ $(avr-g++ -dumpversion), -Os, atmega328p, linked) ==="
FL="-std=gnu++17 -Os -mmcu=atmega328p -DF_CPU=16000000UL -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -Wall -Wextra"
avr_sum() {
  (timeout 60 simavr -g -m atmega328p -f 16000000 "$1" >/dev/null 2>&1 &); sleep 1
  timeout 55 avr-gdb -batch -ex 'set architecture avr' -ex 'target remote :1234' -ex 'break done' -ex 'continue' -ex 'x/1xh &g_sum' "$1" 2>/dev/null \
    | sed -n 's/.*<g_sum>:[[:space:]]*0x\(....\)$/\1/p'
  pkill -x simavr || true
}
have_sim=0; command -v simavr >/dev/null && command -v avr-gdb >/dev/null && have_sim=1

echo "--- the return path with no consumer and no layer defining it adds 0 B: every F5b composition, built from the F5b commit's own IOP-RnD sources ($F5B_COMMIT), against this tree"
if git rev-parse --is-inside-work-tree >/dev/null 2>&1 && git cat-file -e "$F5B_COMMIT" 2>/dev/null; then
  mkdir -p "$OUT/base"; (cd "$(git rev-parse --show-toplevel)" && git archive $F5B_COMMIT HAPI/failCompose HAPI/discoverCompose HAPI/rosCompose OneBus/mqtt | tar -x -C "$OUT/base")
  same=0; total=0
  for st in 0 1 2 3 4 5 6 15; do
    (cd "$OUT/base/HAPI/failCompose" && avr-g++ $FL $INCABS -DF5_STEP=$st roundF5.cpp -Wl,--gc-sections -o "$OUT/base_$st.elf" 2>/dev/null)
    avr-g++ $FL $INC -DF5_STEP=$st roundF5.cpp -Wl,--gc-sections -o "$OUT/s$st.elf" 2> "$OUT/s$st.txt"
    for e in base_$st s$st; do avr-objcopy -O binary -j .text -j .data "$OUT/$e.elf" "$OUT/$e.bin"; done
    total=$((total+1))
    if cmp -s "$OUT/base_$st.bin" "$OUT/s$st.bin"; then same=$((same+1)); else bad "step $st: flash image differs from the F5b build ($(avr-size "$OUT/base_$st.elf" | tail -1 | awk '{print $1"/"$3}') vs $(avr-size "$OUT/s$st.elf" | tail -1 | awk '{print $1"/"$3}'))"; fi
  done
  [ $same = $total ] && ok "steps 0-6 and 15: flash image (.text + .data) byte-identical to the F5b build, $total of $total (step 0 is also R3's own image; step 6: $(avr-size "$OUT/s6.elf" | tail -1 | awk '{print $1" / "$2" / "$3}') text/data/bss)"
else
  skip "byte-identity needs the IOP-RnD git history (commit $F5B_COMMIT); building steps 0-6, 15 here so the rest of this section still has images to work with"
  for st in 0 1 2 3 4 5 6 15; do avr-g++ $FL $INC -DF5_STEP=$st roundF5.cpp -Wl,--gc-sections -o "$OUT/s$st.elf" 2> "$OUT/s$st.txt"; done
fi

echo "--- F2 compositions: cost, symbols, indirect calls, parity"
names7="7 bus back-off: Retry without end + Backoff, in place of Retry<4> + Gate"
names8="8 + the return path: a Reply on the polled rows, Retry answers Pending, and the app reads _serve"
names9="9 + coalescing of idempotent reads (Coalesce store)"
names10="10 + an overall deadline around the device retry loop = full F2"
STRIP='function strip(s) { while (sub(/<[^<>]*>/, "", s)) ; return s }'
prev_t=0; prev_b=0
for st in 6 7 8 9 10 16 17 18; do
  [ $st -ne 6 ] && avr-g++ $FL $INC -DF5_STEP=$st roundF5.cpp -Wl,--gc-sections -o "$OUT/s$st.elf" 2> "$OUT/s$st.txt"
  w=$(grep -c warning "$OUT/s$st.txt" || true)
  read -r t d b < <(avr-size "$OUT/s$st.elf" | tail -1 | awk '{print $1, $2, $3}')
  case $st in
    6)  lab="F5b full (baseline)"; delta="";;
    7)  lab="$names7"; delta=$(printf "  (%+d text, %+d bss vs step 6)" $((t-9312)) $((b-624)));;
    8)  lab="$names8"; delta=$(printf "  (%+d text, %+d bss vs step %d)" $((t-prev_t)) $((b-prev_b)) 7);;
    9)  lab="$names9"; delta=$(printf "  (%+d text, %+d bss vs step 8)" $((t-prev_t)) $((b-prev_b)));;
    10) lab="$names10"; delta=$(printf "  (%+d text, %+d bss vs step 9)" $((t-prev_t)) $((b-prev_b)));;
    16) lab="alone over step 6: the return path (Reply + consumer)"; delta=$(printf "  (%+d text, %+d bss vs step 6)" $((t-9312)) $((b-624)));;
    17) lab="alone over step 6: coalescing";                          delta=$(printf "  (%+d text, %+d bss vs step 6)" $((t-9312)) $((b-624)));;
    18) lab="alone over step 6: an overall deadline";                 delta=$(printf "  (%+d text, %+d bss vs step 6)" $((t-9312)) $((b-624)));;
  esac
  printf "  step %-2d %-88s text %5d  data %3d  bss %4d%s\n" $st "$lab" $t $d $b "$delta"
  [ $st -le 10 ] && { prev_t=$t; prev_b=$b; }
  [ "$w" = 0 ] || bad "step $st: $w warnings"
  if avr-nm -C "$OUT/s$st.elf" | grep -E 'malloc|__divmod|__udivmod|__cxa_guard|_GLOBAL__sub_I|__do_global_ctors|__mul' >/dev/null; then bad "step $st: guard/ctor/malloc/divmod/mul symbol"; fi
  avr-objdump -dC --no-show-raw-insn "$OUT/s$st.elf" | awk "$STRIP"'
    /^[0-9a-f]+ <.*>:$/ { name=$0; sub(/^[0-9a-f]+ </,"",name); sub(/>:$/,"",name); name=strip(name); next }
    /\t(icall|eicall|ijmp|eijmp)/ { n[name]++ }
    END { for (f in n) printf "%d %s\n", n[f], f }' > "$OUT/ic$st.txt"
  if [ "$(wc -l < "$OUT/ic$st.txt")" = 1 ] && grep -q '^1 discover::World::pump()' "$OUT/ic$st.txt"; then :; else bad "step $st: indirect calls beyond the one in pump()"; cat "$OUT/ic$st.txt"; fi
  if [ $have_sim = 1 ]; then
    g++ -std=c++17 -O2 -DF5_STEP=$st -DF5_PARITY $INC roundF5.cpp -o "$OUT/par$st" 2>/dev/null
    nat=$("$OUT/par$st" | sed -n 's/^checksum 0x\(....\)$/\1/p'); avr=$(avr_sum "$OUT/s$st.elf")
    [ "${avr,,}" = "${nat,,}" ] || bad "step $st: simavr checksum 0x$avr != native 0x$nat"
  fi
done
ok "steps 6-10, 16-18: 0 warnings; no guard / ctor / malloc / __divmod / __mul symbols; pump() holds the only indirect call in every image"
[ $have_sim = 1 ] && ok "steps 6-10, 16-18: simulated ATmega328 checksum == native on the fault script (step 8-10 and 16 include the app reading _serve)"

echo "--- the tick fold and the return path in the full F2 image: 0 indirect calls (call-graph walk)"
./reach.py "$OUT/s10.elf" tickAll > "$OUT/reach10.txt" || true
head -1 "$OUT/reach10.txt" | sed 's/^/  tickAll: /'
grep -q 'indirect=0' "$OUT/reach10.txt" && grep -q 'BusEdge' "$OUT/reach10.txt" && ok "the tick fold (bus and device controllers, Overall, Reprobe, the re-issue path) has 0 indirect calls" || { bad "tick fold: an indirect call, or the fold is not in the reachable set"; cat "$OUT/reach10.txt"; }
./reach.py "$OUT/s10.elf" foldServe > "$OUT/reach10s.txt" || true
head -1 "$OUT/reach10s.txt" | sed 's/^/  foldServe (the app reading _serve): /'
grep -q 'indirect=0' "$OUT/reach10s.txt" && grep -q 'SlotTable' "$OUT/reach10s.txt" && ok "the return path at the top (_serve, Reply, the overflow report) has 0 indirect calls" || { bad "_serve path: an indirect call, or not found"; cat "$OUT/reach10s.txt"; }
p=$(./reach.py "$OUT/s10.elf" pump | head -1); echo "  pump, for comparison: $p"
[ "$p" = 'reachable=1 indirect=1' ] && ok "pump() is the one function with the one indirect call" || bad "pump: $p"
exit $rc
