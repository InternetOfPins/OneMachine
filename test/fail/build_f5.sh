#!/usr/bin/env bash
# failCompose F5 verification: failure components composed over discoverCompose's registry.
#   native (g++/clang/ASan+UBSan, mock time): the scenarios; compile-fail cases; mutation checks; the writer audit
#   AVR: the bare composition against discoverCompose R3's own image, cost per component, indirect calls, simavr parity
set -e
cd "$(dirname "$0")"
INC="-I ../../include -I ../../../HAPI/include -I ../../../OneBus/include"
INCABS="-I $(cd ../../include && pwd) -I $(cd ../../../HAPI/include && pwd) -I $(cd ../../../OneBus/include && pwd)"
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
rc=0
ok()   { echo "OK: $*"; }
bad()  { echo "FAIL: $*"; rc=1; }
skip() { echo "SKIP: $*"; }
R3_COMMIT=94d9be8   # re-baselined at the C1 registry/identify split (2026-09-28): every image in this file moved by a small, explained amount then

echo "=== F5.1 native: no failure component chosen is discoverCompose R3 (its checksum 0x4910), steps 0-5 ==="
for st in 0 1 2 3 4 5; do
  g++ -std=c++17 -O2 -Wall -Wextra -DF5_STEP=$st $INC roundF5.cpp -o "$OUT/f5_$st" 2> "$OUT/cc_$st.txt" || { bad "F5_STEP=$st does not compile"; cat "$OUT/cc_$st.txt"; continue; }
  w=$(grep -c warning "$OUT/cc_$st.txt" || true)
  if "$OUT/f5_$st" | tail -1 | grep -q '^OK' && [ "$w" = 0 ]; then ok "F5_STEP=$st (0 warnings): no faults -> R3's checksum"; else bad "F5_STEP=$st (warnings=$w)"; "$OUT/f5_$st" | tail -3; fi
done

echo; echo "=== F5.2 native: the scenarios on the full composition ==="
NATF="-std=c++17 -Wall -Wextra -DF5_STEP=6 -DF5_COUNT -DF5_TAP $INC"
g++ $NATF -O2 roundF5.cpp -o "$OUT/f5_full" 2> "$OUT/cc_full.txt" || { bad "full composition does not compile"; cat "$OUT/cc_full.txt"; }
w=$(grep -c warning "$OUT/cc_full.txt" || true)
"$OUT/f5_full" > "$OUT/full.txt"; cat "$OUT/full.txt" | sed 's/^/  /'
tail -1 "$OUT/full.txt" | grep -q '^OK' && [ "$w" = 0 ] && ok "g++ $(g++ -dumpversion) -O2, 0 warnings: 14 scenarios (transient device, transient bus timeout, NACK -> Stale -> Alive, gone for good, stuck root, stuck root forever, stuck channel, stuck channel forever, root seen from a channel, Unknown, Unknown on a read leg, own fault across a bus fault, fault script)" || bad "full composition (warnings=$w)"
g++ -std=c++17 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -DF5_STEP=6 -DF5_COUNT -DF5_TAP $INC roundF5.cpp -o "$OUT/f5_san"
"$OUT/f5_san" | tail -1 | grep -q '^OK' && ok "ASan+UBSan" || bad "full composition under sanitizers"
if command -v clang++ >/dev/null; then
  clang++ -std=c++17 -O2 -Wall -Wextra -DF5_STEP=6 -DF5_COUNT -DF5_TAP $INC roundF5.cpp -o "$OUT/f5_clang"
  "$OUT/f5_clang" | tail -1 | grep -q '^OK' && ok "clang++ $(clang++ -dumpversion)" || bad "full composition under clang"
fi

echo; echo "=== F5.2b native: a kind with more rows than its table holds (capacity) ==="
g++ -std=c++17 -O2 -Wall -Wextra -DF5_STEP=6 -DF5_COUNT -DF5_TAP -DF5_SMALL_K $INC roundF5.cpp -o "$OUT/f5_small" 2> "$OUT/cc_small.txt" || { bad "capacity build"; cat "$OUT/cc_small.txt"; }
"$OUT/f5_small" | tail -1 | grep -q '^OK' && [ "$(grep -c warning "$OUT/cc_small.txt" || true)" = 0 ] && ok "capacity: the row past the last slot is unprotected and counted, and never touches another row's state" || bad "capacity test"

echo; echo "=== F5.2c native: the bus-return hooks declared (-DF5_RECHECK): FCal reinitOnBusReturn, FSensorB recheck ==="
for st in 6 10; do
  g++ -std=c++17 -O2 -Wall -Wextra -DF5_STEP=$st -DF5_RECHECK -DF5_COUNT -DF5_TAP $INC roundF5.cpp -o "$OUT/f5_re$st" 2> "$OUT/cc_re$st.txt" || { bad "F5_STEP=$st with hooks does not compile"; cat "$OUT/cc_re$st.txt"; continue; }
  w=$(grep -c warning "$OUT/cc_re$st.txt" || true)
  "$OUT/f5_re$st" > "$OUT/re$st.txt"
  if tail -1 "$OUT/re$st.txt" | grep -q '^OK' && [ "$w" = 0 ]; then ok "F5_STEP=$st with hooks, 0 warnings: a bus that came back asks the drivers below it, once each, only those; a device down for its own reasons is not asked"; else bad "F5_STEP=$st with hooks (warnings=$w)"; grep '^FAIL' "$OUT/re$st.txt" | head -5; fi
done
g++ -std=c++17 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -DF5_STEP=10 -DF5_RECHECK -DF5_COUNT -DF5_TAP $INC roundF5.cpp -o "$OUT/f5_resan"
"$OUT/f5_resan" | tail -1 | grep -q '^OK' && ok "hooks: ASan+UBSan" || bad "hooks under sanitizers"

echo; echo "=== F5.2d native: a device without an ID register (-DF5_PRESENCE): the reprobe takes an ACK for its identity ==="
for st in 6 10; do
  g++ -std=c++17 -O2 -Wall -Wextra -DF5_STEP=$st -DF5_PRESENCE -DF5_COUNT -DF5_TAP $INC roundF5.cpp -o "$OUT/f5_pr$st" 2> "$OUT/cc_pr$st.txt" || { bad "F5_STEP=$st presenceOnly does not compile"; cat "$OUT/cc_pr$st.txt"; continue; }
  w=$(grep -c warning "$OUT/cc_pr$st.txt" || true)
  if "$OUT/f5_pr$st" | tail -1 | grep -q '^OK' && [ "$w" = 0 ]; then ok "F5_STEP=$st presenceOnly, 0 warnings: a returning device whose register 0 is not an id is Alive; without the flag it is refused and given up (scenario 2b, both builds)"; else bad "F5_STEP=$st presenceOnly (warnings=$w)"; fi
done

echo; echo "=== F5.3 the writer audit: status is written only through World::setStatus ==="
# the field and the registry's writer are private (discover:: R3 checks that); here: no F5 component names RawStatus,
# and in the tests every status change is accompanied by a counted setStatus call (the 'unexplained' counter)
FH="../../include/oneMachine/fail"
if grep -n 'RawStatus' "$FH/busedge.h" "$FH/devedge.h" "$FH/cause.h" ../support/faultbus.h roundF5.cpp | grep -v '^\S*:[0-9]*:\s*//' | grep -q .; then bad "an F5 file names RawStatus"; grep -n RawStatus "$FH/busedge.h" "$FH/devedge.h" "$FH/cause.h" ../support/faultbus.h roundF5.cpp; else ok "busedge.h devedge.h cause.h faultbus.h roundF5.cpp: no RawStatus"; fi
if grep -n 'RawStatus::' "$FH/layers.h" "$FH/outcome.h" "$FH/deadline.h" "$FH/inject.h" "$FH/world.h" | grep -q .; then bad "a failure component writes through RawStatus"; else ok "layers.h outcome.h deadline.h inject.h world.h: no RawStatus write (F1's own test app is the one declared exception: Env::rawStatus)"; fi

echo; echo "=== F5.4 compile-fail ==="
cf() {  # name flags message
  if g++ -std=c++17 $2 $INC -fsyntax-only roundF5.cpp 2>&1 | grep -qF "$3"; then ok "$1: $3"; else bad "$1 was not rejected with '$3'"; fi
}
cf "a stack that writes status, in an app without lifecycle" "-DF5_STEP=6 -DF5_NEG_NO_LIFECYCLE" "a stack that writes row status needs an app with lifecycle"
cf "Reprobe above Retry" "-DF5_STEP=6 -DF5_NEG_REPROBE_ABOVE_RETRY" "Reprobe takes over Retry's exhaustion: place it below Retry"

echo; echo "=== F5.5 mutation checks: each broken component must make the scenarios fail ==="
mutate5() {  # name relheader-under-oneMachine-or-roundF5.cpp sed-expression [extra g++ flags]
  # relheader is "fail/X.h" or "discover/X.h" (patched in a copy the -I order shadows the real one with) or
  # roundF5.cpp itself (patched in a copy, compiled instead of the original). -I . lets its own quote-includes
  # ("../support/...") fall back to this directory unchanged, wherever the copy sits.
  rm -rf "$OUT/mut"; mkdir -p "$OUT/mut/oneMachine/fail" "$OUT/mut/oneMachine/discover"
  cp ../../include/oneMachine/fail/*.h "$OUT/mut/oneMachine/fail/"; cp ../../include/oneMachine/discover/*.h "$OUT/mut/oneMachine/discover/"
  src="roundF5.cpp"
  if [ -n "$2" ]; then
    if [ "$2" = roundF5.cpp ]; then
      cp roundF5.cpp "$OUT/mut/roundF5.cpp"; src="$OUT/mut/roundF5.cpp"
      sed -i "$3" "$src"; if cmp -s roundF5.cpp "$src"; then bad "mutation '$1' did not apply"; return; fi
    else
      sed -i "$3" "$OUT/mut/oneMachine/$2"
      if cmp -s "../../include/oneMachine/$2" "$OUT/mut/oneMachine/$2"; then bad "mutation '$1' did not apply"; return; fi
    fi
  fi
  if g++ -std=c++17 -O2 -DF5_STEP=${M_STEP:-6} -DF5_COUNT -DF5_TAP $4 -I "$OUT/mut" -I . $INCABS "$src" -o "$OUT/mut/m" 2>/dev/null; then
    n=$("$OUT/mut/m" | grep -c '^FAIL' || true)
    [ "$n" -gt 0 ] && ok "$1 -> $n checks fail" || bad "mutation '$1' went undetected"
  else bad "$1 -> the mutated tree does not compile: a harness error, not a catch"; fi
}
mutate5 "a bus fault is handled per device (the device takes the failure)"  ""          "" "-DF5_NEG_BUS_FAULT_PER_DEVICE"
mutate5 "a bus that comes back leaves its subtree Stale (per-device recovery)" ""       "" "-DF5_NEG_RECOVER_PER_DEVICE"
mutate5 "a bus that comes back resurrects a device that is down on its own"   ""        "" "-DF5_NEG_NO_REAPPLY"
mutate5 "a device that comes back is not initialised again"                   ""        "" "-DF5_NEG_NO_REINIT"
mutate5 "Gone does not release the bindings" discover/registry.h '/Self::release(m);/d'
mutate5 "Gone does not clear the row's state" discover/registry.h '/Dev<>::Table::clear(m);/d'
mutate5 "Recover fires on every kind (R-1 broken)"  fail/layers.h  's|return k == Kind::Timeout \|\| k == Kind::Fault; }|return true; }|'
mutate5 "a Blocked re-probe counts as a miss"       fail/layers.h  's|else if (o.failed())   { if (++misses|else if (!o.isOk())   { if (++misses|'
mutate5 "the bus re-probe is not gated (1 ms)"      roundF5.cpp 's|fail::Gate<100>|fail::Gate<1>|g'
mutate5 "an operation's Stale is not written on a change only (every failure writes)" fail/layers.h 's|else if (!stale) { stale = true; this->rowState(RowState::Stale); }|else { stale = true; this->rowState(RowState::Stale); }|'
mutate5 "a component writes status behind the counter (RawStatus)" fail/devedge.h 's|W::setStatus(row, discover::Status(s));|discover::RawStatus::set(W::reg, row, discover::Status(s));|' "-DDISCOVER_TEST_RAW_STATUS"
mutate5 "a bus fault is blamed on the device's own bus (the bus above is not asked)" fail/busedge.h 's|      if (bus == root) return o;|      return o;|'
mutate5 "a stored operation is re-issued without routing (wrong channel)" fail/devedge.h 's|static void reissue(RowId row)        { W::route(W::reg.rows\[row\].parent); serve|static void reissue(RowId row)        { serve|'
mutate5 "an Unknown is not checked against the bus (no probe)"       ""        "" "-DF5_NEG_NO_UNKNOWN_PROBE"
mutate5 "rows of a kind share one slot (rank stuck at 0)" fail/slots.h '0,/return k;/s//return 0;/'
mutate5 "a slot is claimed by any row, not only its kind's (rank counts every row)" fail/slots.h \
  's|if (W::reg.rows\[r\].drv == d) ++k;|++k;|'
mutate5 "a row past the table shares the last slot instead of running unprotected" fail/slots.h \
  's|return i < K ? &slots\[i\] : nullptr;|return \&slots[i < K ? i : K - 1];|' "-DF5_SMALL_K"
mutate5 "a bus that comes back asks nobody (the hook is never called)"        ""        "" "-DF5_RECHECK -DF5_NEG_NO_RECHECK"
mutate5 "a bus that comes back asks every device, not only the ones under it"   ""        "" "-DF5_RECHECK -DF5_NEG_RECHECK_ALL"
M_STEP=10 mutate5 "a bus that comes back asks a device that is down for its own reasons (scenario 8, F2 build)"  ""        "" "-DF5_RECHECK -DF5_NEG_RECHECK_STALE"
mutate5 "a stateful device's init is run without routing to its channel"       fail/devedge.h 's|else if constexpr (ReinitOnBusReturn<Impl>::value) { W::route(W::reg.rows\[row\].parent); Impl::reinit(row); }|else if constexpr (ReinitOnBusReturn<Impl>::value) { Impl::reinit(row); }|' "-DF5_RECHECK"
mutate5 "a presence-only device is still identified by register 0 at the reprobe" ""        "" "-DF5_PRESENCE -DF5_NEG_PRESENCE_READS_ID"
mutate5 "a device's Unknown is not retried"        fail/devedge.h 's|KindSet<Kind::Absent, Kind::Unknown>::mask|KindSet<Kind::Absent>::mask|'

echo; echo "=== F5.6 AVR (avr-g++ $(avr-g++ -dumpversion), -Os, atmega328p, linked) ==="
FL="-std=gnu++17 -Os -mmcu=atmega328p -DF_CPU=16000000UL -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -Wall -Wextra"
avr_sum() {   # elf -> checksum stored in g_sum by the simulated ATmega328
  (timeout 60 simavr -g -m atmega328p -f 16000000 "$1" >/dev/null 2>&1 &); sleep 1
  timeout 55 avr-gdb -batch -ex 'set architecture avr' -ex 'target remote :1234' -ex 'break done' -ex 'continue' -ex 'x/1xh &g_sum' "$1" 2>/dev/null \
    | sed -n 's/.*<g_sum>:[[:space:]]*0x\(....\)$/\1/p'
  pkill -x simavr || true
}
have_sim=0; command -v simavr >/dev/null && command -v avr-gdb >/dev/null && have_sim=1

avr-g++ $FL $INC -DF5_STEP=0 roundF5.cpp -Wl,--gc-sections -o "$OUT/s0.elf" 2> "$OUT/s0.txt"
echo "--- the bare composition against discover::'s own R3 image, built from the R3 commit's IOP-RnD sources ($R3_COMMIT)"
if git rev-parse --is-inside-work-tree >/dev/null 2>&1 && git cat-file -e "$R3_COMMIT" 2>/dev/null; then
  mkdir -p "$OUT/r3src"; (cd "$(git rev-parse --show-toplevel)" && git archive $R3_COMMIT HAPI/discoverCompose HAPI/rosCompose | tar -x -C "$OUT/r3src")
  (cd "$OUT/r3src/HAPI/discoverCompose" && avr-g++ $FL $INCABS round3.cpp -Wl,--gc-sections -o "$OUT/r3_ref.elf")
  for e in r3_ref s0; do avr-objcopy -O binary -j .text -j .data "$OUT/$e.elf" "$OUT/$e.bin"; done
  if cmp -s "$OUT/r3_ref.bin" "$OUT/s0.bin"; then
    ok "bare: flash image (.text + .data) byte-identical to discover::'s R3 all-on ($(stat -c%s "$OUT/s0.bin") B, sha $(sha256sum "$OUT/s0.bin" | cut -c1-12), $(avr-size "$OUT/s0.elf" | tail -1 | awk '{print $1" / "$2" / "$3}') text/data/bss)"
  else bad "bare: flash image differs from discover::'s R3"; cmp -l "$OUT/r3_ref.bin" "$OUT/s0.bin" | wc -l; fi
else skip "byte-identity needs the IOP-RnD git history (commit $R3_COMMIT)"; fi
if [ $have_sim = 1 ]; then
  s=$(avr_sum "$OUT/s0.elf"); [ "${s,,}" = "4910" ] && ok "bare, simavr checksum 0x$s (discover::'s R3: 0x4910)" || bad "bare simavr checksum '$s'"
fi

echo "--- cumulative compositions: cost, symbols, indirect calls, parity"
STRIP='function strip(s) { while (sub(/<[^<>]*>/, "", s)) ; return s }'
names=("bare = R3's program and bus" "scaffold: reporting bus core, fault script, no failure components" "+ bus edge: Detect(error) + Status" "+ Retry (+ Gate + Hold): the gated bus re-probe" "+ Recover" "+ device edge: Detect + Retry + Status" "+ Reprobe  = full")
prev_t=0; prev_b=0
for st in 0 1 2 3 4 5 6; do
  [ $st -gt 0 ] && avr-g++ $FL $INC -DF5_STEP=$st roundF5.cpp -Wl,--gc-sections -o "$OUT/s$st.elf" 2> "$OUT/s$st.txt"
  w=$(grep -c warning "$OUT/s$st.txt" || true)
  read -r t d b < <(avr-size "$OUT/s$st.elf" | tail -1 | awk '{print $1, $2, $3}')
  if [ $st -le 1 ]; then delta=""; else delta=$(printf "  (+%4d text, +%3d bss vs previous)" $((t-prev_t)) $((b-prev_b))); fi
  printf "  step %d  %-62s text %4d  data %3d  bss %3d%s\n" $st "${names[$st]}" $t $d $b "$delta"
  [ $st -ge 1 ] && { prev_t=$t; prev_b=$b; }
  [ "$w" = 0 ] || bad "step $st: $w warnings"
  if avr-nm -C "$OUT/s$st.elf" | grep -E 'malloc|__divmod|__udivmod|__cxa_guard|_GLOBAL__sub_I|__do_global_ctors' >/dev/null; then bad "step $st: guard/ctor/malloc/divmod symbol"; fi
  avr-objdump -dC --no-show-raw-insn "$OUT/s$st.elf" | awk "$STRIP"'
    /^[0-9a-f]+ <.*>:$/ { name=$0; sub(/^[0-9a-f]+ </,"",name); sub(/>:$/,"",name); name=strip(name); next }
    /\t(icall|eicall|ijmp|eijmp)/ { n[name]++ }
    END { for (f in n) printf "%d %s\n", n[f], f }' > "$OUT/ic$st.txt"
  if [ "$(wc -l < "$OUT/ic$st.txt")" = 1 ] && grep -q '^1 discover::World::pump()' "$OUT/ic$st.txt"; then :; else bad "step $st: indirect calls beyond the one in pump()"; cat "$OUT/ic$st.txt"; fi
  if [ $st -ge 1 ] && [ $have_sim = 1 ]; then
    g++ -std=c++17 -O2 -DF5_STEP=$st -DF5_PARITY $INC roundF5.cpp -o "$OUT/par$st" 2>/dev/null
    nat=$("$OUT/par$st" | sed -n 's/^checksum 0x\(....\)$/\1/p'); avr=$(avr_sum "$OUT/s$st.elf")
    [ "${avr,,}" = "${nat,,}" ] || bad "step $st: simavr checksum 0x$avr != native 0x$nat"
  fi
done
ok "steps 0-6: 0 warnings; no guard / ctor / malloc / __divmod symbols; pump() holds the only indirect call in every image"
[ $have_sim = 1 ] && ok "steps 1-6: simulated ATmega328 checksum == native, on the fault script (mock time, mock faults)"

read -r t1r d1r b1r < <(avr-size "$OUT/s1.elf" | tail -1 | awk '{print $1, $2, $3}')
read -r t6r d6r b6r < <(avr-size "$OUT/s6.elf" | tail -1 | awk '{print $1, $2, $3}')
if [ $((b6r-b1r)) -lt 150 ]; then ok "the tables hold the rows of a kind, not the registry's: the full composition adds $((b6r-b1r)) B of RAM over the scaffold (was 650 B with a slot per registry row)"; else bad "RAM added by the full composition: $((b6r-b1r)) B"; fi

echo "--- alone: the device edge without a bus controller"
avr-g++ $FL $INC -DF5_STEP=15 roundF5.cpp -Wl,--gc-sections -o "$OUT/s15.elf" 2> "$OUT/s15.txt"
[ "$(grep -c warning "$OUT/s15.txt" || true)" = 0 ] || bad "step 15: warnings"
read -r t15 d15 b15 < <(avr-size "$OUT/s15.elf" | tail -1 | awk '{print $1, $2, $3}')
read -r t1 d1 b1 < <(avr-size "$OUT/s1.elf" | tail -1 | awk '{print $1, $2, $3}')
read -r t2 d2 b2 < <(avr-size "$OUT/s2.elf" | tail -1 | awk '{print $1, $2, $3}')
read -r t4 d4 b4 < <(avr-size "$OUT/s4.elf" | tail -1 | awk '{print $1, $2, $3}')
printf "  bus edge alone (step 4 over step 1):    +%4d text, +%3d bss\n" $((t4-t1)) $((b4-b1))
printf "  device edge alone (step 15 over 1):     +%4d text, +%3d bss\n" $((t15-t1)) $((b15-b1))
printf "  bus edge Detect+Status alone (step 2):  +%4d text, +%3d bss\n" $((t2-t1)) $((b2-b1))
avr-objdump -dC --no-show-raw-insn "$OUT/s15.elf" | awk "$STRIP"'
  /^[0-9a-f]+ <.*>:$/ { name=$0; sub(/^[0-9a-f]+ </,"",name); sub(/>:$/,"",name); name=strip(name); next }
  /\t(icall|eicall|ijmp|eijmp)/ { n[name]++ }
  END { for (f in n) printf "%d %s\n", n[f], f }' > "$OUT/ic15.txt"
[ "$(wc -l < "$OUT/ic15.txt")" = 1 ] && grep -q '^1 discover::World::pump()' "$OUT/ic15.txt" && ok "step 15: pump() holds the only indirect call" || bad "step 15: indirect calls beyond pump()"

echo "--- the tick fold: everything reachable from tickAll in the full image"
./reach.py "$OUT/s6.elf" tickAll > "$OUT/reach6.txt" || true
head -1 "$OUT/reach6.txt" | sed 's/^/  /'
if grep -q 'indirect=0' "$OUT/reach6.txt" && grep -q 'BusEdge' "$OUT/reach6.txt" && grep -q 'DevEdge' "$OUT/reach6.txt"; then
  ok "the tick fold (the bus controllers, the device controllers, the probes, the re-issue path, the fan-out) has 0 indirect calls"
else bad "tick fold: an indirect call, or the fold is not in the reachable set"; cat "$OUT/reach6.txt"; fi
p=$(./reach.py "$OUT/s6.elf" pump | head -1); echo "  pump, for comparison: $p"
[ "$p" = 'reachable=1 indirect=1' ] && ok "pump() is the one function with the one indirect call (the row's poll)" || bad "pump: $p"
exit $rc
