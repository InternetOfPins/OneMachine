#!/usr/bin/env bash
# OneMachine fail:: F1 verification (run by build.sh): native (g++/clang/ASan+UBSan), compile-fail cases, mutation checks, and AVR
# (byte-identity of the bare composition with discover::'s R1, cost per component, indirect calls, simavr parity).
# Exits non-zero if any claim does not hold.
set -e
cd "$(dirname "$0")"
. ../tools/lastok.sh
INC="-I ../../include -I ../../../HAPI/include -I ../../../OneBus/include"
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
rc=0
ok()   { echo "OK: $*"; }
bad()  { echo "FAIL: $*"; rc=1; }

echo "=== 1. unit level (Deadline; controller on a scripted edge; mock time) ==="
g++ -std=c++17 -O2 -Wall -Wextra $INC unit_f1.cpp -o "$OUT/unit"
LASTOK_N=2 lastok "$OUT/unit" && ok "g++ $(g++ -dumpversion) -O2, 0 warnings" || { bad "unit"; "$OUT/unit"; }
g++ -std=c++17 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all $INC unit_f1.cpp -o "$OUT/unit_san"
lastok "$OUT/unit_san" && ok "ASan+UBSan" || bad "unit under sanitizers"
if command -v clang++ >/dev/null; then
  clang++ -std=c++17 -O2 -Wall -Wextra $INC unit_f1.cpp -o "$OUT/unit_clang"
  lastok "$OUT/unit_clang" && ok "clang++ $(clang++ -dumpversion)" || bad "unit under clang"
fi
"$OUT/unit" | grep sizeof

echo; echo "=== 2. integration: one device edge in discoverCompose's world, every composition ==="
for st in 0 1 2 3 4 5; do
  g++ -std=c++17 -O2 -Wall -Wextra -DFAIL_STEP=$st $INC roundF1.cpp -o "$OUT/rf_$st" 2> "$OUT/cc_$st.txt" || { bad "step $st does not compile"; cat "$OUT/cc_$st.txt"; continue; }
  w=$(grep -c warning "$OUT/cc_$st.txt" || true)
  if lastok "$OUT/rf_$st" test && [ "$w" = 0 ]; then ok "FAIL_STEP=$st (native, 0 warnings): no faults -> checksum 0x5A03"; else bad "FAIL_STEP=$st (warnings=$w)"; "$OUT/rf_$st" test | tail -3; fi
done
"$OUT/rf_5" test | grep -E 'per row'
g++ -std=c++17 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -DFAIL_STEP=5 $INC roundF1.cpp -o "$OUT/rf_san"
lastok "$OUT/rf_san" test && ok "full composition under ASan+UBSan" || bad "full under sanitizers"
if command -v clang++ >/dev/null; then
  clang++ -std=c++17 -O2 -Wall -Wextra -DFAIL_STEP=5 $INC roundF1.cpp -o "$OUT/rf_clang"
  lastok "$OUT/rf_clang" test && ok "full composition under clang++" || bad "full under clang"
fi

echo; echo "=== 3. compile-fail: each case is rejected with its own message ==="
for pair in \
  "NEG_TICK_HIDES:a layer defines tick(now) without being a TickPart" \
  "NEG_TICK_BODY:a TickPart body defines onTick(now), not tick(now)" \
  "NEG_RETRY_OVERWRITE:R-3: Retry needs a Store below it that cannot overwrite" \
  "NEG_RECOVER_ABOVE_RETRY:R-4: Recover runs inside the failing operation, below Retry" \
  "NEG_RETRY_NO_GATE:Retry is gated: place a Gate below it" \
  "NEG_NO_STATUS:DetectError reports into Status"; do
  def=${pair%%:*}; msg=${pair#*:}
  if g++ -std=c++17 -D$def -I ../../include -I ../../../HAPI/include -fsyntax-only unit_f1.cpp 2>&1 | grep -qF "$msg"; then ok "-D$def: $msg"; else bad "-D$def was not rejected with '$msg'"; fi
done

echo; echo "=== 4. mutation checks: each broken component must make the tests fail ==="
mutate() {  # name file sed-expression
  rm -rf "$OUT/mut"; mkdir -p "$OUT/mut/oneMachine/fail"; cp ../../include/oneMachine/fail/*.h "$OUT/mut/oneMachine/fail/"
  sed -i "$3" "$OUT/mut/oneMachine/fail/$2"
  if cmp -s "../../include/oneMachine/fail/$2" "$OUT/mut/oneMachine/fail/$2"; then bad "mutation '$1' did not apply"; return; fi
  if g++ -std=c++17 -O2 -I "$OUT/mut" $INC unit_f1.cpp -o "$OUT/mut/m" 2>/dev/null; then
    n=$("$OUT/mut/m" | grep -c '^FAIL' || true)
    [ "$n" -gt 0 ] && ok "$1 -> $n checks fail" || bad "mutation '$1' went undetected"
  else bad "$1 -> the mutated tree does not compile: a harness error, not a catch"; fi
}
mutate "Recover fires on every kind (R-1 broken)"      layers.h   's|return k == Kind::Timeout \|\| k == Kind::Fault; }|return true; }|'
mutate "an edge's declared recovery kinds are ignored"  layers.h   's|    static constexpr bool recoversKind(Kind k) { return (EnvRecoverMask<Env>::value \& bit(k)) != 0; }|    static constexpr bool recoversKind(Kind) { return false; }|'
mutate "a row-aware recovery is not called with its row (busReset instead)" layers.h 's|    void busReset()                 { if constexpr (EnvRecoverRow<Env>::value) Env::recover(row); else Env::busReset(); }|    void busReset()                 { Env::busReset(); }|'
mutate "Retry ignores the edge's kind list"           layers.h   's|if (T::retryable(o.kind())) { this->hold();|if (true) { this->hold();|'
mutate "Gate interval off by one"                      layers.h   's|void gateStart(uint32_t now)      { next.arm(now, Ms); }|void gateStart(uint32_t now)      { next.arm(now, Ms + 1); }|'
mutate "Gate not re-armed after a re-issue"            layers.h   's|if (this->holding()) this->gateStart(now);||'
mutate "Deadline compare not wrap-safe"                deadline.h 's|return armed \&\& int32_t(now - at) >= 0;|return armed \&\& now >= at;|'
mutate "Deadline: unarmed can be due"                  deadline.h 's|return armed \&\& int32_t(now - at) >= 0;|return int32_t(now - at) >= 0;|'
mutate "exhaustion silent (no drop, no Gone)"          layers.h   's|this->release(); this->gateStop(); this->noteDrop(); this->exhausted();|this->release(); this->gateStop();|'
mutate "refusal of a fresh op not counted"             layers.h   's|!this->admitFresh()) { this->noteDrop(); return|!this->admitFresh()) { return|'
mutate "TickPart does not forward Base::tick"          layers.h   's|if constexpr (has_tick<T>::value) T::tick(now);||'
mutate "Status does not write the row state on failure" layers.h  's|else if (!stale) { stale = true; this->rowState(RowState::Stale); }|else if (!stale) { stale = true; }|'

echo; echo "=== 5. AVR (avr-g++ $(avr-g++ -dumpversion), -Os, atmega328p, linked) ==="
FL="-std=gnu++17 -Os -mmcu=atmega328p -DF_CPU=16000000UL -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -Wall -Wextra"
(cd ../discover && avr-g++ $FL $INC round1.cpp -Wl,--gc-sections -o "$OUT/r1_ref.elf")
avr-objcopy -O binary "$OUT/r1_ref.elf" "$OUT/r1_ref.bin"
avr_sum() {   # elf -> checksum stored in g_sum by the simulated ATmega328
  (timeout 40 simavr -g -m atmega328p -f 16000000 "$1" >/dev/null 2>&1 &); sleep 1
  timeout 35 avr-gdb -batch -ex 'set architecture avr' -ex 'target remote :1234' -ex 'break done' -ex 'continue' -ex 'x/1xh &g_sum' "$1" 2>/dev/null \
    | sed -n 's/.*<g_sum>:[[:space:]]*0x\(....\)$/\1/p'
  pkill -x simavr || true
}
have_sim=0; command -v simavr >/dev/null && command -v avr-gdb >/dev/null && have_sim=1

echo "--- bare composition vs discoverCompose R1"
for v in "0:" "0:-DBARE_TICK_CALL"; do
  st=${v%%:*}; extra=${v#*:}
  avr-g++ $FL $INC -DFAIL_STEP=$st $extra roundF1.cpp -Wl,--gc-sections -o "$OUT/bare.elf" 2> "$OUT/bare.txt"
  avr-objcopy -O binary "$OUT/bare.elf" "$OUT/bare.bin"
  label="bare${extra:+ + Ticker::run(0)}"
  if cmp -s "$OUT/r1_ref.bin" "$OUT/bare.bin"; then ok "$label: flash image byte-identical to discoverCompose R1 ($(stat -c%s "$OUT/bare.bin") B, sha $(sha256sum "$OUT/bare.bin" | cut -c1-12), $(avr-size "$OUT/bare.elf" | tail -1 | awk '{print $1" / "$2" / "$3}') text/data/bss)"
  else bad "$label: flash image differs from discoverCompose R1"; cmp -l "$OUT/r1_ref.bin" "$OUT/bare.bin" | head -3; fi
done
if [ $have_sim = 1 ]; then
  s=$(avr_sum "$OUT/bare.elf"); [ "${s,,}" = "5a03" ] && ok "bare, simavr checksum 0x$s (discoverCompose R1: 0x5A03)" || bad "bare simavr checksum '$s'"
fi

echo "--- cumulative compositions: cost, symbols, indirect calls, parity"
STRIP='function strip(s) { while (sub(/<[^<>]*>/, "", s)) ; return s }'
names=("bare, real bus (= R1)" "scaffold: scripted faults, no failure components" "+ Detect(error) + Status" "+ Recover" "+ Retry (+ Gate + Hold)" "+ Detect(deadline)  = full")
prev_t=0; prev_b=0
for st in 0 1 2 3 4 5; do
  avr-g++ $FL $INC -DFAIL_STEP=$st roundF1.cpp -Wl,--gc-sections -o "$OUT/s$st.elf" 2> "$OUT/s$st.txt"
  w=$(grep -c warning "$OUT/s$st.txt" || true)
  read -r t d b < <(avr-size "$OUT/s$st.elf" | tail -1 | awk '{print $1, $2, $3}')
  if [ $st -le 1 ]; then delta="";  else delta=$(printf "  (+%4d text, +%3d bss vs previous)" $((t-prev_t)) $((b-prev_b))); fi
  printf "  step %d  %-52s text %4d  data %3d  bss %3d%s\n" $st "${names[$st]}" $t $d $b "$delta"
  [ $st -ge 1 ] && { prev_t=$t; prev_b=$b; }
  [ "$w" = 0 ] || bad "step $st: $w warnings"
  if avr-nm -C "$OUT/s$st.elf" | grep -E 'malloc|__divmod|__udivmod|__cxa_guard|_GLOBAL__sub_I|__do_global_ctors' >/dev/null; then bad "step $st: guard/ctor/malloc/divmod symbol"; fi
  avr-objdump -dC --no-show-raw-insn "$OUT/s$st.elf" | awk "$STRIP"'
    /^[0-9a-f]+ <.*>:$/ { name=$0; sub(/^[0-9a-f]+ </,"",name); sub(/>:$/,"",name); name=strip(name); next }
    /\t(icall|eicall|ijmp|eijmp)/ { n[name]++ }
    END { for (f in n) printf "%d %s\n", n[f], f }' > "$OUT/ic$st.txt"
  if [ "$(wc -l < "$OUT/ic$st.txt")" = 1 ] && grep -q '^1 discover::World::pump()' "$OUT/ic$st.txt"; then :; else bad "step $st: indirect calls beyond the one in pump()"; cat "$OUT/ic$st.txt"; fi
  if [ $st -ge 1 ] && [ $have_sim = 1 ]; then
    nat=$("$OUT/rf_$st" scenario | sed -n 's/^checksum 0x\(....\)$/\1/p'); avr=$(avr_sum "$OUT/s$st.elf")
    [ "${avr,,}" = "${nat,,}" ] || bad "step $st: simavr checksum 0x$avr != native 0x$nat"
  fi
done
ok "steps 0-5: 0 warnings; no guard / ctor / malloc / __divmod symbols; pump() holds the only indirect call in every image"
[ $have_sim = 1 ] && ok "steps 1-5: simulated ATmega328 checksum == native scenario checksum (mock time, scripted faults)"
echo "  per row on the edge (bss delta / 8 rows): Detect(error)+Status $(( ( $(avr-size "$OUT/s2.elf" | tail -1 | awk '{print $3}') - $(avr-size "$OUT/s1.elf" | tail -1 | awk '{print $3}') ) / 8 )) B,"\
     "Recover 0 B, Retry+Gate+Hold $(( ( $(avr-size "$OUT/s4.elf" | tail -1 | awk '{print $3}') - $(avr-size "$OUT/s3.elf" | tail -1 | awk '{print $3}') ) / 8 )) B,"\
     "Detect(deadline) $(( ( $(avr-size "$OUT/s5.elf" | tail -1 | awk '{print $3}') - $(avr-size "$OUT/s4.elf" | tail -1 | awk '{print $3}') ) / 8 )) B"

echo "--- the tick fold: everything reachable from tickAll (a noinline wrapper around Ticker::run) in the full image"
./reach.py "$OUT/s5.elf" tickAll > "$OUT/reach5.txt" || true
head -1 "$OUT/reach5.txt" | sed 's/^/  /'
if grep -q 'indirect=0' "$OUT/reach5.txt" && grep -q 'TickFold' "$OUT/reach5.txt" && grep -q 'SensorA' "$OUT/reach5.txt"; then
  ok "the tick fold (TickFold::run, the driver's re-issue path, Status, the bus reads, the fan-out) has 0 indirect calls"
else bad "tick fold: an indirect call, or the fold is not in the reachable set"; cat "$OUT/reach5.txt"; fi
p=$(./reach.py "$OUT/s5.elf" pump | head -1); echo "  pump, for comparison: $p"
[ "$p" = 'reachable=1 indirect=1' ] && ok "pump() is the one function with the one indirect call (the row's poll)" || bad "pump: $p"

exit $rc
