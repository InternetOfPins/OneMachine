#!/usr/bin/env bash
# discoverCompose Round R3b verification (see HANDOFF.md, Round R3b).
#   identity : the R1, R2 and R3 flash images equal the images built from the previous commit's own sources (cmp).
#   native   : g++ -O2, ASan+UBSan, clang; the scenarios, with the mock's per-address counts asserted.
#   guards   : each conflict rule is rejected with its own message; each mutation must make the scenarios fail.
#   AVR      : size per entry kind (alone / marginal), symbols of what is not listed, hygiene, icall inventory, simavr == native.
#   seam     : what changed in R1..R3 files.
# Exits non-zero if any claim fails. Overrides: HAPI=<dir> ONEBUS=<dir> (checkouts holding include/).
set -u
cd "$(dirname "$0")"
HAPI=${HAPI:-../../../HAPI}; ONEBUS=${ONEBUS:-../../../OneBus}
HAPI_ABS=$(cd "$HAPI" && pwd); ONEBUS_ABS=$(cd "$ONEBUS" && pwd)
INC="-I ../../include -I $HAPI/include -I $ONEBUS/include"
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
rc=0
ok()   { echo "OK:   $*"; }
bad()  { echo "FAIL: $*"; rc=1; }
skip() { echo "SKIP: $*"; }
BASE=94d9be8       # the C1 registry/identify split (2026-09-28): images and the "unchanged since" seam both re-based here, not at the pre-R3b commit

AVRF="-std=gnu++17 -Os -mmcu=atmega328p -DF_CPU=16000000UL -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti"
AVR="avr-g++ $AVRF -Wall -Wextra $INC"
flash_image() { avr-g++ $AVRF -I "$HAPI_ABS/include" -I "$ONEBUS_ABS/include" "$2" -Wl,--gc-sections -o "$1.elf" 2>/dev/null && avr-objcopy -O binary -j .text -j .data "$1.elf" "$1.bin"; }

echo "=== R1, R2 and R3 images against their own recorded baselines (avr-g++ $(avr-g++ -dumpversion)) ==="
avr-g++ $AVRF $INC round1.cpp -Wl,--gc-sections -o "$OUT/new_round1.elf"
avr-g++ $AVRF $INC -DR2_NO_MQTT round2.cpp -Wl,--gc-sections -o "$OUT/new_round2.elf"
avr-g++ $AVRF $INC round3.cpp -Wl,--gc-sections -o "$OUT/new_round3.elf"
../tools/baseline.sh check r1_avr "$OUT/new_round1.elf" || rc=1
../tools/baseline.sh check r2_avr "$OUT/new_round2.elf" || rc=1
../tools/baseline.sh check r3_avr "$OUT/new_round3.elf" || rc=1
R1=$(avr-size "$OUT/new_round1.elf" | awk 'NR==2{print $1"/"$2"/"$3}'); R2=$(avr-size "$OUT/new_round2.elf" | awk 'NR==2{print $1"/"$2"/"$3}'); R3=$(avr-size "$OUT/new_round3.elf" | awk 'NR==2{print $1"/"$2"/"$3}')
[ "$R1" = "2496/56/201" ]   && ok "R1 image text/data/bss $R1"          || bad "R1 image is $R1, was 2496/56/201"
[ "$R2" = "5118/152/661" ]  && ok "R2 all-bindings image text/data/bss $R2" || bad "R2 image is $R2, was 5118/152/661"
[ "$R3" = "4748/104/462" ]  && ok "R3 all-on image text/data/bss $R3"   || bad "R3 image is $R3, was 4748/104/462"

echo; echo "=== native g++ $(g++ -dumpversion) -O2 ==="
if g++ -std=c++17 -O2 -Wall -Wextra $INC round3b.cpp -o "$OUT/r3b" && "$OUT/r3b" > "$OUT/r3b.out"; then
  grep -E "^(  address bytes|checks|OK:)" "$OUT/r3b.out"; ok "round3b native scenarios"
else bad "round3b native"; grep -E "^FAIL" "$OUT/r3b.out" | head; fi
echo; echo "=== native -O1 -fsanitize=address,undefined ==="
if g++ -std=c++17 -O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=all $INC round3b.cpp -o "$OUT/r3bsan" && "$OUT/r3bsan" | tail -1 | grep -q "^OK"; then ok "round3b under ASan+UBSan"; else bad "round3b under ASan+UBSan"; fi
if command -v clang++ >/dev/null; then
  echo; echo "=== native clang++ $(clang++ -dumpversion) -O2 ==="
  if clang++ -std=c++17 -O2 -Wall -Wextra $INC round3b.cpp -o "$OUT/r3bclang" && "$OUT/r3bclang" | tail -1 | grep -q "^OK"; then ok "round3b under clang"; else bad "round3b under clang"; fi
fi

echo; echo "=== compile-fail: each conflict rule is rejected with its own message ==="
for pair in "NEG_REPEAT:driver list has a repeated type: an entry is listed twice" \
            "NEG_PIN_PIN:an address is pinned by two entries" \
            "NEG_CLAIM_PIN:a claim without a row covers every address of a later entry" \
            "NEG_PIN_PROBE:a pin covers every address of a later entry" \
            "NEG_PROTECT:an entry that writes to the device covers an address of a Protect range" \
            "NEG_IGNORE_BRIDGE_USED:IgnoreBridge names a bridge the entries also use elsewhere"; do
  def=${pair%%:*}; msg=${pair#*:}
  if g++ -std=c++17 -D$def $INC -fsyntax-only round3b.cpp 2>&1 | grep -q "static assertion failed: $msg"; then ok "-D$def rejected: $msg"
  else bad "-D$def was not rejected with '$msg'"; fi
done

echo; echo "=== mutations: each a sed-patched copy of the real header, not a build-time switch (a build that does not compile is a failure) ==="
mutate() {  # name relheader sed-expr
  rm -rf "$OUT/mut"; mkdir -p "$OUT/mut/oneMachine/discover"; cp ../../include/oneMachine/discover/*.h "$OUT/mut/oneMachine/discover/"
  sed -i "$3" "$OUT/mut/oneMachine/$2"
  if cmp -s "../../include/oneMachine/$2" "$OUT/mut/oneMachine/$2"; then bad "mutation '$1' did not apply"; return; fi
  if ! g++ -std=c++17 -O1 -I "$OUT/mut" $INC round3b.cpp -o "$OUT/mut/neg" 2> "$OUT/mut/neg.err"; then bad "$1 does not compile"; head -3 "$OUT/mut/neg.err"; return; fi
  bash -c '"$0" > "$1" 2>&1' "$OUT/mut/neg" "$OUT/mut/neg.out" 2>/dev/null; st=$?
  n=$(grep -c '^FAIL' "$OUT/mut/neg.out")
  if [ "$n" -ge 1 ]; then
    if [ $st -ge 128 ]; then ok "$1: caught, $n checks fail, then the run crashes (signal $((st-128)))"
    else ok "$1: caught, $n checks fail"; fi
  else bad "$1: no check fails (exit $st)"; fi
}
mutate "stage 1 skipped: the probes reach addresses nothing answers" discover/identify.h \
  's|static bool present(uint8_t addr, Seen\& seen, bool readOnly) {|static bool present(uint8_t addr, Seen\& seen, bool readOnly) { return true;|'
mutate "a claim creates a row" discover/identify.h \
  's|        if constexpr (!std::is_void<typename ClearedOf<E>::Type>::value) ClearedOf<E>::Type::clear(addr);|        W::reg.add(addr, nullptr, bus, false);\n        if constexpr (!std::is_void<typename ClearedOf<E>::Type>::value) ClearedOf<E>::Type::clear(addr);|'
mutate "the read-probe of a protected range is a write-probe" discover/identify.h \
  's|const oneBus::ProbeKind k = readOnly ? oneBus::ProbeKind::Read : oneBus::probeKindFor(addr);|const oneBus::ProbeKind k = oneBus::ProbeKind::Write;|'
mutate "the list order is not respected" discover/identify.h \
  's|hapi::Chain<K\.\.\., typename Norm<E>::Type>|hapi::Chain<typename Norm<E>::Type, K...>|'
mutate "stage 1 repeated for every entry over the same address" discover/identify.h \
  's|int8_t\& m = seen.known\[uint8_t(k)\];|int8_t\& m = seen.known[uint8_t(k)]; m = -1;|'
mutate "Use<Own,D> ignores the driver's idReg" discover/identify.h \
  's|Use<IdProbe<IdRegOf<D>::value, D::id, D::addrLo, D::addrHi>, D>|Use<IdProbe<0, D::id, D::addrLo, D::addrHi>, D>|'
mutate "a bus timeout on the presence probe is read as absence" discover/identify.h \
  '/if (!ok \&\& oneBus::isBusFault(oneBus::causeOf<Twi>())) ok = oneBus::probe<Twi>(addr, k);/d'
mutate "polled is ignored: a device that produces nothing is never refreshed" discover/driver.h \
  '/template<typename D> struct PolledOf<D, std::void_t<decltype(D::polled)>> : std::bool_constant<D::polled> {};/d'
mutate "IgnoreBridge is ignored like a plain Ignore: the stale selection is never cleared" discover/identify.h \
  '/if constexpr (!std::is_void<typename ClearedOf<E>::Type>::value) ClearedOf<E>::Type::clear(addr);/d'

echo; echo "=== avr-g++ $(avr-g++ -dumpversion) atmega328p: one image per entry choice ==="
STRIP='function strip(s) { while (sub(/<[^<>]*>/, "", s)) ; return s }'
sizes() { avr-size "$1" | awk 'NR==2{print $1+$2, $2+$3}'; }
hygiene() { avr-nm -C "$1" | grep -E 'malloc|__divmod|__udivmod|__cxa_guard|_GLOBAL__sub_I|__do_global_ctors|__divmodhi|__mulhi|__mulsi|__divmodsi' ; }
icalls() {
  avr-objdump -dC --no-show-raw-insn "$1" | awk "$STRIP"'
    /^[0-9a-f]+ <.*>:$/ { name=$0; sub(/^[0-9a-f]+ </,"",name); sub(/>:$/,"",name); name=strip(name); next }
    /\t(icall|eicall|ijmp|eijmp)/ { n[name]++ }
    END { for (f in n) printf "%d %s\n", n[f], f }'
}
ALL="-DR3B_TWO -DR3B_PIN -DR3B_CLONE -DR3B_CLAIM -DR3B_EEP"
declare -A VAR=(
  [base]=""
  [two]="-DR3B_TWO"        [pin]="-DR3B_PIN"        [clone]="-DR3B_CLONE"      [claim]="-DR3B_CLAIM"      [eep]="-DR3B_EEP"
  [all]="$ALL"
  [noTwo]="-DR3B_PIN -DR3B_CLONE -DR3B_CLAIM -DR3B_EEP"
  [noPin]="-DR3B_TWO -DR3B_CLONE -DR3B_CLAIM -DR3B_EEP"
  [noClone]="-DR3B_TWO -DR3B_PIN -DR3B_CLAIM -DR3B_EEP"
  [noClaim]="-DR3B_TWO -DR3B_PIN -DR3B_CLONE -DR3B_EEP"
  [noEep]="-DR3B_TWO -DR3B_PIN -DR3B_CLONE -DR3B_CLAIM"
)
ORDER="base two pin clone claim eep all noTwo noPin noClone noClaim noEep"
declare -A FL RAM
avr_ok=1
for v in $ORDER; do
  if ! $AVR ${VAR[$v]} round3b.cpp -Wl,--gc-sections -o "$OUT/a_$v.elf" 2> "$OUT/a_$v.err"; then bad "avr build [$v]"; head -5 "$OUT/a_$v.err"; avr_ok=0; continue; fi
  if [ -s "$OUT/a_$v.err" ]; then bad "avr build [$v] warns"; head -3 "$OUT/a_$v.err"; fi
  read -r FL[$v] RAM[$v] <<< "$(sizes "$OUT/a_$v.elf")"
  if hygiene "$OUT/a_$v.elf" > "$OUT/hyg_$v.txt" && [ -s "$OUT/hyg_$v.txt" ]; then bad "[$v] forbidden symbols: $(head -3 "$OUT/hyg_$v.txt" | tr '\n' ' ')"; fi
  icalls "$OUT/a_$v.elf" > "$OUT/ic_$v.txt"
  n=$(wc -l < "$OUT/ic_$v.txt"); pump=$(awk '/World<.*>::pump\(\)|World::pump\(\)/ {print $1}' "$OUT/ic_$v.txt")
  if [ "$n" != "1" ] || [ "${pump:-0}" != "1" ]; then bad "[$v] indirect calls: want exactly 1, in pump; got:"; cat "$OUT/ic_$v.txt"; fi
done
if [ "$avr_ok" = 1 ]; then
  ok "every variant links without warnings; no heap / divide / multiply / guard / global-ctor symbols; pump holds the only indirect call (1 icall) in every image"
  echo; echo "--- flash / RAM per entry kind (B; flash = text+data, RAM = data+bss; the mock and the scenario are the same in every image) ---"
  echo "    'alone' = over 'base' (R1's three drivers, bare, one-stage); 'marginal' = 'all' minus 'all without it'"
  printf '  %-9s %7s %7s\n' variant flash RAM
  for v in $ORDER; do printf '  %-9s %7d %7d\n' "$v" "${FL[$v]}" "${RAM[$v]}"; done
  printf '\n  %-52s %8s %8s   %8s %8s\n' entry alone.fl alone.RAM marg.fl marg.RAM
  for pair in "Use<Own,D> for the three drivers (two-stage):two:noTwo" "Use<AddressProbe<0x27>, TextDisplay> (a pin):pin:noPin" \
              "Use<IdProbe<0,0xA9,0x48>, SensorA> (a clone):clone:noClone" "Ignore<0x72> (a claim, no row):claim:noClaim" \
              "Protect + Use<AddressProbe<0x50,0x51>, Eeprom>:eep:noEep"; do
    IFS=: read -r label a n <<< "$pair"
    printf '  %-52s %+8d %+8d   %+8d %+8d\n' "$label" "$((FL[$a]-FL[base]))" "$((RAM[$a]-RAM[base]))" "$((FL[all]-FL[$n]))" "$((RAM[all]-RAM[$n]))"
  done
  printf '  %-52s %+8d %+8d\n' "all five, over base" "$((FL[all]-FL[base]))" "$((RAM[all]-RAM[base]))"
  echo "  note: -Os inlining is not additive: gcc changes what it folds into World::scan as entries are added, so a marginal can be negative"
  scan_of() { avr-nm -C -S -t d "$1" | awk "$STRIP"'{ name=strip($0); if (name ~ /World::scan\(/) print $2+0 }'; }
  echo "        (World::scan is $(scan_of "$OUT/a_noEep.elf") B without the EEPROM entry and $(scan_of "$OUT/a_all.elf") B with it). The symbols an entry adds by itself:"
  for drv in Eeprom TextDisplay; do
    read -r tot ram <<< "$(python3 symtool.py bytes "$OUT/a_all.elf" "$drv")"
    echo "        symbols naming $drv: $tot B (vtable, instance, poll; RAM object bytes counted in the table above)"
  done
  echo; echo "--- indirect calls, all on ---"; cat "$OUT/ic_all.txt"

  echo; echo "--- simavr (simulated atmega328p): every image, checksum against the same variant built natively ---"
  if command -v simavr >/dev/null && command -v avr-gdb >/dev/null; then
    for v in $ORDER; do
      g++ -std=c++17 -O1 -DR3B_PARITY ${VAR[$v]} $INC round3b.cpp -o "$OUT/par_$v"
      nat=$("$OUT/par_$v" | sed -n 's/^checksum 0x\(....\)$/\1/p'); rows=$("$OUT/par_$v" | sed -n 's/^rows //p')
      (timeout 30 simavr -g -m atmega328p -f 16000000 "$OUT/a_$v.elf" >/dev/null 2>&1 &); sleep 1
      avr=$(timeout 25 avr-gdb -batch -ex 'set architecture avr' -ex 'target remote :1234' -ex 'break done' -ex 'continue' \
            -ex 'x/1xh &g_sum' "$OUT/a_$v.elf" 2>/dev/null | sed -n 's/.*<g_sum>:[[:space:]]*0x\(....\)$/\1/p')
      pkill -x simavr || true
      if [ -n "$avr" ] && [ "${avr,,}" = "${nat,,}" ]; then ok "[$v] AVR checksum 0x$avr == native 0x$nat ($rows rows)"
      else bad "[$v] AVR checksum '${avr:-none}' != native '$nat'"; fi
      echo "$nat" > "$OUT/sum_$v"
    done
    # entries change the table, not just the code: the variants must not all agree
    if [ "$(cat "$OUT"/sum_{base,pin,clone,eep} | sort -u | wc -l)" = "4" ]; then ok "the pin, clone and EEPROM entries each change the table (four different checksums)"
    else bad "entries do not change the table"; fi
    if [ "$(cat "$OUT/sum_two")" = "$(cat "$OUT/sum_base")" ]; then ok "Use<Own,D> for the same drivers gives the same table as the bare list"; else bad "Use<Own,D> changes the table"; fi
    if [ "$(cat "$OUT/sum_all")" = "$(cat "$OUT/sum_noClaim")" ]; then ok "the claim on 0x72 (a device nothing listed would take) changes no row"; else bad "the claim changes the table"; fi
  else skip "simavr / avr-gdb not installed"; fi

  echo; echo "=== what is not listed is not linked (scenario 6) ==="
  # symbol names with the type-list arguments folded away (symtool.py): World and the identification fold carry the entry list in their names
  names() { python3 symtool.py names "$1"; }
  O0="avr-g++ -std=gnu++17 -O0 -mmcu=atmega328p -DF_CPU=16000000UL -fno-exceptions -fno-rtti -c $INC"
  echo "    linked -Os images: no symbol names the driver once its entry is gone."
  echo "    -O0 objects (every instantiated function is emitted, so inlining decisions cannot move a name): the symbols the entry adds all name its driver."
  $O0 $ALL round3b.cpp -o "$OUT/o_all.o" 2>/dev/null
  for pair in "Eeprom:noEep:Eeprom" "TextDisplay:noPin:TextDisplay|Banner|Shell"; do
    IFS=: read -r drv v pat <<< "$pair"
    left=$(names "$OUT/a_$v.elf" | grep -c "$drv"); in=$(names "$OUT/a_all.elf" | grep -c "$drv")
    [ "$left" = "0" ] && [ "$in" -ge 1 ] && ok "$drv: $in symbols in the image with its entry, none without it (image $((FL[all]-FL[$v])) B / $((RAM[all]-RAM[$v])) B smaller)" || bad "$drv: $in symbols with its entry, $left without it"
    $O0 ${VAR[$v]} round3b.cpp -o "$OUT/o_$v.o" 2>/dev/null
    python3 symtool.py diff "$OUT/o_all.o" "$OUT/o_$v.o" > "$OUT/d_$v"
    gone=$(grep -c '^- ' "$OUT/d_$v"); back=$(grep -c '^+ ' "$OUT/d_$v"); stray=$(grep '^- ' "$OUT/d_$v" | grep -Ev "$pat")
    [ -z "$stray" ] && [ "$back" = "0" ] && ok "$drv: the entry adds $gone symbols at -O0, every one naming $drv (for the display also the binding its found() instantiates), and removes none" || { bad "$drv: symbols the entry adds that do not name it, or that it removes:"; { echo "$stray"; grep '^+ ' "$OUT/d_$v"; } | head -6 | cut -c1-170; }
    grep '^- ' "$OUT/d_$v" | sed -E 's/hapi::APIOf<[^>]*>//g; s/^- /    + /' | cut -c1-120 | head -8
  done
  echo "--- a driver reached only by a pin carries no probe code (-O0: every instantiated function is emitted) ---"
  $O0 -DR3B_PIN round3b.cpp -o "$OUT/pin.o" 2>/dev/null; $O0 -DR3B_LCD_BARE round3b.cpp -o "$OUT/bare.o" 2>/dev/null
  pinp=$(avr-nm -C "$OUT/pin.o" | grep -c "TextDisplay.*::probe"); barep=$(avr-nm -C "$OUT/bare.o" | grep -c "TextDisplay.*::probe")
  [ "$pinp" = "0" ] && ok "pinned only: no TextDisplay probe symbol" || bad "pinned only: $pinp probe symbols"
  [ "$barep" -ge 1 ] && ok "control, the same driver listed bare: $barep probe symbol(s), so the check can see one" || bad "control found no probe symbol for the bare driver"
fi

echo; echo "=== seam ==="
if git rev-parse --is-inside-work-tree >/dev/null 2>&1 && git cat-file -e "$BASE" 2>/dev/null; then
  same="capability.h sensors.h display.h binding.h state.h stateful.h mockTwi.h mockDisplay.h mockStateful.h format.h mqttSink.h round1.cpp round2.cpp round3.cpp statePlacement.cpp ../rosCompose/topicChain.h ../../OneBus/mqtt/mqttWire.h ../../OneBus/mqtt/tcpTransport.h"
  # build_r1.sh/build_r2.sh/build_r3.sh dropped from this list at the C1 re-baseline (2026-09-28): they are test infrastructure, re-baselined
  # in step with this file whenever the reference commit moves, which is not the claim this seam makes (the round SOURCES, not the harness)
  if git diff --quiet "$BASE" -- $same; then ok "unchanged by R3b: $same"; else bad "changed by R3b:"; git diff --stat "$BASE" -- $same; fi
  echo "changed for R3b (git diff --stat $BASE):"
  git diff --stat "$BASE" -- driver.h registry.h build.sh | sed 's/^/  /'
  echo "new: identify.h mockAck.h round3b.cpp build_r3b.sh"
else skip "seam diff needs the IOP-RnD git history (commit $BASE)"; fi

echo; [ $rc = 0 ] && echo "ROUND 3b: ALL OK" || echo "ROUND 3b: FAILED"
exit $rc
