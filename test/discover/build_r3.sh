#!/usr/bin/env bash
# discoverCompose Round 3 verification (see HANDOFF.md, Round 3).
#   identity : the R1 and R2 flash images equal the images built from the R2 commit's own sources (cmp).
#   native   : g++ -O2, ASan+UBSan, clang; the removal / return / bridge / rediscovery scenarios.
#   guards   : broken variants must fail, an undeclared state must not compile.
#   AVR      : size matrix per state kind, symbol hygiene, indirect-call inventory, simavr checksum == native.
#   seam     : what changed in R1/R2 files.
# Exits non-zero if any claim fails. Overrides: HAPI=<dir> ONEBUS=<dir> (checkouts holding include/).
set -u
cd "$(dirname "$0")"
. ../tools/lastok.sh
HAPI=${HAPI:-../../../HAPI}; ONEBUS=${ONEBUS:-../../../OneBus}
HAPI_ABS=$(cd "$HAPI" && pwd); ONEBUS_ABS=$(cd "$ONEBUS" && pwd)
INC="-I ../../include -I $HAPI/include -I $ONEBUS/include"
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
rc=0
ok()   { echo "OK:   $*"; }
bad()  { echo "FAIL: $*"; rc=1; }
skip() { echo "SKIP: $*"; }
BASE=94d9be8       # the C1 registry/identify split (2026-09-28): the images legitimately moved then; the image-equality check is re-based here, not at the R2 commit
R2COMMIT=28bbeb4   # the R2 commit itself: the R2..R3 seam claim below is a fixed historical fact, untouched by C1's later, separate change
R3=a977079         # the R3 commit: the seam claim is about R2..R3; later work on shared files is checked by its own round

AVRF="-std=gnu++17 -Os -mmcu=atmega328p -DF_CPU=16000000UL -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti"
AVR="avr-g++ $AVRF -Wall -Wextra $INC"
flash_image() { avr-g++ $AVRF -I "$HAPI_ABS/include" -I "$ONEBUS_ABS/include" "$2" -Wl,--gc-sections -o "$1.elf" 2>/dev/null && avr-objcopy -O binary -j .text -j .data "$1.elf" "$1.bin"; }

echo "=== R1 and R2 images against their own recorded baselines (avr-g++ $(avr-g++ -dumpversion)) ==="
avr-g++ $AVRF $INC round1.cpp -Wl,--gc-sections -o "$OUT/new_round1.elf"
avr-g++ $AVRF $INC -DR2_NO_MQTT round2.cpp -Wl,--gc-sections -o "$OUT/new_round2.elf"
../tools/baseline.sh check r1_avr "$OUT/new_round1.elf" || rc=1
../tools/baseline.sh check r2_avr "$OUT/new_round2.elf" || rc=1
R1=$(avr-size "$OUT/new_round1.elf" | awk 'NR==2{print $1"/"$2"/"$3}'); R2=$(avr-size "$OUT/new_round2.elf" | awk 'NR==2{print $1"/"$2"/"$3}')
[ "$R1" = "2496/56/201" ] && ok "R1 image text/data/bss $R1" || bad "R1 image is $R1, was 2496/56/201"
[ "$R2" = "5118/152/661" ] && ok "R2 all-bindings image text/data/bss $R2" || bad "R2 image is $R2, was 5118/152/661"

echo; echo "=== native g++ $(g++ -dumpversion) -O2 ==="
if g++ -std=c++17 -O2 -Wall -Wextra $INC round3.cpp -o "$OUT/r3" && "$OUT/r3" > "$OUT/r3.out"; then
  grep -E "^(state:|checksum|OK:)" "$OUT/r3.out"; ok "round3 native scenarios"
else bad "round3 native"; grep -E "^FAIL" "$OUT/r3.out" | head; fi
echo; echo "=== native -O1 -fsanitize=address,undefined ==="
if g++ -std=c++17 -O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=all $INC round3.cpp -o "$OUT/r3san" && lastok "$OUT/r3san"; then ok "round3 under ASan+UBSan"; else bad "round3 under ASan+UBSan"; fi
if command -v clang++ >/dev/null; then
  echo; echo "=== native clang++ $(clang++ -dumpversion) -O2 ==="
  if clang++ -std=c++17 -O2 -Wall -Wextra $INC round3.cpp -o "$OUT/r3clang" && lastok "$OUT/r3clang"; then ok "round3 under clang"; else bad "round3 under clang"; fi
fi

echo; echo "=== broken variants must be caught (each a sed-patched copy of the real header(s), not a build-time switch) ==="
mutate() {  # name relheader sed-expr [relheader2 sed-expr2]
  rm -rf "$OUT/mut"; mkdir -p "$OUT/mut/oneMachine/discover"; cp ../../include/oneMachine/discover/*.h "$OUT/mut/oneMachine/discover/"
  sed -i "$3" "$OUT/mut/oneMachine/$2"
  if cmp -s "../../include/oneMachine/$2" "$OUT/mut/oneMachine/$2"; then bad "mutation '$1' did not apply ($2)"; return; fi
  if [ -n "${4:-}" ]; then
    sed -i "$5" "$OUT/mut/oneMachine/$4"
    if cmp -s "../../include/oneMachine/$4" "$OUT/mut/oneMachine/$4"; then bad "mutation '$1' did not apply ($4)"; return; fi
  fi
  g++ -std=c++17 -O1 -I "$OUT/mut" $INC round3.cpp -o "$OUT/mut/m" 2>/dev/null
  ( "$OUT/mut/m" > "$OUT/mut/m.out" 2>&1 ) 2>/dev/null; st=$?
  if [ $st = 0 ]; then bad "$1: the native test still passes"
  elif [ $st -ge 128 ]; then ok "$1: caught (the test crashes, signal $((st-128)): a released row is dereferenced)"
  else ok "$1: caught ($(grep -c '^FAIL' "$OUT/mut/m.out") checks fail, exit $st)"; fi
}
mutate "a released row's DeviceState/ClientState is not cleared (Gone leaves stale data behind)" discover/registry.h \
  '/Dev<>::Table::clear(m);/d'
mutate "setStatus only reaches direct children, not the whole subtree" discover/registry.h \
  's|if (under(m, r))|if (reg.rows[m].parent == r)|'
# Found while converting, not before: this file has exactly one ClientState-bearing consumer (Banner3 over SD), so no
# mutation of Shell::client()/OutBase::client() sharing is observable here -- round3.cpp's own scenario never actually
# exercised the cross-consumer collision the old NEG_SHARED_CLIENT switch claimed to guard against. The sharing code
# itself is gone from the header regardless (SharedCs no longer exists, in either file); a real regression test for
# it needs two consumers of the same ClientState type, which is new test-scenario work, not part of this conversion.
skip "SharedCs's cross-consumer collision has no observable case in this file (one ClientState-bearing consumer); the old switch was untested here even before conversion"
mutate "Gone does not release the consumers bound to the row" discover/registry.h \
  '/Self::release(m);/d'
mutate "pump() polls dead rows (a released row is still asked to read)" discover/registry.h \
  's|if (!reg.rows\[r\].isBus \&\& reg.status(r) == Status::Alive) {|if (!reg.rows[r].isBus) {|'
mutate "route() sends a bridge selection through a dead row" discover/registry.h \
  '/if constexpr (LifecycleOf<Self>::value) if (row.status() != Status::Alive) continue;/d'
mutate "bind() is never called (the R1 hook the app relies on)" discover/driver.h \
  '/if constexpr (HasBind<W, Impl>::value)/,+1d'
mutate "a released Shell binding does not check the row's status (a Stale row is still called)" discover/binding.h \
  '/if (W::reg.status(row) != Status::Alive) return nullptr;/d'
mutate "a released Shell binding does not check status, AND DeviceState indexing has no bounds guard (out-of-table access)" discover/binding.h \
  '/if (W::reg.status(row) != Status::Alive) return nullptr;/d' \
  discover/state.h 's|return rows\[r < N ? r : N\];|return rows[r];|'
echo "note: identity (NEG_NO_IDENTITY_CHECK, binding.h Shell::get()) is not repeated here on purpose: with the registry releasing every binding, no stale binding can exist in an app with lifecycle (round2's build_r2.sh already catches it)."
if g++ -std=c++17 -DNEG_UNDECLARED_STATE $INC -fsyntax-only round3.cpp 2>&1 | grep -q "driver declares no DeviceState"; then ok "-DNEG_UNDECLARED_STATE rejected: driver declares no DeviceState"
else bad "-DNEG_UNDECLARED_STATE was not rejected with its message"; fi
if g++ -std=c++17 -DR3_NO_LIFE -DNEG_STATUS_WITHOUT_LIFECYCLE $INC -fsyntax-only round3.cpp 2>&1 | grep -q "setStatus writes status"; then ok "-DNEG_STATUS_WITHOUT_LIFECYCLE rejected: an app that writes status must declare lifecycle"
else bad "-DNEG_STATUS_WITHOUT_LIFECYCLE was not rejected with its message"; fi
for pair in "NEG_STATUS_FIELD_WRITE:the row's status field" "NEG_STATUS_REGISTRY_WRITE:the registry's status writer"; do
  def=${pair%%:*}; what=${pair#*:}
  if g++ -std=c++17 -D$def $INC -fsyntax-only round3.cpp 2>&1 | grep -q "is private"; then ok "-D$def rejected: $what is private, World::setStatus is the only way in"
  else bad "-D$def was not rejected as private"; fi
done

echo; echo "=== DeviceState placement (sizes from the same types; the pool is derived, not built) ==="
g++ -std=c++17 -Wall -Wextra $INC statePlacement.cpp -o "$OUT/sp" && "$OUT/sp" || bad "statePlacement"

echo; echo "=== avr-g++ $(avr-g++ -dumpversion) atmega328p ==="
STRIP='function strip(s) { while (sub(/<[^<>]*>/, "", s)) ; return s }'
sizes() { avr-size "$1" | awk 'NR==2{print $1+$2, $2+$3}'; }
hygiene() { avr-nm -C "$1" | grep -E 'malloc|__divmod|__udivmod|__cxa_guard|_GLOBAL__sub_I|__do_global_ctors|__divmodhi|__mulhi|__mulsi|__divmodsi' ; }
icalls() {
  avr-objdump -dC --no-show-raw-insn "$1" | awk "$STRIP"'
    /^[0-9a-f]+ <.*>:$/ { name=$0; sub(/^[0-9a-f]+ </,"",name); sub(/>:$/,"",name); name=strip(name); next }
    /\t(icall|eicall|ijmp|eijmp)/ { n[name]++ }
    END { for (f in n) printf "%d %s\n", n[f], f }'
}
declare -A VAR=(
  [base]="-DR3_NO_DEV -DR3_NO_CLIENT -DR3_NO_LIFE"
  [dev]="-DR3_NO_CLIENT -DR3_NO_LIFE"
  [client]="-DR3_NO_DEV -DR3_NO_LIFE"
  [life]="-DR3_NO_DEV -DR3_NO_CLIENT"
  [all]=""
  [noDev]="-DR3_NO_DEV"
  [noClient]="-DR3_NO_CLIENT"
  [noLife]="-DR3_NO_LIFE"
)
declare -A FL RAM
avr_ok=1
for v in base dev client life all noDev noClient noLife; do
  if ! $AVR ${VAR[$v]} round3.cpp -Wl,--gc-sections -o "$OUT/a_$v.elf" 2> "$OUT/a_$v.err"; then bad "avr build [$v]"; head -5 "$OUT/a_$v.err"; avr_ok=0; continue; fi
  read -r FL[$v] RAM[$v] <<< "$(sizes "$OUT/a_$v.elf")"
  if hygiene "$OUT/a_$v.elf" > "$OUT/hyg_$v.txt" && [ -s "$OUT/hyg_$v.txt" ]; then bad "[$v] forbidden symbols: $(head -3 "$OUT/hyg_$v.txt" | tr '\n' ' ')"; fi
  icalls "$OUT/a_$v.elf" > "$OUT/ic_$v.txt"
  n=$(wc -l < "$OUT/ic_$v.txt"); pump=$(awk '/World<.*>::pump\(\)|World::pump\(\)/ {print $1}' "$OUT/ic_$v.txt")
  if [ "$n" != "1" ] || [ "${pump:-0}" != "1" ]; then bad "[$v] indirect calls: want exactly 1, in pump; got:"; cat "$OUT/ic_$v.txt"; fi
done
if [ "$avr_ok" = 1 ]; then
  ok "every variant links; no heap / divide / multiply / guard / global-ctor symbols; pump holds the only indirect call (1 icall) in every image"
  echo; echo "--- flash / RAM per state kind (B; flash = text+data, RAM = data+bss) ---"
  echo "    'alone' = over 'base' (stateful drivers and consumers, nothing declared, no lifecycle); 'marginal' = 'all' minus 'all without it'"
  printf '  %-9s %7s %7s\n' variant flash RAM
  for v in base dev client life all; do printf '  %-9s %7d %7d\n' "$v" "${FL[$v]}" "${RAM[$v]}"; done
  printf '\n  %-34s %8s %8s   %8s %8s\n' piece alone.fl alone.RAM marg.fl marg.RAM
  for pair in "DeviceState (slot per row + init):dev:noDev" "ClientState (cursor per consumer):client:noClient" "lifecycle (setStatus, skip dead, release):life:noLife"; do
    IFS=: read -r label a n <<< "$pair"
    printf '  %-34s %+8d %+8d   %+8d %+8d\n' "$label" "$((FL[$a]-FL[base]))" "$((RAM[$a]-RAM[base]))" "$((FL[all]-FL[$n]))" "$((RAM[all]-RAM[$n]))"
  done
  printf '  %-34s %+8d %+8d\n' "all three, over base" "$((FL[all]-FL[base]))" "$((RAM[all]-RAM[base]))"
  echo; echo "--- RAM by symbol, all on (bytes) ---"
  avr-nm -C -S -t d "$OUT/a_all.elf" | awk "$STRIP"'
    $3 ~ /^[bBdDvVuU]$/ {
      sz=$2+0; name=strip($0)
      if      (name ~ /Table::rows/)                 k="DeviceState table (slot x rows)"
      else if (name ~ /ClientCell/)                  k="direct consumer ClientState"
      else if (name ~ /Handle3::out/)                k="class handles (binding + ClientState slot)"
      else if (name ~ /Banner3::lcd/)                k="direct shell"
      else if (name ~ /mock|Display::|ScreenT::/)    k="mock bus + displays (test scaffold)"
      else if (name ~ /Log3/)                        k="sample log (test scaffold)"
      else if (name ~ /World.*::reg/)                k="registry"
      else                                           k="other"
      t[k]+=sz }
    END { for (k in t) printf "%6d  %s\n", t[k], k }' | sort -rn
  echo; echo "--- indirect calls, all on ---"; cat "$OUT/ic_all.txt"

  echo; echo "--- simavr (simulated atmega328p): run the image, compare checksum with the same build on native ---"
  if command -v simavr >/dev/null && command -v avr-gdb >/dev/null; then
    g++ -std=c++17 -O1 -DR3_PARITY $INC round3.cpp -o "$OUT/r3parity"
    nat=$("$OUT/r3parity" | sed -n 's/^checksum 0x\(....\)$/\1/p')
    (timeout 30 simavr -g -m atmega328p -f 16000000 "$OUT/a_all.elf" >/dev/null 2>&1 &); sleep 1
    avr=$(timeout 25 avr-gdb -batch -ex 'set architecture avr' -ex 'target remote :1234' -ex 'break done' -ex 'continue' \
          -ex 'x/1xh &g_sum' "$OUT/a_all.elf" 2>/dev/null | sed -n 's/.*<g_sum>:[[:space:]]*0x\(....\)$/\1/p')
    pkill -x simavr || true
    if [ -n "$avr" ] && [ "${avr,,}" = "${nat,,}" ]; then ok "AVR checksum 0x$avr == native 0x$nat (table + statuses + every DeviceState slot + both displays and logs + samples + counters + cursor)"
    else bad "AVR checksum '${avr:-none}' != native '$nat'"; fi
    nn=$("$OUT/r3" | sed -n 's/^checksum 0x\(....\)$/\1/p')
    [ "${nn,,}" = "${nat,,}" ] && ok "the test binary's own scenario() run gives the same checksum (0x$nn)" || bad "test-binary checksum 0x$nn != parity 0x$nat"
  else skip "simavr / avr-gdb not installed"; fi
fi

echo; echo "=== seam ==="
if git rev-parse --is-inside-work-tree >/dev/null 2>&1 && git cat-file -e "$R2COMMIT" 2>/dev/null && git cat-file -e "$R3" 2>/dev/null; then
  same="capability.h sensors.h display.h mockTwi.h mockDisplay.h format.h mqttSink.h round1.cpp build_r1.sh ../rosCompose/topicChain.h ../../OneBus/mqtt/mqttWire.h ../../OneBus/mqtt/tcpTransport.h"
  if git diff --quiet "$R2COMMIT" "$R3" -- $same; then ok "unchanged by R3 (R2 commit $R2COMMIT .. R3 commit $R3): $same"; else bad "changed by R3:"; git diff --stat "$R2COMMIT" "$R3" -- $same; fi
  echo "changed for R3 (git diff --stat $R2COMMIT $R3):"
  git diff --stat "$R2COMMIT" "$R3" -- driver.h registry.h binding.h round2.cpp build.sh build_r2.sh | sed 's/^/  /'
else skip "seam diff needs the IOP-RnD git history (commit $R2COMMIT)"; fi

echo; [ $rc = 0 ] && echo "ROUND 3: ALL OK" || echo "ROUND 3: FAILED"
exit $rc
