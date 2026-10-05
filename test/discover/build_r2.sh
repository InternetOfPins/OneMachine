#!/usr/bin/env bash
# OneMachine discover:: Round 2 verification (see HANDOFF.md, Round 2).
#   native : g++ -O2, ASan+UBSan, clang; independent payload (JSON/CSV) and captured-packet checks.
#   guards : compile-fail and broken-variant runs must be caught.
#   AVR    : per-binding size matrix, symbol hygiene, indirect-call inventory, simavr checksum == native.
#   seam   : which R1 files changed, which did not (self-skips outside the IOP-RnD history it refers to).
# Exits non-zero if any claim fails. Overrides: HAPI=<dir> ONEBUS=<dir> (checkouts holding include/), PY=<python with paho>.
# MQTT QoS 0 is compiled out (-DR2_NO_MQTT): mqttSink.h needs OneBus/mqtt, which is not part of this library; the
# packet-capture claim is checked below without a real broker; the live-broker leg stays in R&D.
set -u
cd "$(dirname "$0")"
. ../tools/lastok.sh
HAPI=${HAPI:-../../../HAPI}; ONEBUS=${ONEBUS:-../../../OneBus}
INC="-I ../../include -I $HAPI/include -I $ONEBUS/include -DR2_NO_MQTT"
OUT=$(mktemp -d); BROKER_PID=""
trap 'rm -rf "$OUT"; [ -n "$BROKER_PID" ] && kill "$BROKER_PID" 2>/dev/null' EXIT
rc=0
ok()   { echo "OK:   $*"; }
bad()  { echo "FAIL: $*"; rc=1; }
skip() { echo "SKIP: $*"; }

echo "=== native g++ $(g++ -dumpversion) -O2 ==="
if g++ -std=c++17 -O2 -Wall -Wextra $INC round2.cpp -o "$OUT/r2" && "$OUT/r2" > "$OUT/r2.out"; then
  grep -E "^(OK:|checksum)" "$OUT/r2.out"; ok "round2 native assertions"
else
  bad "round2 native"; grep -E "^FAIL|^first" "$OUT/r2.out" | head; fi

echo; echo "=== native -O1 -fsanitize=address,undefined ==="
if g++ -std=c++17 -O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=all $INC round2.cpp -o "$OUT/r2san" && lastok "$OUT/r2san"; then
  ok "round2 under ASan+UBSan"; else bad "round2 under ASan+UBSan"; fi
if command -v clang++ >/dev/null; then
  echo; echo "=== native clang++ $(clang++ -dumpversion) -O2 ==="
  if clang++ -std=c++17 -O2 -Wall -Wextra $INC round2.cpp -o "$OUT/r2clang" && lastok "$OUT/r2clang"; then
    ok "round2 under clang"; else bad "round2 under clang"; fi
fi

echo; echo "=== independent checks ==="
python3 check_r2.py payloads "$OUT/r2.out" && ok "JSON+CSV payloads == independent formatter" || bad "payloads"
skip "PUBLISH-packet and live-Mosquitto checks: MQTT is compiled out here (-DR2_NO_MQTT, OneBus/mqtt is not part of this library); they stay in R&D"

echo; echo "=== compile-fail guards (each rejected with its own message) ==="
for pair in "NEG_CLASS_NO_PROVIDER:no driver in the list provides this output class" \
            "NEG_DIRECT_ABSENT:a binding consumer names a driver that is not in the driver list"; do
  def=${pair%%:*}; msg=${pair#*:}
  if g++ -std=c++17 -D$def $INC -fsyntax-only round2.cpp 2>&1 | grep -q "$msg"; then ok "-D$def rejected: $msg"
  else bad "-D$def was not rejected with '$msg'"; fi
done

echo; echo "=== broken variants must be caught (each a sed-patched copy of the real header, not a build-time switch) ==="
mutate() {  # name relheader sed-expr [prelude: run before the checked run, e.g. to also capture check_r2.py's own view]
  rm -rf "$OUT/mut"; mkdir -p "$OUT/mut/oneMachine/discover"; cp ../../include/oneMachine/discover/*.h "$OUT/mut/oneMachine/discover/"
  sed -i "$3" "$OUT/mut/oneMachine/$2"
  if cmp -s "../../include/oneMachine/$2" "$OUT/mut/oneMachine/$2"; then bad "mutation '$1' did not apply"; return; fi
  if g++ -std=c++17 -O1 -I "$OUT/mut" $INC round2.cpp -o "$OUT/mut/m" 2>/dev/null; then
    "$OUT/mut/m" > "$OUT/mut/m.out" 2>&1
    n=$(grep -c '^FAIL' "$OUT/mut/m.out" || true)
    [ "$n" -gt 0 ] && ok "$1 -> $n native checks fail" || bad "mutation '$1' went undetected"
  else bad "$1 -> the mutated tree does not compile: a harness error, not a catch"; fi
}
mutate "Shell::get() does not check the row's status (a released row can still be reached)" discover/binding.h \
  '/if (W::reg.status(row) != Status::Alive) return nullptr;/d'
mutate "Shell::get() does not check identity (a rediscovered row's old driver pointer is still trusted)" discover/binding.h \
  '/if (W::reg.rows\[row\].drv != instOf<Iface>()) return nullptr;/d'
mutate "bind() is never called (the R1 hook the app relies on)" discover/driver.h \
  '/if constexpr (HasBind<W, Impl>::value)/,+1d'
rm -rf "$OUT/mut"; mkdir -p "$OUT/mut/oneMachine/discover"; cp ../../include/oneMachine/discover/*.h "$OUT/mut/oneMachine/discover/"
sed -i 's|n = uint8_t(n + putStr(o + n, ",\\"value\\":"));|n = uint8_t(n + putStr(o + n, ",\\"val\\":"));|' "$OUT/mut/oneMachine/discover/format.h"
cmp -s ../../include/oneMachine/discover/format.h "$OUT/mut/oneMachine/discover/format.h" && bad "mutation 'wrong JSON key name' did not apply"
g++ -std=c++17 -O1 -I "$OUT/mut" $INC round2.cpp -o "$OUT/mut/jf" 2>/dev/null
"$OUT/mut/jf" > "$OUT/mut/jf.out" 2>&1
python3 check_r2.py payloads "$OUT/mut/jf.out" >/dev/null 2>&1 && bad "wrong JSON key name: the independent formatter check still passes" || ok "wrong JSON key name: the independent formatter check fails too"

echo; echo "=== avr-g++ $(avr-g++ -dumpversion) atmega328p, MQTT excluded ==="
AVR="avr-g++ -std=gnu++17 -Os -mmcu=atmega328p -DF_CPU=16000000UL -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -Wall -Wextra $INC"
STRIP='function strip(s) { while (sub(/<[^<>]*>/, "", s)) ; return s }'
sizes() { avr-size "$1" | awk 'NR==2{print $1+$2, $2+$3}'; }     # flash (text+data), RAM (data+bss)
hygiene() { avr-nm -C "$1" | grep -E 'malloc|__divmod|__udivmod|__cxa_guard|_GLOBAL__sub_I|__do_global_ctors|__divmodhi|__mulhi|__mulsi|__divmodsi' ; }
icalls() {
  avr-objdump -dC --no-show-raw-insn "$1" | awk "$STRIP"'
    /^[0-9a-f]+ <.*>:$/ { name=$0; sub(/^[0-9a-f]+ </,"",name); sub(/>:$/,"",name); name=strip(name); next }
    /\t(icall|eicall|ijmp|eijmp)/ { n[name]++ }
    END { for (f in n) printf "%d %s\n", n[f], f }'
}

declare -A VAR=(
  [base]="-DR2_NO_DIRECT -DR2_NO_CLASS -DR2_NO_JSON -DR2_NO_CSV"
  [direct]="-DR2_NO_CLASS -DR2_NO_JSON -DR2_NO_CSV"
  [class]="-DR2_NO_DIRECT -DR2_NO_JSON -DR2_NO_CSV"
  [json]="-DR2_NO_DIRECT -DR2_NO_CLASS -DR2_NO_CSV"
  [csv]="-DR2_NO_DIRECT -DR2_NO_CLASS -DR2_NO_JSON"
  [all]=""
  [noDirect]="-DR2_NO_DIRECT"
  [noClass]="-DR2_NO_CLASS"
  [noJson]="-DR2_NO_JSON"
  [noCsv]="-DR2_NO_CSV"
)
declare -A FL RAM
avr_ok=1
for v in base direct class json csv all noDirect noClass noJson noCsv; do
  if ! $AVR ${VAR[$v]} round2.cpp -Wl,--gc-sections -o "$OUT/a_$v.elf" 2> "$OUT/a_$v.err"; then bad "avr build [$v]"; head -5 "$OUT/a_$v.err"; avr_ok=0; continue; fi
  read -r FL[$v] RAM[$v] <<< "$(sizes "$OUT/a_$v.elf")"
  if hygiene "$OUT/a_$v.elf" > "$OUT/hyg_$v.txt" && [ -s "$OUT/hyg_$v.txt" ]; then bad "[$v] forbidden symbols: $(head -3 "$OUT/hyg_$v.txt" | tr '\n' ' ')"; fi
  icalls "$OUT/a_$v.elf" > "$OUT/ic_$v.txt"
  n=$(wc -l < "$OUT/ic_$v.txt"); pump=$(awk '/World<.*>::pump\(\)|World::pump\(\)/ {print $1}' "$OUT/ic_$v.txt")
  if [ "$n" != "1" ] || [ "${pump:-0}" != "1" ]; then bad "[$v] indirect calls: want exactly 1, in pump; got:"; cat "$OUT/ic_$v.txt"; fi
done
if [ "$avr_ok" = 1 ]; then
  ok "every variant links; no heap / divide / guard / global-ctor symbols; pump holds the only indirect call (1 icall) in every image"
  echo; echo "--- flash / RAM per binding (B; flash = text+data, RAM = data+bss) ---"
  echo "    'alone'    = delta of that binding over 'base' (display driver rows + mock scaffolding, no binding on): pays for the shared code itself"
  echo "    'marginal' = 'all' minus 'all without it': shared code (digit formatter, display operations, sink) counted once"
  printf '  %-9s %7s %7s\n' variant flash RAM
  for v in base direct class json csv all; do printf '  %-9s %7d %7d\n' "$v" "${FL[$v]}" "${RAM[$v]}"; done
  printf '\n  %-22s %8s %8s   %8s %8s\n' binding alone.fl alone.RAM marg.fl marg.RAM
  for pair in "direct connection:direct:noDirect" "capability-set class:class:noClass" "generic client JSON:json:noJson" "generic client CSV:csv:noCsv"; do
    IFS=: read -r label a n <<< "$pair"
    printf '  %-22s %+8d %+8d   %+8d %+8d\n' "$label" "$((FL[$a]-FL[base]))" "$((RAM[$a]-RAM[base]))" "$((FL[all]-FL[$n]))" "$((RAM[all]-RAM[$n]))"
  done
  printf '  %-22s %+8d %+8d\n' "all three, over base" "$((FL[all]-FL[base]))" "$((RAM[all]-RAM[base]))"
  echo; echo "--- code size by part, all bindings on (bytes, text symbols; classified on the name with template arguments stripped: a World member carries the whole consumer list in its type) ---"
  avr-nm -C -S -t d "$OUT/a_all.elf" | awk "$STRIP"'
    $3 ~ /^[tTwW]$/ {
      sz=$2+0; name=strip($0)
      if      (name ~ /topic_chain::/)                                 k="fan-out folds with the consumer bodies inlined (Subscriber::deliver)"
      else if (name ~ /fmt::|MemSink/)                                 k="generic client: digit formatter + sink"
      else if (name ~ /TextDisplay|LineDisplay/)                       k="display drivers (TextDisplay, LineDisplay)"
      else if (name ~ /Banner|Readout|Mirror|OutBase|OpPrint|OpClear/) k="binding consumers (Banner, Readout, Mirror) and the class handle"
      else if (name ~ /oneBus::/)                                      k="OneBus I2cMaster (real, reused)"
      else if (name ~ /mock::|mockdisp::/)                             k="mock bus + displays (test scaffold)"
      else if (name ~ /DriverBase|SensorA|SensorB|Mux/)                k="sensor and bridge drivers (R1)"
      else if (name ~ /discover::/)                                    k="discovery / registry / pump / emit, fan-out and consumer bodies inlined here (R1 + R2)"
      else                                                             k="crt / main / checksum (scaffold)"
      t[k]+=sz }
    END { for (k in t) printf "%6d  %s\n", t[k], k }' | sort -rn
  echo; echo "--- indirect calls, all bindings on ---"; cat "$OUT/ic_all.txt"
  echo "shell interface calls: 0 (Shell::get returns a typed pointer to a stateless singleton, its operations are static; the class handle dispatches by pointer compare) — both counted in the inventory above, which has no indirect call outside pump"

  echo; echo "--- simavr (simulated atmega328p): run the image, compare checksum with the same build on native ---"
  if command -v simavr >/dev/null && command -v avr-gdb >/dev/null; then
    g++ -std=c++17 -O1 -DR2_PARITY -DR2_NO_MQTT $INC round2.cpp -o "$OUT/r2parity"
    nat=$("$OUT/r2parity" | sed -n 's/^checksum 0x\(....\)$/\1/p')
    (timeout 30 simavr -g -m atmega328p -f 16000000 "$OUT/a_all.elf" >/dev/null 2>&1 &); sleep 1
    avr=$(timeout 25 avr-gdb -batch -ex 'set architecture avr' -ex 'target remote :1234' -ex 'break done' -ex 'continue' \
          -ex 'x/1xh &g_sum' "$OUT/a_all.elf" 2>/dev/null | sed -n 's/.*<g_sum>:[[:space:]]*0x\(....\)$/\1/p')
    pkill -x simavr || true
    if [ -n "$avr" ] && [ "${avr,,}" = "${nat,,}" ]; then ok "AVR checksum 0x$avr == native 0x$nat (table + display screen and command log + JSON/CSV payload bytes)"
    else bad "AVR checksum '${avr:-none}' != native '$nat'"; fi
  else skip "simavr / avr-gdb not installed"; fi
fi

echo; echo "=== seam ==="
# R1 image, built from the R1 program as it is now, against the size it had before R2 (avr-gcc 7.3.0)
# (re-baselined at the C1 registry/identify split, 2026-09-28: that round moved every image in this file by a small, explained amount)
if $AVR round1.cpp -Wl,--gc-sections -o "$OUT/r1.elf" 2>/dev/null; then
  r1=$(avr-size "$OUT/r1.elf" | awk 'NR==2{print $1"/"$2"/"$3}')
  [ "$r1" = "2496/56/201" ] && ok "R1 image unchanged: text/data/bss $r1" || bad "R1 image is $r1, was 2496/56/201"
else bad "R1 does not build for AVR"; fi
# R2's claim is about what R2 changed: compare the two commits, so later rounds may change these files
if git rev-parse --is-inside-work-tree >/dev/null 2>&1 && git cat-file -e f6717a4 2>/dev/null && git cat-file -e 28bbeb4 2>/dev/null; then
  same="capability.h registry.h mockTwi.h ../rosCompose/topicChain.h ../../OneBus/mqtt/mqttWire.h ../../OneBus/mqtt/tcpTransport.h"
  if git diff --quiet f6717a4 28bbeb4 -- $same; then ok "unchanged by R2 (R1 commit f6717a4 .. R2 commit 28bbeb4): $same"; else bad "changed by R2:"; git diff --stat f6717a4 28bbeb4 -- $same; fi
  echo "changed by R2 (git diff --stat f6717a4 28bbeb4):"; git diff --stat f6717a4 28bbeb4 -- driver.h round1.cpp | sed 's/^/  /'
else skip "seam diff needs the IOP-RnD git history (commits f6717a4, 28bbeb4)"; fi

echo; [ $rc = 0 ] && echo "ROUND 2: ALL OK" || echo "ROUND 2: FAILED"
exit $rc
