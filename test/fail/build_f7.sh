#!/usr/bin/env bash
# failCompose F7 verification: a health monitor (health.h) composed over discoverCompose's registry and F5's edges.
#   native (g++/clang/ASan+UBSan, mock time): the scenarios (default, F7_REQUIRED, F7_NO_ISOLATE builds); mutations
#   AVR: cost with and without Health composed, indirect calls, forbidden symbols, simavr checksum parity
set -e
cd "$(dirname "$0")"
INC="-I ../../include -I ../../../HAPI/include -I ../../../OneBus/include"
INCABS="-I $(cd ../../include && pwd) -I $(cd ../../../HAPI/include && pwd) -I $(cd ../../../OneBus/include && pwd)"
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
rc=0
ok()  { echo "OK: $*"; }
bad() { echo "FAIL: $*"; rc=1; }

echo "=== F7.1 native: the three builds (default, F7_REQUIRED, F7_NO_ISOLATE) ==="
declare -A FL=( [default]="" [required]="-DF7_REQUIRED" [no_isolate]="-DF7_NO_ISOLATE" )
for v in default required no_isolate; do
  g++ -std=c++17 -O2 -Wall -Wextra ${FL[$v]} $INC roundF7.cpp -o "$OUT/f7_$v" 2> "$OUT/cc_$v.txt" || { bad "$v does not compile"; cat "$OUT/cc_$v.txt"; continue; }
  w=$(grep -c warning "$OUT/cc_$v.txt" || true)
  "$OUT/f7_$v" > "$OUT/r_$v.txt" || true
  if tail -1 "$OUT/r_$v.txt" | grep -q '^OK' && [ "$w" = 0 ]; then ok "$v, 0 warnings: $(tail -2 "$OUT/r_$v.txt" | head -1)"; else bad "$v (warnings=$w)"; grep '^FAIL' "$OUT/r_$v.txt" | head -5; fi
done

echo; echo "=== F7.2 sanitizers and clang, on the default build ==="
g++ -std=c++17 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all $INC roundF7.cpp -o "$OUT/f7san"
"$OUT/f7san" | tail -1 | grep -q '^OK' && ok "ASan+UBSan" || bad "under sanitizers"
if command -v clang++ >/dev/null; then
  clang++ -std=c++17 -O2 -Wall -Wextra $INC roundF7.cpp -o "$OUT/f7clang" 2> "$OUT/cc_clang.txt"
  w=$(grep -c warning "$OUT/cc_clang.txt" || true)
  "$OUT/f7clang" | tail -1 | grep -q '^OK' && [ "$w" = 0 ] && ok "clang++ $(clang++ -dumpversion), 0 warnings" || bad "under clang (warnings=$w)"
fi

echo; echo "=== F7.3 mutations: each must make the scenarios fail by their checks ==="
mutate7() {  # name flags [expect-build]
  local name="$1" flags="$2" build="${3:-default}"
  if ! g++ -std=c++17 -O1 ${FL[$build]} $flags $INC roundF7.cpp -o "$OUT/m7" 2> "$OUT/m7.err"; then bad "$name does not compile"; head -5 "$OUT/m7.err"; return; fi
  "$OUT/m7" > "$OUT/m7.out" 2>&1 || true
  n=$(grep -c '^FAIL' "$OUT/m7.out" || true)
  [ "$n" -ge 1 ] && ok "$name -> $n checks fail" || bad "mutation '$name' went undetected"
}
mutate7 "the monitor never fires (no flap counted)"                       "-DF7_NEG_NO_FLAP"
mutate7 "a bus fault is counted as its devices' own flaps"                "-DF7_NEG_BUS_AS_DEVICE_FLAP"
mutate7 "a required row is quarantined anyway"                            "-DF7_NEG_IGNORE_REQUIRED" required
mutate7 "no hysteresis: probation exits at the same level it entered"     "-DF7_NEG_NO_HYSTERESIS"
mutate7 "a quarantine that never ends"                                    "-DF7_NEG_QUARANTINE_FOREVER"
mutate7 "bus cost attributed to the wrong row"                            "-DF7_NEG_WRONG_ROW_COST"

echo; echo "=== F7.4 AVR (avr-g++ $(avr-g++ -dumpversion), -Os, atmega328p, linked): cost, indirect calls, forbidden symbols ==="
TB_PREFIX=""
command -v avr-g++ >/dev/null || TB_PREFIX=~/.platformio/packages/toolchain-atmelavr/bin/
FL7="-std=gnu++17 -Os -mmcu=atmega328p -DF_CPU=16000000UL -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -Wall -Wextra"
for v in full nohealth; do
  extra=""; [ "$v" = nohealth ] && extra="-DF7_NO_HEALTH"
  "${TB_PREFIX}avr-g++" $FL7 $extra $INC roundF7.cpp -Wl,--gc-sections -o "$OUT/$v.elf" 2> "$OUT/av_$v.txt" || { bad "AVR $v does not compile"; cat "$OUT/av_$v.txt"; continue; }
  w=$(grep -c warning "$OUT/av_$v.txt" || true)
  [ "$w" = 0 ] || bad "AVR $v: $w warnings"
done
read -r tf df bf < <("${TB_PREFIX}avr-size" "$OUT/full.elf" | tail -1 | awk '{print $1, $2, $3}')
read -r tn dn bn < <("${TB_PREFIX}avr-size" "$OUT/nohealth.elf" | tail -1 | awk '{print $1, $2, $3}')
ok "flash (text+data): full $((tf+df)) B, without Health $((tn+dn)) B, the monitor costs $((tf+df-tn-dn)) B"
ok "RAM (data+bss): full $((df+bf)) B, without Health $((dn+bn)) B, the monitor costs $((df+bf-dn-bn)) B"
ic=$("${TB_PREFIX}avr-objdump" -dC --no-show-raw-insn "$OUT/full.elf" | awk '/\t(icall|eicall|ijmp|eijmp)/ { ++n } END { print n + 0 }')
[ "$ic" = 1 ] && ok "one indirect call in the whole image (pump()'s poll)" || bad "indirect calls: $ic (expected 1)"
fs=$("${TB_PREFIX}avr-nm" -C "$OUT/full.elf" | grep -cE 'malloc|__cxa_guard' || true)
[ "$fs" = 0 ] && ok "no malloc/__cxa_guard symbols" || bad "forbidden symbols: $fs"

echo; echo "=== F7.5 simavr parity: the simulated ATmega328's scripted-flap checksum == native's (C1 item B.7, closes the F5-shaped gap) ==="
avr_sum() {   # elf -> checksum stored in g_sum by the simulated ATmega328
  (timeout 60 simavr -g -m atmega328p -f 16000000 "$1" >/dev/null 2>&1 &); sleep 1
  timeout 55 avr-gdb -batch -ex 'set architecture avr' -ex 'target remote :1234' -ex 'break done' -ex 'continue' -ex 'x/1xh &g_sum' "$1" 2>/dev/null \
    | sed -n 's/.*<g_sum>:[[:space:]]*0x\(....\)$/\1/p'
  pkill -x simavr || true
}
if command -v simavr >/dev/null && command -v avr-gdb >/dev/null; then
  g++ -std=c++17 -O2 -DF7_PARITY $INC roundF7.cpp -o "$OUT/f7par" 2>/dev/null
  nat=$("$OUT/f7par" | sed -n 's/^checksum 0x\(....\)$/\1/p')
  avr=$(avr_sum "$OUT/full.elf")
  [ -n "$avr" ] && [ "${avr,,}" = "${nat,,}" ] && ok "simulated ATmega328 checksum 0x$avr == native 0x$nat (the scripted flap: 8 power-loss cycles, mock time)" || bad "simavr checksum '${avr:-none}' != native 0x$nat"
else
  echo "skip: simavr / avr-gdb not installed"
fi

[ $rc = 0 ] && echo "failCompose F7: OK" || echo "failCompose F7: FAILED"
exit $rc
