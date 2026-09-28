#!/usr/bin/env bash
# discoverCompose Round 1 verification.
#   native : g++ -O2 (+ASan/UBSan, + clang if present) — run the table/sample assertions.
#   AVR    : avr-g++ 7.3 -Os atmega328p, linked — size, symbol hygiene, indirect-call inventory.
# Exits non-zero if the native assertions fail or the AVR structure claims do not hold.
set -e
cd "$(dirname "$0")"
INC="-I ../../include -I ../../../HAPI/include -I ../../../OneBus/include"
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
rc=0

echo "=== native g++ $(g++ -dumpversion) -O2 ==="
g++ -std=c++17 -O2 -Wall -Wextra $INC round1.cpp -o "$OUT/r1"
"$OUT/r1"

echo; echo "=== native -O1 -fsanitize=address,undefined ==="
g++ -std=c++17 -O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=all $INC round1.cpp -o "$OUT/r1san"
"$OUT/r1san" | tail -1

if command -v clang++ >/dev/null; then
  echo; echo "=== native clang++ $(clang++ -dumpversion) -O2 ==="
  clang++ -std=c++17 -O2 -Wall -Wextra $INC round1.cpp -o "$OUT/r1clang"
  "$OUT/r1clang" | tail -1
fi

echo; echo "=== compile-fail guards (each must be rejected with its own message) ==="
for pair in "NEG_UNDECLARED_CAP:driver emits a capability it does not declare" "NEG_DUP_DRIVER:driver list has a repeated type" "NEG_DERIVED_DRIVER:driver must derive from DriverBase<itself,W>"; do
  def=${pair%%:*}; msg=${pair#*:}
  if g++ -std=c++17 -D$def $INC -fsyntax-only round1.cpp 2>&1 | grep -q "$msg"; then echo "OK: -D$def rejected: $msg"
  else echo "FAIL: -D$def was not rejected with '$msg'"; rc=1; fi
done

echo; echo "=== avr-g++ $(avr-g++ -dumpversion) -Os atmega328p (linked) ==="
avr-g++ -std=gnu++17 -Os -mmcu=atmega328p -DF_CPU=16000000UL -ffunction-sections -fdata-sections \
  -fno-exceptions -fno-rtti -Wall -Wextra $INC round1.cpp -Wl,--gc-sections -o "$OUT/r1.elf"
avr-size "$OUT/r1.elf"

if command -v simavr >/dev/null && command -v avr-gdb >/dev/null; then
  echo; echo "--- simavr (simulated atmega328p): run the image, compare checksum with native ---"
  nat=$("$OUT/r1" | sed -n 's/^checksum 0x\(....\)$/\1/p')
  (timeout 30 simavr -g -m atmega328p -f 16000000 "$OUT/r1.elf" >/dev/null 2>&1 &); sleep 1
  avr=$(timeout 25 avr-gdb -batch -ex 'set architecture avr' -ex 'target remote :1234' -ex 'break done' -ex 'continue' \
        -ex 'x/1xh &g_sum' "$OUT/r1.elf" 2>/dev/null | sed -n 's/.*<g_sum>:[[:space:]]*0x\(....\)$/\1/p')
  pkill -x simavr || true
  if [ -n "$avr" ] && [ "${avr,,}" = "${nat,,}" ]; then echo "OK: AVR checksum 0x$avr == native 0x$nat"
  else echo "FAIL: AVR checksum '${avr:-none}' != native '$nat'"; rc=1; fi
fi

echo; echo "--- symbol hygiene (want: none) ---"
if avr-nm -C "$OUT/r1.elf" | grep -E 'malloc|__divmod|__udivmod|__cxa_guard|_GLOBAL__sub_I|__do_global_ctors'; then
  echo "FAIL: heap / soft-division / guard / global-ctor symbols present"; rc=1
else echo "none"; fi

echo; echo "--- indirect calls per function (icall/eicall/ijmp) ---"
STRIP='function strip(s) { while (sub(/<[^<>]*>/, "", s)) ; return s }'
avr-objdump -dC --no-show-raw-insn "$OUT/r1.elf" | awk "$STRIP"'
  /^[0-9a-f]+ <.*>:$/ { name=$0; sub(/^[0-9a-f]+ </,"",name); sub(/>:$/,"",name); name=strip(name); next }
  /\t(icall|eicall|ijmp|eijmp)/ { n[name]++ }
  END { for (f in n) printf "%3d  %s\n", n[f], f }' > "$OUT/icalls"
cat "$OUT/icalls"
pump_n=$(awk '/World::pump\(\)/ {print $1}' "$OUT/icalls")
if [ "${pump_n:-0}" != "1" ]; then echo "FAIL: pump has ${pump_n:-0} indirect calls (want exactly 1)"; rc=1
else echo "OK: pump = 1 icall (the row's poll)"; fi
if [ "$(wc -l < "$OUT/icalls")" != "1" ]; then echo "FAIL: indirect calls outside pump"; rc=1
else echo "OK: pump is the only function in the whole image with an indirect call (poll/read/fan-out/route/scan: 0)"; fi

echo; echo "--- code size by part (bytes, text symbols) ---"
avr-nm -C -S -t d "$OUT/r1.elf" | awk "$STRIP"'
  $3 ~ /^[tTwW]$/ {
  sz=$2+0; name=strip($0)
  if      (name ~ /mock::/)                            k="mock bus (test scaffold)"
  else if (name ~ /oneBus::/)                          k="OneBus I2cMaster (real, reused)"
  else if (name ~ /DriverBase|Sensor[AB]::|Mux::/)     k="drivers (poll/readRegs/read/select)"
  else if (name ~ /discover::/)                        k="discovery: route/scan/claimed/pump"
  else if (name ~ /TempLogger|ValuePrinter/)           k="consumers (fan-out bodies, outlined)"
  else                                                 k="crt/main/checksum (scaffold)"
  t[k]+=sz }
  END { for (k in t) printf "%6d  %s\n", t[k], k }' | sort -rn

echo; echo "--- RAM by part (bytes, data+bss symbols) ---"
avr-nm -C -S -t d "$OUT/r1.elf" | awk "$STRIP"'
  $3 ~ /^[bBdDvVuU]$/ {
  sz=$2+0; name=strip($0)
  if      (name ~ /mock::/)                            k="mock bus (test scaffold)"
  else if (name ~ /::reg/)                             k="registry (8 rows x 5 B + count/overflow)"
  else if (name ~ /::fan/)                             k="capability fan-outs"
  else if (name ~ /vtable/)                            k="driver vtables"
  else if (name ~ /TempLogger|ValuePrinter/)           k="consumer log buffers (test scaffold)"
  else                                                 k="other (g_sum, misc)"
  t[k]+=sz }
  END { for (k in t) printf "%6d  %s\n", t[k], k }' | sort -rn

exit $rc
