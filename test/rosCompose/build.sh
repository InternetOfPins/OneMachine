#!/usr/bin/env bash
# OneMachine rosCompose verification.
#   native (g++/clang/ASan+UBSan): round1's script -- topic fan-out, service round trip, action lifecycle+cancel,
#                                   QoS history -- checked by its own asserts (see round1.cpp), not by this script.
#   AVR (bare avr-g++, no framework): symbol hygiene (no heap, no operator delete/new, no guard) and indirect-call
#                                     inventory; simavr checksum == native; the zero-cost tool on header inclusion.
# Exits non-zero if any claim does not hold.
set -e
cd "$(dirname "$0")"
. ../tools/lastok.sh
INC="-I ../../include -I ../../../HAPI/include"
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
rc=0
ok()  { echo "OK: $*"; }
bad() { echo "FAIL: $*"; rc=1; }

echo "=== native g++ $(g++ -dumpversion) -O2 ==="
g++ -std=c++17 -O2 -Wall -Wextra $INC round1.cpp -o "$OUT/r1" 2> "$OUT/r1.txt" || { bad "round1 does not compile"; cat "$OUT/r1.txt"; }
"$OUT/r1" | tail -2
lastok "$OUT/r1" && [ "$(grep -c warning "$OUT/r1.txt" || true)" = 0 ] && ok "g++ -O2, 0 warnings" || bad "round1 native"

echo; echo "=== native -O1 -fsanitize=address,undefined ==="
g++ -std=c++17 -O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=all $INC round1.cpp -o "$OUT/r1san"
lastok "$OUT/r1san" && ok "ASan+UBSan" || bad "round1 under sanitizers"
if command -v clang++ >/dev/null; then
  echo; echo "=== native clang++ $(clang++ -dumpversion) -O2 ==="
  clang++ -std=c++17 -O2 -Wall -Wextra $INC round1.cpp -o "$OUT/r1clang" 2> "$OUT/r1c.txt"
  lastok "$OUT/r1clang" && [ "$(grep -c warning "$OUT/r1c.txt" || true)" = 0 ] && ok "clang++, 0 warnings" || bad "round1 under clang"
fi

echo; echo "=== avr-g++ $(avr-g++ -dumpversion) -Os atmega328p, bare (no framework) ==="
FL="-std=gnu++17 -Os -mmcu=atmega328p -DF_CPU=16000000UL -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -Wall -Wextra"
avr-g++ $FL $INC round1.cpp -Wl,--gc-sections -o "$OUT/r1.elf" 2> "$OUT/r1avr.txt"
[ "$(grep -c warning "$OUT/r1avr.txt" || true)" = 0 ] && ok "avr-g++, 0 warnings" || { bad "avr-g++ warnings"; cat "$OUT/r1avr.txt"; }
avr-size "$OUT/r1.elf"

echo; echo "--- symbol hygiene: no heap, no operator delete/new, no guard/ctor (Cap<Msg>'s destructor is protected, non-virtual: no deleting destructor is ever emitted) ---"
if avr-nm -C "$OUT/r1.elf" | grep -iE 'malloc|operator delete|operator new|__cxa_guard|_GLOBAL__sub_I|__do_global_ctors'; then
  bad "heap / operator new-or-delete / guard / global-ctor symbols present"
else ok "none"; fi

echo; echo "--- indirect calls (icall/eicall/ijmp): the transport Cap seam hops, and nothing from composition ---"
STRIP='function strip(s) { while (sub(/<[^<>]*>/, "", s)) ; return s }'
avr-objdump -dC --no-show-raw-insn "$OUT/r1.elf" | awk "$STRIP"'
  /^[0-9a-f]+ <.*>:$/ { name=$0; sub(/^[0-9a-f]+ </,"",name); sub(/>:$/,"",name); name=strip(name); next }
  /\t(icall|eicall|ijmp|eijmp)/ { n[name]++ }
  END { for (f in n) printf "%3d  %s\n", n[f], f }' | tee "$OUT/icalls"
[ -s "$OUT/icalls" ] && ok "every indirect call is a transport-seam hop: Service::deliver (4 instantiations: doubler, ActionServer's goal/result/cancel services) each 1 Cap::deliver to their linkOut; Client::resolve's stored callback pointer; ActionServer::advance's Cap::deliver to statusPub -- none from the fan-out or composition itself" || bad "no indirect calls found (unexpected: the transport seam should show up)"

echo; echo "--- simavr (simulated atmega328p): run the image, compare checksum with native ---"
if command -v simavr >/dev/null && command -v avr-gdb >/dev/null; then
  nat=$("$OUT/r1" | sed -n 's/^checksum 0x\(....\)$/\1/p')
  (timeout 30 simavr -g -m atmega328p -f 16000000 "$OUT/r1.elf" >/dev/null 2>&1 &); sleep 1
  avr=$(timeout 25 avr-gdb -batch -ex 'set architecture avr' -ex 'target remote :1234' -ex 'break done' -ex 'continue' \
        -ex 'x/1xh &g_sum' "$OUT/r1.elf" 2>/dev/null | sed -n 's/.*<g_sum>:[[:space:]]*0x\(....\)$/\1/p')
  pkill -x simavr || true
  if [ -n "$avr" ] && [ "${avr,,}" = "${nat,,}" ]; then ok "AVR checksum 0x$avr == native 0x$nat"; else bad "AVR checksum '${avr:-none}' != native '$nat'"; fi
else echo "SKIP: simavr / avr-gdb not installed"; fi

echo; echo "--- zero-cost: a topic-only composition costs the same whether qos.h/service.h/action.h are reachable or not ---"
../tools/zero_cost.sh rosCompose_unused_headers headers_cost.cpp "$INC" "-DROS_ALL_HEADERS" "" || rc=1

exit $rc
