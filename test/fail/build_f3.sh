#!/usr/bin/env bash
# OneMachine fail:: F3 verification: the delivery edge, Store policies and failure layers per consumer class.
#   unit (mock time)  the four classes with mock consumers; the compile-time rules of each class; mutations
#   AVR               a consumer declared with no layer is the hand-written consumer, byte for byte; R2's image with the headers included is unchanged;
#                     cost per class (alone / marginal); one icall; simavr == native
# The logic/real-broker legs (four consumers through discover::'s fan-out, one being the MQTT edge, against a real
# Mosquitto) stay in R&D: they need OneBus/mqtt, which is not part of this library.
set -e
cd "$(dirname "$0")"
INC="-I ../../include -I ../../../HAPI/include -I ../../../OneBus/include"
INCABS="-I $(cd ../../include && pwd) -I $(cd ../../../HAPI/include && pwd) -I $(cd ../../../OneBus/include && pwd)"
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
rc=0
ok()   { echo "OK: $*"; }
bad()  { echo "FAIL: $*"; rc=1; }
skip() { echo "SKIP: $*"; }
R2_COMMIT=94d9be8   # re-baselined at the C1 registry/identify split (2026-09-28), not the R2 commit

echo "=== F3.1 unit level: the classes, the report on _deliver(), isolation (mock time) ==="
g++ -std=c++17 -O2 -Wall -Wextra $INC unit_f3.cpp -o "$OUT/u3" 2> "$OUT/u3.txt" || { bad "unit_f3 does not compile"; cat "$OUT/u3.txt"; }
"$OUT/u3" | tail -3
"$OUT/u3" | tail -1 | grep -q '^OK' && [ "$(grep -c warning "$OUT/u3.txt" || true)" = 0 ] && ok "g++ -O2, 0 warnings: display (latest wins, the replaced counted and reported once), storage (N records in order, the newest refused when full, a pulled card retried on a back-off and remounted, nothing lost), fire-and-forget (never retried), direct (status only), no layer at all (nothing kept), no sink time inside offer()" || bad "unit_f3"
g++ -std=c++17 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all $INC unit_f3.cpp -o "$OUT/u3s" && "$OUT/u3s" | tail -1 | grep -q '^OK' && ok "ASan+UBSan" || bad "unit_f3 under sanitizers"
if command -v clang++ >/dev/null; then
  clang++ -std=c++17 -O2 -Wall -Wextra $INC unit_f3.cpp -o "$OUT/u3c" && "$OUT/u3c" | tail -1 | grep -q '^OK' && ok "clang++ -O2" || bad "unit_f3 under clang"
fi

echo; echo "=== F3.2 compile-fail: each class rejects what it does not accept, with its own message ==="
for pair in "NEG_DISPLAY_RETRY:a display keeps only the latest record: it does not accept Retry" \
            "NEG_DISPLAY_BUFFER:a display has one slot (Latest): Buffer(N) is for storage and fire-and-forget consumers" \
            "NEG_STORAGE_OVERWRITE:a storage consumer never overwrites a record it has not written" \
            "NEG_FF_RETRY:a fire-and-forget consumer is never retried" \
            "NEG_DIRECT_STORE:a direct shell takes the sample in on() and has no store: status only" \
            "NEG_DIRECT_RETRY:a direct shell is never retried" \
            "NEG_STORE_NO_STATUS:Buffer counts what it refuses into Status: place Status below it"; do
  def=${pair%%:*}; msg=${pair#*:}
  if g++ -std=c++17 -D$def $INC -fsyntax-only unit_f3.cpp 2>&1 | grep -qF "static assertion failed: $msg"; then ok "-D$def rejected: $msg"; else bad "-D$def was not rejected with '$msg'"; fi
done

echo; echo "=== F3.3 mutation checks (unit level): each broken behaviour must make the tests fail (a tree that does not compile is a failure) ==="
for pair in "F3_NEG_BUFFER_OVERWRITE:a Store on the SD consumer overwrites the newest waiting record" \
            "F3_NEG_UNCOUNTED_DROP:a refused record is not counted" \
            "F3_NEG_BLOCKING_FANOUT:a slow consumer blocks the fan-out (the sink is called from offer)" \
            "F3_NEG_LATCHED_READY:a pulled card still reads ready" \
            "F3_NEG_NO_POISON_CHECK:a poison record is retried like any other failure, never dropped"; do
  def=${pair%%:*}; what=${pair#*:}
  if ! g++ -std=c++17 -O1 -D$def $INC unit_f3.cpp -o "$OUT/neg" 2> "$OUT/neg.err"; then bad "-D$def does not compile: $(head -2 "$OUT/neg.err" | tr '\n' ' ')"; continue; fi
  "$OUT/neg" > "$OUT/neg.out" 2>&1 || true
  n=$(grep -c '^FAIL' "$OUT/neg.out"); [ "$n" -gt 0 ] && ok "-D$def ($what) -> $n checks fail" || bad "-D$def ($what): no check fails"
done

echo; echo "=== F3.4 mutations on the library header itself (a copy, patched; each must make the tests fail) ==="
mutateL() {  # name file sed-expression
  rm -rf "$OUT/mutl"; mkdir -p "$OUT/mutl/oneMachine/fail"; cp ../../include/oneMachine/fail/*.h "$OUT/mutl/oneMachine/fail/"
  sed -i "$3" "$OUT/mutl/oneMachine/fail/$2"
  if cmp -s "../../include/oneMachine/fail/$2" "$OUT/mutl/oneMachine/fail/$2"; then bad "mutation '$1' did not apply"; return; fi
  if g++ -std=c++17 -O2 -I "$OUT/mutl" $INCABS unit_f3.cpp -o "$OUT/mutl/m" 2> "$OUT/mutl/mut.err"; then
    n=$("$OUT/mutl/m" | grep -c '^FAIL' || true)
    [ "$n" -gt 0 ] && ok "$1 -> $n checks fail" || bad "mutation '$1' went undetected"
  else bad "mutation '$1' does not compile (the harness, not the check): $(head -3 "$OUT/mutl/mut.err" | tr '\n' ' ')"; fi
}
mutateL "a record that failed for good is not reported" delivery.h 's|          ctl().pop(); ++failed; last = o; haveLast = true; headFailed = false;|          ctl().pop(); ++failed;|'
mutateL "a record that failed for good is not counted"  delivery.h 's|          ctl().pop(); ++failed; last = o; haveLast = true; headFailed = false;|          ctl().pop(); last = o; haveLast = true; headFailed = false;|'

echo; echo "=== F3.5 AVR (avr-g++ $(avr-g++ -dumpversion), -Os, atmega328p) ==="
FLB="-std=gnu++17 -Os -mmcu=atmega328p -DF_CPU=16000000UL -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti"
FL="$FLB -Wall -Wextra"
STRIP='function strip(s) { while (sub(/<[^<>]*>/, "", s)) ; return s }'
avr_sum() {
  (timeout 90 simavr -g -m atmega328p -f 16000000 "$1" >/dev/null 2>&1 &); sleep 1
  timeout 85 avr-gdb -batch -ex 'set architecture avr' -ex 'target remote :1234' -ex 'break done' -ex 'continue' -ex 'x/1xh &g_sum' "$1" 2>/dev/null \
    | sed -n 's/.*<g_sum>:[[:space:]]*0x\(....\)$/\1/p'
  pkill -x simavr || true
}
have_sim=0; command -v simavr >/dev/null && command -v avr-gdb >/dev/null && have_sim=1

echo "--- a fan-out with consumers and no delivery layer: discover::'s R2 image, with the delivery headers included, against its own IOP-RnD history ($R2_COMMIT)"
if git rev-parse --is-inside-work-tree >/dev/null 2>&1 && git cat-file -e "$R2_COMMIT" 2>/dev/null; then
  mkdir -p "$OUT/r2src"; (cd "$(git rev-parse --show-toplevel)" && git archive $R2_COMMIT HAPI/discoverCompose HAPI/rosCompose OneBus/mqtt | tar -x -C "$OUT/r2src")
  (cd "$OUT/r2src/HAPI/discoverCompose" && avr-g++ $FLB $INCABS round2.cpp -Wl,--gc-sections -o "$OUT/r2_old.elf")
  avr-g++ $FLB $INCABS -include "../../include/oneMachine/fail/delivery.h" -include "../support/mockdev.h" ../discover/round2.cpp -Wl,--gc-sections -o "$OUT/r2_new.elf"
  for e in r2_old r2_new; do avr-objcopy -O binary -j .text -j .data "$OUT/$e.elf" "$OUT/$e.bin"; done
  if cmp -s "$OUT/r2_old.bin" "$OUT/r2_new.bin"; then ok "R2's all-bindings image is byte-identical with delivery.h and mockdev.h included ($(avr-size "$OUT/r2_new.elf" | tail -1 | awk '{print $1"/"$2"/"$3}') text/data/bss, $(stat -c %s "$OUT/r2_new.bin") B)"; else bad "R2's image differs with the delivery headers included"; fi
else skip "byte-identity needs the IOP-RnD git history (commit $R2_COMMIT)"; fi

declare -A VAR=( [P]="-DF3_P" [BD]="-DF3_BD" [BS]="-DF3_BS" [BR]="-DF3_BR" [D]="-DF3_D" [S]="-DF3_S" [F]="-DF3_F" [X]="-DF3_X"
                 [ALL]="-DF3_D -DF3_S -DF3_F" [dB]="-DF3_BD -DF3_S -DF3_F" [sB]="-DF3_D -DF3_BS -DF3_F" [fB]="-DF3_D -DF3_S -DF3_BR" [ALLB]="-DF3_BD -DF3_BS -DF3_BR" )
declare -A DESC=( [P]="a consumer written by hand (calls the display sink from on())" [BD]="the display sink through Delivered, no layer" [BS]="the SD sink through Delivered, no layer" [BR]="the ring sink through Delivered, no layer"
                  [D]="display class: Latest + Status" [S]="storage class: Buffer<4> + Retry + HoldOp + Backoff + Recover + Status" [F]="fire-and-forget class: Buffer<3> + Status" [X]="direct with status: DetectError + Status"
                  [ALL]="display + storage + fire-and-forget" [dB]="all, the display bare" [sB]="all, the SD bare" [fB]="all, the ring bare" [ALLB]="all three bare" )
ORDER="P BD BS BR D S F X ALL dB sB fB ALLB"
declare -A T D B
for v in $ORDER; do
  avr-g++ $FL $INC -I . ${VAR[$v]} size_f3.cpp -Wl,--gc-sections -o "$OUT/f3_$v.elf" 2> "$OUT/f3_$v.err" || { bad "avr build [$v]"; head -5 "$OUT/f3_$v.err"; continue; }
  [ -s "$OUT/f3_$v.err" ] && { bad "avr build [$v] warns"; head -3 "$OUT/f3_$v.err"; }
  read -r T[$v] D[$v] B[$v] < <(avr-size "$OUT/f3_$v.elf" | tail -1 | awk '{print $1, $2, $3}')
  avr-objcopy -O binary -j .text -j .data "$OUT/f3_$v.elf" "$OUT/f3_$v.bin"
  if avr-nm -C "$OUT/f3_$v.elf" | grep -E 'malloc|__divmod|__udivmod|__cxa_guard|_GLOBAL__sub_I|__do_global_ctors|__divmodhi|__mulhi|__mulsi|__divmodsi' > "$OUT/hyg_$v.txt"; then bad "[$v] forbidden symbols: $(head -3 "$OUT/hyg_$v.txt" | tr '\n' ' ')"; fi
  avr-objdump -dC --no-show-raw-insn "$OUT/f3_$v.elf" | awk "$STRIP"'
    /^[0-9a-f]+ <.*>:$/ { name=$0; sub(/^[0-9a-f]+ </,"",name); sub(/>:$/,"",name); name=strip(name); next }
    /\t(icall|eicall|ijmp|eijmp)/ { n[name]++ }
    END { for (f in n) printf "%d %s\n", n[f], f }' > "$OUT/ic_$v.txt"
  pump=$(awk '/World::pump\(\)/ {print $1}' "$OUT/ic_$v.txt"); n=$(wc -l < "$OUT/ic_$v.txt")
  if [ "${pump:-0}" != "1" ] || [ "$n" != "1" ]; then bad "[$v] indirect calls: want exactly 1, in pump; got:"; cat "$OUT/ic_$v.txt"; fi
done
ok "every variant links with 0 warnings; no malloc / divide / multiply / guard / global-constructor symbol; pump holds the only indirect call (1 icall) in every image: the delivery edge adds none"
if cmp -s "$OUT/f3_P.bin" "$OUT/f3_BD.bin"; then ok "a consumer declared through Delivered with no layer is the hand-written consumer: the flash images are byte-identical (${T[P]} / ${D[P]} / ${B[P]} text/data/bss)"; else bad "P and BD differ ($(cmp -l "$OUT/f3_P.bin" "$OUT/f3_BD.bin" | wc -l) bytes)"; fi
echo "--- flash / RAM per consumer class (text / data / bss; the sinks and the session are the same in every image)"
printf '  %-6s %-72s %7s %5s %5s\n' variant what text data bss
for v in $ORDER; do printf '  %-6s %-72s %7s %5s %5s\n' "$v" "${DESC[$v]}" "${T[$v]}" "${D[$v]}" "${B[$v]}"; done
echo "  alone = the class over the same sink with no layer;  marginal = all three minus all three with that class bare"
printf '  %-28s %9s %9s   %9s %9s\n' class alone.text alone.RAM marg.text marg.RAM
row() { printf '  %-28s %+9d %+9d   %+9d %+9d\n' "$1" "$((T[$2]-T[$3]))" "$(( (D[$2]+B[$2])-(D[$3]+B[$3]) ))" "$((T[ALL]-T[$4]))" "$(( (D[ALL]+B[ALL])-(D[$4]+B[$4]) ))"; }
row "display (Latest, Status)" D BD dB
row "storage (Buffer, Retry, ...)" S BS sB
row "fire-and-forget (Buffer)" F BR fB
printf '  %-28s %+9d %+9d\n' "direct + status (over the SD bare)" "$((T[X]-T[BS]))" "$(( (D[X]+B[X])-(D[BS]+B[BS]) ))"
printf '  %-28s %+9d %+9d\n' "all three, over all three bare" "$((T[ALL]-T[ALLB]))" "$(( (D[ALL]+B[ALL])-(D[ALLB]+B[ALLB]) ))"

echo "--- simavr: every image runs the session (14 samples; the ring stalls, the card is pulled and put back) to the same checksum as the same build on native"
if [ "$have_sim" = 1 ]; then
  for v in $ORDER; do
    g++ -std=c++17 -O1 -DF3_PARITY ${VAR[$v]} $INC -I . size_f3.cpp -o "$OUT/par_$v" && nat=$("$OUT/par_$v" | sed -n 's/^checksum 0x\(....\)$/\1/p')
    avr=$(avr_sum "$OUT/f3_$v.elf")
    if [ -n "$avr" ] && [ "${avr,,}" = "${nat,,}" ]; then ok "[$v] simulated ATmega328 checksum 0x$avr == native"; else bad "[$v] simulated '${avr:-none}' != native '$nat'"; fi
    echo "$nat" > "$OUT/sum_$v"
  done
  [ "$(cat "$OUT/sum_P")" = "$(cat "$OUT/sum_BD")" ] && ok "P and BD run to the same checksum" || bad "P and BD checksums differ"
  [ "$(cat "$OUT/sum_ALL")" != "$(cat "$OUT/sum_ALLB")" ] && ok "the classes change what the session does (the bare and the layered fan-outs end differently: the layers act)" || bad "the layers change nothing in the session"
else echo "SKIP: simavr / avr-gdb not installed"; fi

exit $rc
