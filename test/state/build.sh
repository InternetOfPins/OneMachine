#!/usr/bin/env bash
# oneMachine/state: every claim about the state module, in one script. Exit status 1 if any check FAILs; a "note" is a measurement.
#   native  g++ and clang++ (+ASan/UBSan): the typed net against the recurrence and a hand-indexed twin, the compile-time rules (each with its
#           own message), the schema hash, the wire format with its atomic read, the faces, array fields, nested compositions
#   oracle  python reads the self-description and computes the hash and the frames without any of the C++ (test/state/*.py)
#   AVR     avr-g++ 7.3 -Os atmega328p: typed == hand-indexed (identical image), names used only for the hash stay out of the flash,
#           the ATmega peer (simavr) and the native peer exchange frames both ways, the cost of the wire format and the faces
#   mutations  each patch must compile and be caught by its named checks
# HAPI=<hapi/include> overrides the HAPI checkout used (default: next to this repo).
cd "$(dirname "$0")"
H=$(realpath "${HAPI:-../../../HAPI/include}")
[ -f "$H/hapi/slots.h" ] || { echo "no HAPI with hapi/slots.h (0.8.0) at ${HAPI:-../../../HAPI/include}: set HAPI=<hapi/include>"; exit 1; }
W=$(mktemp -d); trap 'rm -rf "$W"' EXIT
pass=0; fail=0
ok()   { echo "  ok    $1"; pass=$((pass+1)); }
bad()  { echo "  FAIL  $1: $2"; fail=$((fail+1)); }
note() { echo "  note  $1: $2"; }
have() { command -v "$1" >/dev/null 2>&1; }
INC=$(realpath ../../include)                  # holds oneMachine/state/*.h
AVR="avr-g++ -std=gnu++17 -mmcu=atmega328p -Os -I$H -I$INC"
# host NAME SRC FLAGS.. EXPECT   compile with g++ and clang++ (+ASan/UBSan), run, output must contain EXPECT
host() { local n=$1 s=$2 e=${@: -1}; local f=("${@:3:$#-3}")
  for cxx in g++ clang++; do have $cxx || continue
    $cxx -std=c++17 -Wall -Wextra -fsanitize=address,undefined "${f[@]}" -I"$H" "$s" -o "$W/x" 2>"$W/err" || { bad "$n [$cxx]" "does not compile: $(grep -m1 error "$W/err")"; continue; }
    o=$("$W/x" 2>&1); rc=$?
    if [ $rc -eq 0 ] && printf '%s' "$o" | grep -qF -- "$e"; then ok "$n [$cxx]: $e"; else bad "$n [$cxx]" "exit $rc: $(printf '%s' "$o" | head -c 160)"; fi
  done; }
# rej NAME SRC FLAGS.. MSG   must NOT compile, and say MSG (g++, clang++, avr-g++)
rej() { local n=$1 s=$2 m=${@: -1}; local f=("${@:3:$#-3}")
  for cxx in "g++ -std=c++17" "clang++ -std=c++17" "avr-g++ -std=gnu++17 -mmcu=atmega328p"; do have ${cxx%% *} || continue
    if $cxx -fsyntax-only "${f[@]}" -I"$H" "$s" 2>"$W/err"; then bad "$n [${cxx%% *}]" "compiled, must be rejected"
    elif grep -qF -- "$m" "$W/err"; then ok "$n [${cxx%% *}]: rejected with \"$m\""; else bad "$n [${cxx%% *}]" "rejected, not with \"$m\": $(grep -m1 error "$W/err")"; fi
  done; }
# rejre NAME SRC FLAGS.. REGEX   must NOT compile, and the error matches REGEX (a compiler-worded error, not ours)
rejre() { local n=$1 s=$2 m=${@: -1}; local f=("${@:3:$#-3}")
  for cxx in "g++ -std=c++17" "clang++ -std=c++17" "avr-g++ -std=gnu++17 -mmcu=atmega328p"; do have ${cxx%% *} || continue
    if $cxx -fsyntax-only "${f[@]}" -I"$H" "$s" 2>"$W/err"; then bad "$n [${cxx%% *}]" "compiled, must be rejected"
    elif grep -Eqi -- "$m" "$W/err"; then ok "$n [${cxx%% *}]: rejected (/$m/)"; else bad "$n [${cxx%% *}]" "rejected, not with /$m/: $(grep -m1 error "$W/err")"; fi
  done; }
# accept NAME SRC FLAGS..   must compile (g++, clang++, avr-g++)
accept() { local n=$1 s=$2; local f=("${@:3}")
  for cxx in "g++ -std=c++17" "clang++ -std=c++17" "avr-g++ -std=gnu++17 -mmcu=atmega328p"; do have ${cxx%% *} || continue
    if $cxx -fsyntax-only "${f[@]}" -I"$H" "$s" 2>"$W/err"; then ok "$n [${cxx%% *}]"; else bad "$n [${cxx%% *}]" "$(grep -m1 error "$W/err")"; fi
  done; }
# avrsize NAME SRC FLAGS..   -> $W/NAME.elf; sets SZ, MD5 (whole-program disassembly without addresses)
avrsize() { local n=$1 s=$2; shift 2
  $AVR "$@" "$s" -o "$W/$n.elf" 2>"$W/err" || { bad "$n [avr]" "$(grep -m1 error "$W/err")"; SZ=; MD5=; return; }
  SZ=$(avr-size -C --mcu=atmega328p "$W/$n.elf" | awk '/^Program:/{p=$2}/^Data:/{d=$2}END{print p" B flash / "d" B ram"}')
  MD5=$(avr-objdump -d "$W/$n.elf" | grep -v 'file format' | cut -f2- | md5sum | cut -c1-12); }

TF=(-I"$INC" -I.)
echo "== native: typed net vs the recurrence and vs the hand-indexed net (300 steps x 5 starts, poison-filled next)"
host t1_check_net check_net.cpp "${TF[@]}" "0 failed"

echo "== rules of state.h, each with its own message"
rej    r_dup_tag          rules.cpp "${TF[@]}" -DDUP_TAG "state: two layers claim the same tag"
rej    r_dup_layer_name   rules.cpp "${TF[@]}" -DDUP_LAYER_NAME "state: two layers have the same name"
rej    r_dup_field_name   rules.cpp "${TF[@]}" -DDUP_FIELD_NAME "state: two fields of one layer have the same name"
rej    r_too_many_layers  rules.cpp "${TF[@]}" -DTOO_MANY_LAYERS -DONEMACHINE_STATE_MAX_NAMES=2 "state: more layers than ONEMACHINE_STATE_MAX_NAMES"
rej    r_too_many_fields  rules.cpp "${TF[@]}" -DTOO_MANY_FIELDS -DONEMACHINE_STATE_MAX_NAMES=2 "state: a layer has more fields than ONEMACHINE_STATE_MAX_NAMES"
rej    r_float_field      rules.cpp "${TF[@]}" -DFLOAT_FIELD "state: a field must be an integer or bool"
rej    r_char_field       rules.cpp "${TF[@]}" -DCHAR_FIELD "state: use int8_t/uint8_t"
rejre  r_literal_name     rules.cpp "${TF[@]}" -DLITERAL_NAME "no match|no matching|cannot"
rej    r_no_name          rules.cpp "${TF[@]}" -DNO_NAME "state: a layer tag needs a"
rej    r_missing_tag      rules.cpp "${TF[@]}" -DMISSING_TAG "state: no layer with that tag"
rej    r_next_of_upper    rules.cpp "${TF[@]}" -DNEXT_OF_UPPER "state: no layer with that tag"
rej    r_dependent_order  rules.cpp "${TF[@]}" -DDEPENDENT_ORDER "state: no layer with that tag"
accept r_prev_of_upper    rules.cpp "${TF[@]}" -DPREV_OF_UPPER
accept r_net              rules.cpp "${TF[@]}"
rej    r_own_next         rules.cpp "${TF[@]}" -DOWN_NEXT "state: no layer with that tag"
rejre  r_below_write      rules.cpp "${TF[@]}" -DBELOW_WRITE "read-only|const-qualified|cannot assign"
rejre  r_prev_write       rules.cpp "${TF[@]}" -DPREV_WRITE "read-only|const-qualified|cannot assign"

echo "== schema hash: what changes it (compile time; the pinned value is evaluated by avr-g++ too)"
accept t1_hash_checks hash_checks.cpp "${TF[@]}"

echo "== AVR: typed vs hand-indexed, same harness, whole-program disassembly"
declare -A MD SZS
for mode in noinline inline; do
  M=""; [ $mode = inline ] && M="-DINLINE"
  avrsize "id_${mode}_typed" avr_prog.cpp $M "${TF[@]}";                    T_MD=$MD5; T_SZ=$SZ
  avrsize "id_${mode}_flat"  avr_prog.cpp $M -DFLAT -DFLAT_CAST "${TF[@]}"; F_MD=$MD5; F_SZ=$SZ
  avrsize "id_${mode}_bytes" avr_prog.cpp $M -DFLAT "${TF[@]}";             B_SZ=$SZ
  [ -n "$T_MD" ] && [ "$T_MD" = "$F_MD" ] && ok "typed == hand-indexed (pointer-cast accessors), $mode: identical disassembly ($T_SZ, md5 $T_MD)" || bad "typed vs hand-indexed, $mode" "typed $T_SZ $T_MD, flat $F_SZ $F_MD"
  note "$mode" "hand-indexed with byte-composed accessors: $B_SZ (typed $T_SZ); same operations, different register allocation"
done

echo "== AVR symbols: nothing the hand-indexed program does not have"
BAD='malloc|__cxa_guard|_GLOBAL__sub_I|__do_global_ctors|divmod|__mul|__div|__udiv'
for m in noinline inline; do
  tf="$W/id_${m}_typed.elf"; ff="$W/id_${m}_flat.elf"; [ -f "$tf" ] || { bad "symbols $m" "no elf"; continue; }
  tn=$(avr-nm "$tf" | grep -ciE "$BAD"); fn=$(avr-nm "$ff" | grep -ciE "$BAD")
  ti=$(avr-objdump -d "$tf" | grep -ciE '\s(icall|eicall)\b'); fi=$(avr-objdump -d "$ff" | grep -ciE '\s(icall|eicall)\b')
  { [ "$tn" -le "$fn" ] && [ "$ti" -le "$fi" ]; } && ok "symbols [$m]: typed $tn/$ti (malloc, guard, ctor, div/mul symbols / icall), hand-indexed $fn/$fi" || bad "symbols [$m]" "typed $tn/$ti vs hand-indexed $fn/$fi"
done

echo "== AVR: names used only for the hash stay out of the flash image"
NAMES='fibA|fibB|watch|steps'
image() { avr-objcopy -O binary -j .text -j .data "$1" "$W/img.bin" && strings -a -n 4 "$W/img.bin"; }   # what is flashed: .text and .data, not the ELF symbol table
avrsize n_plain  avr_prog.cpp "${TF[@]}";                U=$SZ
avrsize n_hash   avr_prog.cpp "${TF[@]}" -DHASH;         Hh=$SZ
avrsize n_face   avr_prog.cpp "${TF[@]}" -DFACE;         Fc=$SZ
for n in n_plain n_hash; do
  c=$(image "$W/$n.elf" | grep -cE "$NAMES"); [ "$c" = 0 ] && ok "$n: no layer or field name in the flashed image ($( [ $n = n_plain ] && echo "$U" || echo "$Hh"))" || bad "$n" "$c name strings found"
done
c=$(avr-nm -C "$W/n_hash.elf" | grep -ciE 'fnv|Hasher|schema'); [ "$c" = 0 ] && ok "n_hash: the hash is a constant, no hashing code or symbol" || bad n_hash "$c hashing symbols"
c=$(image "$W/n_face.elf" | grep -cE "$NAMES"); [ "$c" -gt 0 ] && ok "n_face (control): a face that uses the names does put them in the image ($Fc; $c strings), so the check can fail" || bad n_face "the control shows no names: the check above proves nothing"

echo "== mutations: each must compile and be caught by its named checks"
# mut NAME FILE SEDEXPR EXPECT_FAIL.. -- EXPECT_OK..    copy the tree, patch FILE, build check_net (g++ -O0), run it
mut() { local n=$1 f=$2 e=$3; shift 3; local fails=() oks=(); local sw=0
  for a in "$@"; do if [ "$a" = "--" ]; then sw=1; elif [ $sw = 0 ]; then fails+=("$a"); else oks+=("$a"); fi; done
  local d="$W/mut_$n"; rm -rf "$d"; mkdir -p "$d/include" "$d/t"; cp -r "$INC"/. "$d/include/"; cp ./*.h ./*.cpp "$d/t/"
  local orig="./$f" tgt="$d/t/$f"; [ "${f#oneMachine/}" != "$f" ] && { orig="$INC/$f"; tgt="$d/include/$f"; }
  sed -i "$e" "$tgt"; if cmp -s "$orig" "$tgt"; then bad "mutation $n" "the patch did not change $f"; return; fi
  g++ -std=c++17 -O0 -fno-strict-aliasing -I"$d/include" -I"$d/t" -I"$H" "$d/t/check_net.cpp" -o "$d/x" 2>"$W/err" || { bad "mutation $n" "harness failure: the mutant does not compile: $(grep -m1 error "$W/err")"; return; }
  local out; out=$("$d/x" 2>&1); local miss=""
  for c in "${fails[@]}"; do printf '%s' "$out" | grep -q "^CHECK $c: FAIL" || miss="$miss $c"; done
  for c in "${oks[@]}";   do printf '%s' "$out" | grep -q "^CHECK $c: ok"   || miss="$miss !$c"; done
  if [ -z "$miss" ]; then ok "mutation $n: caught by ${fails[*]}${oks[*]:+ (still ok: ${oks[*]})}"; else bad "mutation $n" "not as expected:$miss; failing: $(printf '%s' "$out" | grep -c ': FAIL')"; fi; }
mut inplace  check_net.cpp 's|n.step(p); p = n;|p.step(p); n = p;|'                               values hand watch
mut overlap  oneMachine/state/state.h 's|else \(this->\)\?me() = Body::run(static_cast<const typename O::Res\&>(\*this), prev);|else { S tmp = Body::run(static_cast<const typename O::Res\&>(*this), prev); char* q = reinterpret_cast<char*>(\&this->me()); if (sizeof(S) == 2 \&\& q != reinterpret_cast<char*>(this)) q -= 1; __builtin_memcpy(q, \&tmp, sizeof(S)); }|
s|else self.me() = Body::run(static_cast<const Below\&>(self), prev);|else { S tmp = Body::run(static_cast<const Below\&>(self), prev); char* q = reinterpret_cast<char*>(\&self.me()); if (sizeof(S) == 2 \&\& q != reinterpret_cast<char*>(this)) q -= 1; __builtin_memcpy(q, \&tmp, sizeof(S)); }|' values hand
mut order    net.h 's|state::Layer<FibA,SlotFibA,StepA>, state::Layer<FibB,SlotFibB,StepB>>|state::Layer<FibB,SlotFibB,StepB>, state::Layer<FibA,SlotFibA,StepA>>|' golden-paths golden-hash -- values watch hand disjoint pure-next

T2F=(-I"$INC" -I.)
# t2checks DIR   build check_wire and host_peer in DIR (a tree with include/ for the headers and t/ for these tests) and print CHECK lines:
#                the native checks, the independent oracle (python, from the description text alone) and the pin
t2checks() { local d=$1 x="$1/t"
  g++ -std=c++17 -O1 -I"$H" -I"$d/include" -I"$x" "$x/check_wire.cpp" -o "$x/cw" 2>"$W/err" || { echo "HARNESS FAILURE: check_wire does not compile: $(grep -m1 error "$W/err")"; return 1; }
  g++ -std=c++17 -O1 -I"$H" -I"$d/include" -I"$x" "$x/host_peer.cpp" -o "$x/hp" 2>"$W/err" || { echo "HARNESS FAILURE: host_peer does not compile: $(grep -m1 error "$W/err")"; return 1; }
  "$x/cw" | grep '^CHECK'
  "$x/hp" describe > "$x/desc.txt"; "$x/hp" values 0 > "$x/v0.txt"; "$x/hp" values $STEPS_N > "$x/v5.txt"
  local ph; ph=$(python3 hash_from_description.py < "$x/desc.txt" 2>&1); local nh; nh=$("$x/hp" hash)
  [ "$ph" = "$nh $(grep '^size ' "$x/desc.txt" | cut -d' ' -f2)" ] && echo "CHECK oracle-hash: ok" || echo "CHECK oracle-hash: FAIL -- python: $ph; native: $nh"
  local f0; f0=$(python3 frame_from_description.py "$x/desc.txt" "$x/v0.txt" 2>&1)
  [ "$f0" = "$("$x/hp" frame 0)" ] && echo "CHECK oracle-frame: ok" || echo "CHECK oracle-frame: FAIL -- python $f0"
  local f5; f5=$(python3 frame_from_description.py "$x/desc.txt" "$x/v5.txt" 2>&1)
  [ "$f5" = "$("$x/hp" frame $STEPS_N)" ] && echo "CHECK oracle-frame-stepped: ok" || echo "CHECK oracle-frame-stepped: FAIL -- python $f5"
  { cat "$x/desc.txt"; echo "frame0 $("$x/hp" frame 0)"; echo "frame$STEPS_N $("$x/hp" frame $STEPS_N)"; } > "$x/pin.txt"
  cmp -s "$x/pin.txt" golden.txt && echo "CHECK pin: ok" || echo "CHECK pin: FAIL -- the description or a frame differs from golden.txt"
}
STEPS_N=5
echo "== wire format, native: wire format checks, and the independent oracle (python, from the description text alone)"
rm -rf "$W/base"; mkdir -p "$W/base/include" "$W/base/t"; cp ./*.h ./*.cpp ./*.txt "$W/base/t/"; cp -r "$INC"/. "$W/base/include/"
out=$(t2checks "$W/base"); echo "$out" | sed 's/^CHECK /        /' | sed 's/^/  /' | head -0
if [ -z "$(echo "$out" | grep -v '^CHECK .*: ok')" ] && [ "$(echo "$out" | grep -c '^CHECK')" -ge 15 ]; then ok "t2 native and oracle: $(echo "$out" | grep -c '^CHECK') checks, all ok ($(echo "$out" | sed -n 's/^CHECK \(.*\): ok/\1/p' | tr '\n' ' '))"; else bad "t2 native and oracle" "$(echo "$out" | grep -v '^CHECK .*: ok' | head -3)"; fi
hpb="$W/base/t/hp"
J=$("$hpb" json); V=$(cat "$W/base/t/v0.txt")
python3 - "$J" "$V" <<'PY' && ok "json face: valid JSON, every value equal to the state" || bad "json face" "differs from the state"
import json, sys
j = json.loads(sys.argv[1]); v = dict(l.split('=') for l in sys.argv[2].split('\n') if l)
flat = {'%s/%s' % (l, f): int(x) for l, d in j.items() for f, x in d.items()}
assert flat == {k: int(x) for k, x in v.items()}, (flat, v)
PY

echo "== the real AVR image (simavr) and the native peer: frames both ways, the description, refusals"
avrpeer() { local tag=$1; shift   # avrpeer TAG FLAGS..: native header, AVR peer, run in simavr -> $W/$tag.out (lines without simavr's marks)
  g++ -std=c++17 -O1 "$@" -I"$H" "${T2F[@]}" host_peer.cpp -o "$W/$tag.hp" 2>"$W/err" || { bad "$tag host_peer" "$(grep -m1 error "$W/err")"; return 1; }
  mkdir -p "$W/$tag.in"; "$W/$tag.hp" header > "$W/$tag.in/wire_in.h"
  $AVR "$@" "${T2F[@]}" -I"$W/$tag.in" avr_peer.cpp -o "$W/$tag.elf" 2>"$W/err" || { bad "$tag avr_peer" "$(grep -m1 error "$W/err")"; return 1; }
  timeout 30 simavr -m atmega328p -f 16000000 "$W/$tag.elf" 2>&1 | sed 's/\x1b\[[0-9;]*m//g' | tr -d '\r' | sed 's/\.$//' > "$W/$tag.out"
  grep -q '^END' "$W/$tag.out" || { bad "$tag simavr" "the peer did not finish: $(head -c 200 "$W/$tag.out")"; return 1; }; }
if have simavr && have avr-g++; then
  if avrpeer peer; then
    P="$W/peer.out"; HPB="$W/peer.hp"
    sed -n '1,/^misc\/i64/p' "$P" > "$W/peer.desc"
    cmp -s "$W/peer.desc" "$W/base/t/desc.txt" && ok "the description the AVR emits (names read from flash) is byte-equal to the native one" || bad "AVR description" "differs from the native description"
    ah=$(sed -n 's/^hash //p' "$P"); nh=$("$HPB" hash); ph=$(python3 hash_from_description.py < "$W/peer.desc" 2>&1 | cut -d' ' -f1)
    { [ "$ah" = "$nh" ] && [ "$ph" = "$nh" ]; } && ok "schema hash $nh: AVR's own constant, native constexpr, and python computed from the AVR's text agree" || bad "hash" "avr $ah native $nh python $ph"
    [ "$(sed -n 's/^GOOD status=//p' "$P")" = 0 ] && ok "AVR (int 16 bits) reads the native frame: status Ok" || bad "GOOD status" "$(grep GOOD "$P" | head -2)"
    [ "$(sed -n 's/^GOOD READ //p' "$P")" = "$("$HPB" frame 0)" ] && ok "AVR re-serialises what it read: identical to the native frame" || bad "GOOD READ" "differs"
    aout=$(sed -n 's/^GOOD OUT //p' "$P")
    [ "$aout" = "$("$HPB" frame $STEPS_N)" ] && ok "after $STEPS_N steps AVR and native write the same 37 bytes" || bad "GOOD OUT" "avr $aout native $("$HPB" frame $STEPS_N)"
    "$HPB" verify "$aout" >/dev/null && ok "native reads the AVR's frame: equal to its own state after the steps" || bad "verify" "native does not agree with the AVR frame"
    python3 frame_decode.py "$W/peer.desc" "$aout" > "$W/peer.dec" 2>"$W/err" && "$HPB" values $STEPS_N | cmp -s - "$W/peer.dec" \
      && ok "a decoder that knows only the AVR's description (python, no C++) reads the AVR's frame: every value equals the native state" || bad "decode from description" "$(head -c 200 "$W/err") $(diff <("$HPB" values $STEPS_N) "$W/peer.dec" | head -3)"
    python3 frame_decode.py "$W/peer.desc" "$(sed 's/^4d00f56e/4d00f56f/' <<<"$aout")" >/dev/null 2>&1 && bad "decode refusal" "a frame with another header was decoded" || ok "the same decoder refuses a frame whose header is not the description's hash"
    zero=$("$HPB" zero); refusal_ok=1
    for c in "BADHASH 1" "BADBOOL 3" "SHORT 2" "LONG 2"; do set -- $c
      [ "$(sed -n "s/^$1 status=//p" "$P")" = "$2" ] && [ "$(sed -n "s/^$1 READ //p" "$P")" = "$zero" ] || { refusal_ok=0; bad "AVR refusal $1" "status $(sed -n "s/^$1 status=//p" "$P") (want $2), state $(sed -n "s/^$1 READ //p" "$P" | head -c 30)..."; }; done
    [ $refusal_ok = 1 ] && ok "AVR refuses bad hash / bad bool / short / long frames with the right status and leaves the state untouched"
  fi
  echo "== a machine that declares its widths loosely is refused: SLOPPY (int32 field declared as int)"
  if avrpeer sloppy -DSLOPPY; then
    P="$W/sloppy.out"; HPS="$W/sloppy.hp"
    ah=$(sed -n 's/^hash //p' "$P"); nh=$("$HPS" hash)
    [ "$ah" != "$nh" ] && ok "hash differs between the builds (AVR $ah, native $nh)" || bad "sloppy hash" "equal: $ah"
    grep -q '^misc/i32 i16$' "$P" && "$HPS" describe | grep -q '^misc/i32 i32$' && ok "the two descriptions say why: misc/i32 is i16 on AVR, i32 on the host" || bad "sloppy description" "the difference is not in the text"
    [ "$(sed -n 's/^GOOD status=//p' "$P")" = 1 ] && ok "the AVR refuses the native frame (BadHash) instead of misreading it" || bad "sloppy status" "$(grep GOOD "$P" | head -2)"
    python3 hash_from_description.py < <(sed -n '1,/^misc\/i64/p' "$P") >/dev/null && ok "the AVR's own description of the sloppy net is still self-consistent (python recomputes its hash)" || bad "sloppy self-description" "python disagrees"
  fi
else note t2-avr "simavr or avr-g++ missing: the cross-machine checks were not run"; fi

echo "== loose widths: a plain int in a slot fails the build of the target where it differs (ONEMACHINE_STATE_PIN)"
for cxx in "g++ -std=c++17" "clang++ -std=c++17" "avr-g++ -std=gnu++17 -mmcu=atmega328p"; do have ${cxx%% *} || continue
  $cxx -fsyntax-only "${T2F[@]}" -I"$H" width_alias.cpp 2>"$W/err" && ok "width_alias [${cxx%% *}]: int16_t/int32_t are typedefs of int on some targets (a type-identity rule cannot work)" || bad "width_alias [${cxx%% *}]" "$(grep -m1 error "$W/err")"
  $cxx -fsyntax-only "${T2F[@]}" -I"$H" pin_check.cpp 2>"$W/err" && ok "pin, fixed-width net [${cxx%% *}]" || bad "pin, fixed-width net [${cxx%% *}]" "$(grep -m1 error "$W/err")"
  if $cxx -fsyntax-only "${T2F[@]}" -I"$H" -DSLOPPY pin_check.cpp 2>"$W/err"; then [ "${cxx%% *}" = avr-g++ ] && bad "pin, sloppy net [avr-g++]" "compiled, must be rejected" || ok "pin, sloppy net [${cxx%% *}]: the host's int is 32 bits, the pin holds here"
  else grep -qF "state: the schema of Net2 is not the pinned one on this target" "$W/err" && [ "${cxx%% *}" = avr-g++ ] && ok "pin, sloppy net [avr-g++]: the build fails with state's own message" || bad "pin, sloppy net [${cxx%% *}]" "$(grep -m1 error "$W/err")"; fi
done

echo "== AVR cost of the wire format and the face (fib net: 7 state bytes, 11 frame bytes), against hand-written twins"
cost() { avrsize "cost$RANDOM" avr_cost.cpp "$@" -I"$INC" -I. >/dev/null; }
avrsize c_base   avr_cost.cpp "${T2F[@]}";                          C0=$SZ; M0=$MD5
avrsize c_inc    avr_cost.cpp "${T2F[@]}" -DINC;                    C1=$SZ; M1=$MD5
avrsize c_write  avr_cost.cpp "${T2F[@]}" -DWRITE;                  CW=$SZ
avrsize c_read   avr_cost.cpp "${T2F[@]}" -DREAD;                   CR=$SZ
avrsize c_desc   avr_cost.cpp "${T2F[@]}" -DDESCRIBE;               CD=$SZ
avrsize c_hbase  avr_cost.cpp "${T2F[@]}" -DFLAT_CAST -DHAND_BASE;  CHB=$SZ; MHB=$MD5
avrsize c_hwrite avr_cost.cpp "${T2F[@]}" -DFLAT_CAST -DHAND_WRITE; CHW=$SZ
avrsize c_hread  avr_cost.cpp "${T2F[@]}" -DFLAT_CAST -DHAND_READ;  CHR=$SZ
avrsize c_readi  avr_cost.cpp "${T2F[@]}" -DREAD '-DONEMACHINE_STATE_EACH_INLINE=[[gnu::always_inline]]'; CRI=$SZ; MRI=$MD5
avrsize c_desci  avr_cost.cpp "${T2F[@]}" -DDESCRIBE '-DONEMACHINE_STATE_EACH_INLINE=[[gnu::always_inline]]'; CDI=$SZ; MDI=$MD5
avrsize c_readp  avr_cost.cpp "${T2F[@]}" -DREAD; MRP=$MD5
avrsize c_descp  avr_cost.cpp "${T2F[@]}" -DDESCRIBE; MDP=$MD5
[ -n "$MRI" ] && [ "$MRI" != "$MRP" ] && [ "$MDI" != "$MDP" ] && ok "ONEMACHINE_STATE_EACH_INLINE reaches the walks (it changes the image of read and of describe), so the size policy is not a no-op" || bad "each inline" "the image of read or describe is the same with and without the macro"

[ "$M0" = "$M1" ] && ok "the three headers included, nothing called: identical image ($C0)" || bad "included, unused" "$C0 $M0 vs $C1 $M1"
[ "$M0" = "$MHB" ] && ok "the typed net and the hand-indexed net (pointer-cast accessors) are the same image with the cost harness ($C0)" || bad "cost harness" "typed $M0 hand $MHB"
n0=${C0%% *}; sz() { echo $(( ${1%% *} - n0 )); }
note "write"    "typed +$(sz "$CW") B; hand-written twin +$(( ${CHW%% *} - n0 )) B (portable, straight-line)"
note "read"     "typed +$(sz "$CR") B (with ONEMACHINE_STATE_EACH_INLINE=always_inline: +$(sz "$CRI") B); hand-written twin +$(( ${CHR%% *} - n0 )) B (no atomicity, no bool pass)"
note "describe" "typed +$(sz "$CD") B flash, RAM $(echo "$CD" | sed 's/.*\/ //') vs base $(echo "$C0" | sed 's/.*\/ //') (with always_inline: +$(sz "$CDI") B)"
[ "${CW%% *}" -le "${CHW%% *}" ] && ok "typed write is no larger than the hand-written twin (${CW%% *} B vs ${CHW%% *} B)" || bad "write cost" "typed $CW hand $CHW"
[ "$(echo "$CD" | sed 's/.*\/ //')" = "$(echo "$C0" | sed 's/.*\/ //')" ] && ok "the description adds no RAM: names and fixed text are read from flash" || bad "describe RAM" "$CD vs $C0"

echo "== wire and hash mutations: each must compile and be caught by its named checks"
# mutw NAME FILE SEDEXPR [FILE SEDEXPR].. -- FAIL.. -- OK..    patch a copy of the tree, run t2checks on it
mutw() { local n=$1; shift; local patches=() fails=() oks=() sw=0
  while [ $# -gt 0 ]; do if [ "$1" = "--" ]; then sw=$((sw+1)); shift; continue; fi
    case $sw in 0) patches+=("$1" "$2"); shift 2;; 1) fails+=("$1"); shift;; *) oks+=("$1"); shift;; esac; done
  local d="$W/mw_$n"; rm -rf "$d"; mkdir -p "$d/include" "$d/t"; cp ./*.h ./*.cpp ./*.txt "$d/t/"; cp -r "$INC"/. "$d/include/"
  local i; for ((i=0;i<${#patches[@]};i+=2)); do local f=${patches[i]} e=${patches[i+1]} orig="./${patches[i]}" tgt="$d/t/${patches[i]}"
    [ "${f#oneMachine/}" != "$f" ] && { orig="$INC/$f"; tgt="$d/include/$f"; }
    sed -i "$e" "$tgt"; cmp -s "$orig" "$tgt" && { bad "mutation $n" "the patch did not change $f"; return; }; done
  local out; out=$(t2checks "$d"); if echo "$out" | grep -q 'HARNESS FAILURE'; then bad "mutation $n" "$(echo "$out" | grep -m1 'HARNESS FAILURE')"; return; fi
  local miss=""; for c in "${fails[@]}"; do echo "$out" | grep -q "^CHECK $c: FAIL" || miss="$miss $c"; done
  for c in "${oks[@]}"; do echo "$out" | grep -q "^CHECK $c: ok" || miss="$miss !$c"; done
  [ -z "$miss" ] && ok "mutation $n: caught by ${fails[*]}${oks[*]:+ (still ok: ${oks[*]})}" || bad "mutation $n" "not as expected:$miss; failing: $(echo "$out" | grep -c ': FAIL')"; }
mutw big-endian  oneMachine/state/wire.h 's|for (unsigned i = 0; i < sizeof(E); i++) { \*p++ = uint8_t(u); u = typename UInt<sizeof(E)>::type(u >> 8); }|for (unsigned i = 0; i < sizeof(E); i++) *p++ = uint8_t(u >> (8 * (sizeof(E) - 1 - i)));|' \
                 oneMachine/state/wire.h 's|for (unsigned i = sizeof(E); i-- > 0; ) u = typename UInt<sizeof(E)>::type((u << 8) \| p\[i\]);|for (unsigned i = 0; i < sizeof(E); i++) u = typename UInt<sizeof(E)>::type((u << 8) \| p[i]);|' \
                 -- oracle-frame oracle-frame-stepped pin -- roundtrip roundtrip-stepped canonical
mutw no-bool-check oneMachine/state/wire.h 's|if (\*p > 1) ok = false;||' -- refuse-bool atomic-bool canonical
mutw not-atomic  oneMachine/state/wire.h 's|Reader<false> check{in + 4, true}; r.each(check);|Reader<true> check{in + 4, true}; r.each(check); check.ok = false; { Reader<false> c2{in + 4, true}; r.each(c2); check.ok = c2.ok; }|' -- atomic-bool -- refuse-bool
mutw no-hash-check oneMachine/state/wire.h 's|if (h != schema_v<R>) return Status::BadHash;||' -- refuse-hash atomic-hash
mutw no-length-check oneMachine/state/wire.h 's|if (n != wire_size<R>()) return Status::BadLength;||' -- refuse-length
mutw hash-no-separator oneMachine/state/state.h 's|constexpr void layer(Name n) { h = fnv(fnvs(h, n), 0xFF); }|constexpr void layer(Name n) { h = fnvs(h, n); }|' -- oracle-hash pin
mutw layer-order net2.h 's|state::Layer<FibA,SlotFibA,StepA>, state::Layer<FibB,SlotFibB,StepB>>::Res;|state::Layer<FibB,SlotFibB,StepB>, state::Layer<FibA,SlotFibA,StepA>>::Res;|' -- pin -- oracle-hash oracle-frame roundtrip

echo "== array fields: native checks, the python oracle on the array grammar, the AVR image"
T3F=(-I"$INC" -I.)
have g++ && for cxx in g++ clang++; do have $cxx || continue
  $cxx -std=c++17 -Wall -Wextra -fsanitize=address,undefined -I"$H" "${T3F[@]}" array_check.cpp -o "$W/ac" 2>"$W/err" || { bad "array_check [$cxx]" "$(grep -m1 error "$W/err")"; continue; }
  o=$("$W/ac"); echo "$o" | grep -q '^0 failed' && ok "array_check [$cxx]: $(echo "$o" | grep -c '^CHECK') checks" || bad "array_check [$cxx]" "$(echo "$o" | grep FAIL | head -2)"; done
"$W/ac" describe > "$W/ad.txt"; "$W/ac" values 0 > "$W/av0.txt"; "$W/ac" values 5 > "$W/av5.txt"
[ "$(python3 hash_from_description.py < "$W/ad.txt" 2>&1 | cut -d' ' -f1)" = "$(sed -n 's/^hash //p' "$W/ad.txt")" ] && ok "array net: python recomputes the hash and size from the description (arrays included)" || bad "array oracle hash" "differs"
[ "$(python3 frame_from_description.py "$W/ad.txt" "$W/av0.txt")" = "$("$W/ac" frame 0)" ] && [ "$(python3 frame_from_description.py "$W/ad.txt" "$W/av5.txt")" = "$("$W/ac" frame 5)" ] && ok "array net: python builds the same frame as the native writer, before and after 5 steps" || bad "array oracle frame" "differs"
python3 frame_decode.py "$W/ad.txt" "$("$W/ac" frame 5)" | cmp -s - "$W/av5.txt" && ok "array net: a decoder that knows only the description reads the frame back to the same values" || bad "array decode" "differs"
if have simavr && have avr-g++; then
  { echo "#include <oneMachine/state/state.h>"; printf 'static const uint8_t ARR_IN[] ONEMACHINE_STATE_ROM = {'; "$W/ac" frame 0 | sed 's/../0x&,/g'; printf '};\n'; } > "$W/array_in.h"
  $AVR "${T3F[@]}" -I"$W" array_avr.cpp -o "$W/array_avr.elf" 2>"$W/err" && timeout 30 simavr -m atmega328p -f 16000000 "$W/array_avr.elf" 2>&1 | sed 's/\x1b\[[0-9;]*m//g' | tr -d '\r' | sed 's/\.$//' > "$W/array_avr.out" || bad array_avr "$(grep -m1 error "$W/err")"
  sed -n '1,/^arr\/n /p' "$W/array_avr.out" | cmp -s - "$W/ad.txt" && ok "array net on AVR: the description is byte-equal to the native one" || bad "array avr description" "differs"
  [ "$(sed -n 's/^status=//p' "$W/array_avr.out")" = 0 ] && [ "$(sed -n 's/^OUT //p' "$W/array_avr.out")" = "$("$W/ac" frame 5)" ] && ok "array net on AVR: reads the native frame and writes the same bytes as native after 5 steps (u64[2], i16[3], bool[2], u8[4])" || bad "array avr frame" "$(tail -3 "$W/array_avr.out")"
fi

echo "== nested compositions: a nested APIOf (once, twice) and a nested Chain as one component of an outer chain"
for cxx in g++ clang++; do have $cxx || continue
  $cxx -std=c++17 -Wall -Wextra -Wpedantic -Werror -fsanitize=address,undefined -I"$INC" -I"$H" nested_check.cpp -o "$W/nc" 2>"$W/err" || { bad "nested [$cxx]" "$(grep -m1 error "$W/err")"; continue; }
  o=$("$W/nc"); echo "$o" | grep -q '^0 failed' && ok "nested APIOf and Chain [$cxx]: $(echo "$o" | grep -c '^CHECK') checks (same size, walk order and schema hash as the flat composition, the step holds)" || bad "nested [$cxx]" "$(echo "$o" | grep FAIL | head -2)"
done
rej nested_dup nested_check.cpp -I"$INC" -DDUP_ACROSS "state: two layers claim the same tag"

echo "== compile time of a long chain: the name checks are linear per layer; the limit is the compilers' template depth, reached by HAPI's own nesting"
accept ct_at_cap   compile_time.cpp -I"$INC" -DMODE_CONTRACT -DNLAYERS=32
rej    ct_over_cap compile_time.cpp -I"$INC" -DMODE_CONTRACT -DNLAYERS=33 "state: more layers than ONEMACHINE_STATE_MAX_NAMES"
accept ct_raised   compile_time.cpp -I"$INC" -DMODE_CONTRACT -DNLAYERS=128 -DONEMACHINE_STATE_MAX_NAMES=200
ctt() { local cxx=$1 m=$2 n=$3; LC_ALL=C /usr/bin/time -f "%e %M" $cxx -std=c++17 -fsyntax-only -DMODE_$m -DNLAYERS=$n -DONEMACHINE_STATE_MAX_NAMES=400 -I"$INC" -I"$H" compile_time.cpp 2>&1 >/dev/null | tail -1 | LC_ALL=C awk '{printf "%.2fs/%dMB", $1, $2/1024}'; }
for cxx in g++ clang++; do have $cxx || continue
  note "compile time [$cxx]" "N=32: plain $(ctt $cxx PLAIN 32), state $(ctt $cxx CONTRACT 32)  |  N=128: plain $(ctt $cxx PLAIN 128), state $(ctt $cxx CONTRACT 128)   (-fsyntax-only)"
done

echo; echo "$pass ok, $fail FAIL"; [ $fail -eq 0 ]
