#!/usr/bin/env bash
# oneMachine/role: every claim about the role module, in one script. Exit status 1 if any check FAILs; a "note" is a measurement.
#   native  g++ and clang++ (+ASan/UBSan): role_check (the test machine through role::Link in one process)
#   rules   each rule of role::Machine and the kinds is a compile error with its own message (g++, clang++, avr-g++)
#   python  python/onemachine drives the test machine as a consumer that knows only role names (check.py), across four firmware
#           variants (rewired, a role added, a role removed); its Schema against test/state's native peers (check_schema.py);
#           examples/python's drive.py against that example's host build
#   AVR     avr-g++ -Os atmega328p in simavr: the role layer's cost per request and per scan, Fixed vs Found vs Pinned, and the link
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
INC=$(realpath ../../include)
F=(-I"$H" -I"$INC")

echo "== native: the test machine through role::Link, in one process"
for cxx in g++ clang++; do have $cxx || continue
  for label in O2 san; do
    if [ $label = O2 ]; then mode="-O2 -Wall -Wextra -Wpedantic -Werror"; else mode="-O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=all"; fi
    if ! $cxx -std=c++17 $mode "${F[@]}" role_check.cpp -o "$W/rc" 2>"$W/err"; then bad "role_check [$cxx $label]" "does not build: $(grep -m1 error "$W/err")"; continue; fi
    o=$("$W/rc" 2>&1); [ $? -eq 0 ] && ok "role_check [$cxx $label]: $(printf '%s' "$o" | tail -1)" || bad "role_check [$cxx $label]" "$(printf '%s' "$o" | grep FAIL | head -3)"
  done
done

echo "== rules, each with its own message"
rej() { local n=$1 d=$2 m=$3
  for cxx in "g++ -std=c++17" "clang++ -std=c++17" "avr-g++ -std=gnu++17 -mmcu=atmega328p"; do have ${cxx%% *} || continue
    if $cxx -fsyntax-only -D$d "${F[@]}" rules.cpp 2>"$W/err"; then bad "$n [${cxx%% *}]" "compiled, must be rejected"
    elif grep -qF -- "$m" "$W/err"; then ok "$n [${cxx%% *}]: \"$m\""; else bad "$n [${cxx%% *}]" "rejected, not with \"$m\": $(grep -m1 error "$W/err")"; fi
  done; }
for cxx in "g++ -std=c++17" "clang++ -std=c++17" "avr-g++ -std=gnu++17 -mmcu=atmega328p"; do have ${cxx%% *} || continue
  $cxx -fsyntax-only "${F[@]}" rules.cpp 2>"$W/err" && ok "rules.cpp plain compiles [${cxx%% *}]" || bad "rules.cpp plain [${cxx%% *}]" "$(grep -m1 error "$W/err")"; done
rej two_on_one     TWO_ON_ONE     "role: two roles on one endpoint"
rej same_name      SAME_NAME      "state: two layers have the same name"
rej max_above_top  MAX_ABOVE_TOP  "role: a light's max is above its endpoint's top"
rej safe_above_max SAFE_ABOVE_MAX "role: a light's safe level is above its max"
rej axis_range     AXIS_RANGE     "role: an axis needs steps/mm > 0 and min < max"

echo "== python: a consumer that knows only role names (python/onemachine)"
if have python3; then
  built=1
  for v in ":sim" "-DWIRING_B:sim_wiring_b" "-DFIRMWARE_V2:sim_v2" "-DFIRMWARE_V3:sim_v3"; do
    g++ -std=c++17 -O2 -Wall -Wextra -Werror ${v%%:*} "${F[@]}" sim_device.cpp -o "$W/${v##*:}" 2>"$W/err" || { bad "sim_device ${v%%:*}" "$(grep -m1 error "$W/err")"; built=0; }
  done
  if [ $built = 1 ]; then
    python3 check.py "$W" > "$W/py.out" 2>&1; rc=$?
    grep -c '^  ok' "$W/py.out" | { read n; [ $rc -eq 0 ] && ok "check.py: $n checks, $(tail -1 "$W/py.out")" || bad "check.py" "$(grep -E 'FAIL|Error' "$W/py.out" | head -5)"; }
  fi
  if g++ -std=c++17 -O2 -Wall -Wextra -Werror "${F[@]}" ../../examples/python/host/main.cpp -o "$W/exhost" 2>"$W/err"; then
    python3 ../../examples/python/drive.py --sim "$W/exhost" --check > "$W/ex.out" 2>&1 && ok "examples/python: drive.py against the host build, $(grep -c '^  ok' "$W/ex.out") checks" || bad "examples/python drive.py" "$(grep -E 'FAIL|Error' "$W/ex.out" | head -3)"
  else bad "examples/python host build" "$(grep -m1 error "$W/err")"; fi
  g++ -std=c++17 -O1 "${F[@]}" -I../state ../state/host_peer.cpp -o "$W/hp" && g++ -std=c++17 -O1 "${F[@]}" -I../state ../state/array_check.cpp -o "$W/ap" &&
    { python3 check_schema.py "$W/hp" "$W/ap" > "$W/sc.out" 2>&1 && ok "check_schema.py: $(grep -c '^ok' "$W/sc.out") checks against the native state peers" || bad "check_schema.py" "$(grep FAIL "$W/sc.out" | head -3)"; }
else note python "no python3: the consumer checks did not run"; fi

echo "== AVR: the role layer's cost (ATmega328P, simavr at 16 MHz), six lights on a PCA9685 behind a mux channel"
if have avr-g++ && have simavr; then
  for v in 4 0 1 2 3; do
    if ! avr-g++ -std=gnu++17 -mmcu=atmega328p -Os -DVARIANT=$v "${F[@]}" avr_cost.cpp -o "$W/c$v.elf" 2>"$W/err"; then bad "avr_cost V$v" "$(grep -m1 error "$W/err")"; continue; fi
    sz=$(avr-size -C --mcu=atmega328p "$W/c$v.elf" | awk '/^Program:/{p=$2}/^Data:/{d=$2}END{print p" B flash / "d" B ram"}')
    o=$(timeout 30 simavr -m atmega328p -f 16000000 "$W/c$v.elf" 2>&1 | sed 's/\x1b\[[0-9;]*m//g' | tr -d '\r' | sed 's/\.$//')
    printf '%s' "$o" | grep -q '^OK' || { bad "avr_cost V$v" "the roles did not reach the device: $(printf '%s' "$o" | head -c 200)"; continue; }
    name=(Fixed Found Pinned "Found + link + descriptions" "no role layer (the app writes the channels)")
    note "${name[$v]}" "$sz, apply of 6 roles $(printf '%s' "$o" | awk '/^APPLY/{print $2}') cycles, after a scan $(printf '%s' "$o" | awk '/^BIND/{print $2}') cycles"
  done
else note AVR "no avr-g++/simavr: the cost was not measured"; fi

echo "role: $pass ok, $fail failed"
[ $fail -eq 0 ]
