#!/usr/bin/env bash
# The zero-cost check, as one reusable tool instead of a per-round script (C1 item 9):
#   "An option not chosen inside a present component must cost nothing" -- checked by building the SAME source twice
#   (component present, every option off / the option's code not compiled in at all) and comparing section sizes
#   (text, data, bss -- not raw bytes: two builds can lay the same code out differently, harmlessly, once anything
#   crosses a template-instantiation boundary differently between them) and the SET of symbol names (not addresses,
#   which move; a symbol appearing or disappearing is real, a symbol at a different offset is not).
#
#   zero_cost.sh <name> <source.cpp> "<include flags>" "<on flags>" "<off flags>" [avr-mmcu, default atmega328p]
#
# Exit 0 and "OK" if sections and symbol-name sets match; exit 1 and a diff otherwise. AVR only (avr-g++ -Os), since
# that is where a hidden byte costs something; a native build cost is answered by the checksums already in each round.
set -eu
NAME=${1:?name}; SRC=${2:?source.cpp}; INC=${3:-}; ONFL=${4:-}; OFFFL=${5:-}; MMCU=${6:-atmega328p}
TB=""; command -v avr-g++ >/dev/null || TB=~/.platformio/packages/toolchain-atmelavr/bin/
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
FL="-std=gnu++17 -Os -mmcu=$MMCU -DF_CPU=16000000UL -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -Wall -Wextra"

build() {  # tag flags -> $OUT/<tag>.elf, or exits 1 printing the compiler's errors
  local tag="$1" flags="$2"
  if ! "${TB}avr-g++" $FL $flags $INC "$SRC" -Wl,--gc-sections -o "$OUT/$tag.elf" 2> "$OUT/$tag.err"; then
    echo "FAIL: $NAME [$tag] does not build"; cat "$OUT/$tag.err"; exit 1
  fi
  local w; w=$(grep -c warning "$OUT/$tag.err" || true)
  [ "$w" = 0 ] || { echo "FAIL: $NAME [$tag] $w warnings"; cat "$OUT/$tag.err"; exit 1; }
}

build on  "$ONFL"
build off "$OFFFL"

read -r ton don bon < <("${TB}avr-size" "$OUT/on.elf"  | tail -1 | awk '{print $1, $2, $3}')
read -r tof dof bof < <("${TB}avr-size" "$OUT/off.elf" | tail -1 | awk '{print $1, $2, $3}')

# symbol names only, address and value columns stripped; sorted, so instantiation order cannot cause a spurious diff
"${TB}avr-nm" -C "$OUT/on.elf"  | awk '{ $1=$2=""; print }' | sed 's/^  *//' | LC_ALL=C sort > "$OUT/on.sym"
"${TB}avr-nm" -C "$OUT/off.elf" | awk '{ $1=$2=""; print }' | sed 's/^  *//' | LC_ALL=C sort > "$OUT/off.sym"

ok=1
if [ "$ton" != "$tof" ] || [ "$don" != "$dof" ] || [ "$bon" != "$bof" ]; then
  echo "FAIL: $NAME: sections differ -- on text=$ton data=$don bss=$bon, off text=$tof data=$dof bss=$bof"
  ok=0
fi
if ! diff -q "$OUT/on.sym" "$OUT/off.sym" >/dev/null; then
  echo "FAIL: $NAME: symbol sets differ:"
  diff "$OUT/on.sym" "$OUT/off.sym" | sed 's/^/  /' | head -20
  ok=0
fi
[ "$ok" = 1 ] && echo "OK: $NAME: sections and symbols identical between [on: $ONFL] and [off: $OFFFL] (text=$ton data=$don bss=$bon)"
exit $((1 - ok))
