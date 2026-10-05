#!/usr/bin/env bash
# A checked-in reference for "this composition is exactly this big and exports exactly these symbols" -- the
# whole-composition counterpart to zero_cost.sh's on/off comparison (C1 item 9). zero_cost.sh compares two builds
# of the same source against each other; this one compares one build against a snapshot recorded in this repo, so
# a claim like "the bare edge equals the plain program" survives without reaching into another repo's git history
# to prove it (the old shape of these checks: fetch a specific IOP-RnD commit and rebuild it, which only works
# inside that repo and SKIPs everywhere else). Sizes and the symbol NAME set, not raw bytes or addresses -- the
# same reason zero_cost.sh uses them: two builds can lay identical code out differently, harmlessly.
#   baseline.sh record <name> <elf>   saves test/baselines/<name>.size and .syms from <elf> (overwrites; a
#                                     deliberate act after confirming the build it's taken from is correct, not
#                                     something a check step should ever do on its own)
#   baseline.sh check  <name> <elf>   compares <elf> against the saved baseline; OK or a diff, exit 0 or 1
#   baseline.sh selftest              the clone numbering is ignored, a clone of another kind or a renamed symbol is not
set -eu
MODE=${1:?record|check|selftest}; NAME=${2:-}; ELF=${3:-}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIR="$HERE/../baselines"; mkdir -p "$DIR"
TB=""; command -v avr-nm >/dev/null || TB=~/.platformio/packages/toolchain-atmelavr/bin/
size() { "${TB}avr-size" "$1" | tail -1 | awk '{print $1, $2, $3}'; }
# GCC numbers the clones it makes of a function (`[clone .isra.64]`) in the order it makes them, so adding a function elsewhere renumbers them: the
# kind of clone is part of the symbol set, its number is not. Both sides of a comparison are normalised, so a stored list needs no re-recording.
# byte order, not the locale's: the lists are stored in this repo and compared on other machines
norm() { sed -E 's/\.(isra|constprop|part|cold|lto_priv)\.[0-9]+/.\1/g' | LC_ALL=C sort; }
syms() { "${TB}avr-nm" -C "$1" | awk '{ $1=$2=""; print }' | sed 's/^  *//' | norm; }

case "$MODE" in
  selftest)
    a=$(printf 'f() [clone .isra.64]\ng() [clone .constprop.3]\n' | norm)
    b=$(printf 'g() [clone .constprop.12]\nf() [clone .isra.76]\n' | norm)
    c=$(printf 'f() [clone .constprop.64]\ng() [clone .constprop.3]\n' | norm)
    d=$(printf 'f() [clone .isra.64]\nh() [clone .constprop.3]\n' | norm)
    if [ "$a" = "$b" ] && [ "$a" != "$c" ] && [ "$a" != "$d" ]; then echo "OK: baseline.sh ignores the clone numbering, not the kind of clone or the name"
    else echo "FAIL: baseline.sh selftest"; exit 1; fi
    ;;
  record)
    : "${NAME:?name}" "${ELF:?elf}"
    size "$ELF" > "$DIR/$NAME.size"
    syms "$ELF" > "$DIR/$NAME.syms"
    echo "RECORDED: $NAME ($(cat "$DIR/$NAME.size") text/data/bss, $(wc -l < "$DIR/$NAME.syms") symbols)"
    ;;
  check)
    : "${NAME:?name}" "${ELF:?elf}"
    if [ ! -f "$DIR/$NAME.size" ] || [ ! -f "$DIR/$NAME.syms" ]; then
      echo "FAIL: $NAME: no baseline recorded (run: baseline.sh record $NAME <elf>)"; exit 1
    fi
    got_size=$(size "$ELF"); want_size=$(cat "$DIR/$NAME.size")
    ok=1
    if [ "$got_size" != "$want_size" ]; then echo "FAIL: $NAME: size $got_size, baseline $want_size"; ok=0; fi
    tmp=$(mktemp); syms "$ELF" > "$tmp"
    want_syms=$(mktemp); norm < "$DIR/$NAME.syms" > "$want_syms"
    if ! diff -q "$tmp" "$want_syms" >/dev/null; then
      echo "FAIL: $NAME: symbol set differs from its baseline:"; diff "$tmp" "$want_syms" | sed 's/^/  /' | head -20; ok=0
    fi
    rm -f "$tmp" "$want_syms"
    [ "$ok" = 1 ] && echo "OK: $NAME matches its baseline ($got_size text/data/bss, $(wc -l < "$DIR/$NAME.syms") symbols)"
    exit $((1 - ok))
    ;;
  *) echo "usage: baseline.sh record|check <name> <elf>" >&2; exit 2 ;;
esac
