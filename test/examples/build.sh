#!/usr/bin/env bash
# Every example, built for real for the Nano (PlatformIO, Arduino framework, atmega328p) -- so an example can't
# silently rot as the library evolves. A broken first example is the worst first impression a library can make.
# This is a compile check, not a correctness one: what the library's own test/ suites already cover (zero-cost,
# the compile-time rules, the failure and health mutations) is not re-proven here, only that the example still
# builds against the current headers. Exits non-zero if any example fails to build.
set -u
cd "$(dirname "$0")"
PIO=${PIO:-pio}; command -v "$PIO" >/dev/null || PIO=~/.platformio/penv/bin/pio
rc=0
ok()  { echo "OK: $*"; }
bad() { echo "FAIL: $*"; rc=1; }

for dir in ../../examples/*/; do
  name=$(basename "$dir")
  [ -f "$dir/platformio.ini" ] || continue
  out=$("$PIO" run -d "$dir" -e nano 2>&1)
  if echo "$out" | grep -q '\[SUCCESS\]'; then
    size=$(echo "$out" | grep -A1 'Flash:' | tail -1)
    ok "examples/$name builds for the Nano ($(echo "$out" | sed -n 's/^Flash: .*(used \([0-9]*\) bytes.*/\1 B flash/p'), $(echo "$out" | sed -n 's/^RAM:   .*(used \([0-9]*\) bytes.*/\1 B RAM/p'))"
  else
    bad "examples/$name does not build for the Nano"
    echo "$out" | grep -E 'error:|Error' | sed 's/^/  /'
  fi
done
exit $rc
