#!/usr/bin/env bash
# Every example, built for real for each board its platformio.ini names (PlatformIO, Arduino framework: the Nano for
# most, the D1 mini for examples/spi) -- so an example can't
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
  for env in $(sed -n 's/^\[env:\(.*\)\]$/\1/p' "$dir/platformio.ini"); do
    out=$("$PIO" run -d "$dir" -e "$env" 2>&1)
    if echo "$out" | grep -q '\[SUCCESS\]'; then
      ok "examples/$name builds for $env ($(echo "$out" | sed -n 's/^Flash: .*(used \([0-9]*\) bytes.*/\1 B flash/p'), $(echo "$out" | sed -n 's/^RAM:   .*(used \([0-9]*\) bytes.*/\1 B RAM/p'))"
    else
      bad "examples/$name does not build for $env"
      echo "$out" | grep -E 'error:|Error' | sed 's/^/  /'
    fi
  done
done
exit $rc
