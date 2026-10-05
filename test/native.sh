#!/usr/bin/env bash
# The native test programs, built and run with $CXX (default g++): -O2 with -Wall -Wextra -Wpedantic -Werror, then
# ASan+UBSan. Each program exits non-zero when a check fails. Exits non-zero if any program fails to build or run.
# Expects HAPI and OneBus next to this repo (../../HAPI, ../../OneBus). The AVR and simulator checks are in each
# suite's build.sh; this is the part that needs only a C++17 compiler.
#
#   CXX=clang++ test/native.sh
set -u
cd "$(dirname "$0")"
CXX=${CXX:-g++}
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
INC="-I ../../include -I ../../../HAPI/include -I ../../../OneBus/include"
rc=0

# dir/program[:extra flags]
PROGRAMS=(
  discover/round1
  discover/round2:-DR2_NO_MQTT
  discover/round3
  discover/round3b
  discover/spi1
  discover/spi_fail
  discover/spi_health
  discover/spi_irq
  discover/spi_irq:-DFALLBACK
  discover/spi_irq:-DNOCHECK
  discover/bmp280_comp
  fail/unit_f1
  fail/unit_f2
  fail/unit_f3
  fail/roundF1
  fail/roundF5:-DF5_COUNT\ -DF5_TAP
  fail/roundF7
  fail/services
  fail/twi_reprobe_ids
  rosCompose/round1
  state/check_net
  state/check_wire
  state/array_check
  state/nested_check
  state/example
  role/role_check
  role/call_check
  role/payload_check
)

run() {  # label dir name extra-flags compile-flags...
  local label="$1" dir="$2" name="$3" extra="$4"; shift 4
  local bin="$OUT/${name}_${label}"
  if ! (cd "$dir" && $CXX -std=c++17 "$@" $extra $INC "$name.cpp" -o "$bin") 2> "$OUT/err.txt"; then
    echo "FAIL: $dir/$name [$label] does not build"; sed 's/^/  /' "$OUT/err.txt" | head -20; rc=1; return
  fi
  if (cd "$dir" && "$bin" > "$OUT/out.txt" 2>&1); then
    echo "OK: $dir/$name [$label] $(tail -1 "$OUT/out.txt" | cut -c1-70)"
  else
    echo "FAIL: $dir/$name [$label] exited non-zero"; tail -15 "$OUT/out.txt" | sed 's/^/  /'; rc=1
  fi
}

echo "=== $CXX $($CXX -dumpversion) ==="
for p in "${PROGRAMS[@]}"; do
  path=${p%%:*}; extra=""; [ "$path" != "$p" ] && extra=${p#*:}
  d=$(dirname "$path"); n=$(basename "$path")
  run O2 "$d" "$n" "$extra" -O2 -Wall -Wextra -Wpedantic -Werror
  run san "$d" "$n" "$extra" -O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=all
done
exit $rc
