#!/usr/bin/env bash
# SPI discovery round 1 verification.
#   native : g++ -O2 (+ASan/UBSan, + clang if present) -- the identification, empty-slot and RC522 card assertions.
#   rules  : each compile-time rule rejects its case with its own message.
#   AVR    : avr-g++ -Os atmega328p, linked over the real AVR SPI core -- it builds, and its size.
# Exits non-zero if an assertion fails, a rule does not fire, or the AVR image does not build.
set -e
cd "$(dirname "$0")"
INC="-I ../../include -I ../../../HAPI/include -I ../../../OneBus/include"
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
rc=0

echo "=== native g++ $(g++ -dumpversion) -O2 ==="
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror $INC spi1.cpp -o "$OUT/s1"
"$OUT/s1" || rc=1

echo; echo "=== native -O1 -fsanitize=address,undefined ==="
g++ -std=c++17 -O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=all $INC spi1.cpp -o "$OUT/s1san"
"$OUT/s1san" | tail -1 || rc=1

if command -v clang++ >/dev/null; then
  echo; echo "=== native clang++ $(clang++ -dumpversion) -O2 ==="
  clang++ -std=c++17 -O2 -Wall -Wextra $INC spi1.cpp -o "$OUT/s1clang"
  "$OUT/s1clang" | tail -1 || rc=1
fi

echo; echo "=== compile-fail guards (each must be rejected with its own message) ==="
for pair in "NEG_ID_IDLE:an SPI id of 0x00 or 0xFF is what an empty slot reads" \
            "NEG_FIXED_TWICE:two Fixed entries name the same slot" \
            "NEG_FIXED_PAST:a Fixed slot past the bus's last chip select" \
            "NEG_I2C_ENTRY:an SPI entry is an SPI driver or Fixed<Slot, Driver>" \
            "NEG_DERIVED_DRIVER:driver must derive from SpiDriverBase<itself,W>"; do
  def=${pair%%:*}; msg=${pair#*:}
  if g++ -std=c++17 -D$def $INC -fsyntax-only spi1.cpp 2>&1 | grep -qF "$msg"; then echo "OK: -D$def rejected: $msg"
  else echo "FAIL: -D$def was not rejected with '$msg'"; rc=1; fi
done

if command -v avr-g++ >/dev/null; then
  echo; echo "=== avr-g++ $(avr-g++ -dumpversion) -Os atmega328p (linked, real SPI core) ==="
  avr-g++ -std=gnu++17 -Os -mmcu=atmega328p -DF_CPU=16000000UL -ffunction-sections -fdata-sections \
    -fno-exceptions -fno-rtti -Wall -Wextra $INC -I ../../../OneChip/include avr_spi.cpp -Wl,--gc-sections -o "$OUT/spi.elf" \
    && avr-size "$OUT/spi.elf" || { echo "FAIL: the AVR SPI image does not build"; rc=1; }
else
  echo "(avr-g++ not found: AVR build skipped)"
fi
exit $rc
