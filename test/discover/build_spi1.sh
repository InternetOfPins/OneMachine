#!/usr/bin/env bash
# SPI discovery round 1 verification.
#   native : g++ -O2 (+ASan/UBSan, + clang if present) -- the identification, empty-slot and RC522 card assertions.
#   rules  : each compile-time rule rejects its case with its own message.
#   irq    : the RC522's interrupt part (spi_irq.cpp) and the ESP8266 delivery components' pin rules (irq_delivery.cpp).
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

echo; echo "=== RC522 under the failure edge (spi_fail.cpp): g++ -O2, then ASan/UBSan ==="
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror $INC spi_fail.cpp -o "$OUT/sf"
"$OUT/sf" || rc=1
g++ -std=c++17 -O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=all $INC spi_fail.cpp -o "$OUT/sfsan"
"$OUT/sfsan" | tail -1 || rc=1
if command -v clang++ >/dev/null; then
  clang++ -std=c++17 -O2 -Wall -Wextra $INC spi_fail.cpp -o "$OUT/sfclang"
  "$OUT/sfclang" | tail -1 || rc=1
fi

echo; echo "=== RC522 under the health monitor (spi_health.cpp): g++ -O2, then ASan/UBSan ==="
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror $INC spi_health.cpp -o "$OUT/sh"
"$OUT/sh" || rc=1
g++ -std=c++17 -O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=all $INC spi_health.cpp -o "$OUT/shsan"
"$OUT/shsan" | tail -1 || rc=1
if command -v clang++ >/dev/null; then
  clang++ -std=c++17 -O2 -Wall -Wextra $INC spi_health.cpp -o "$OUT/shclang"
  "$OUT/shclang" | tail -1 || rc=1
fi

echo; echo "=== RC522 interrupt part (spi_irq.cpp): fail-fast, -DFALLBACK, -DNOCHECK; g++ -O2, then ASan/UBSan ==="
for cfg in "" "-DFALLBACK" "-DNOCHECK"; do
  g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror $cfg $INC spi_irq.cpp -o "$OUT/si"
  echo "[${cfg:-default}] $("$OUT/si")" || rc=1
  g++ -std=c++17 -O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=all $cfg $INC spi_irq.cpp -o "$OUT/sisan"
  "$OUT/sisan" | tail -1 || rc=1
  if command -v clang++ >/dev/null; then
    clang++ -std=c++17 -O2 -Wall -Wextra $cfg $INC spi_irq.cpp -o "$OUT/siclang"
    "$OUT/siclang" | tail -1 || rc=1
  fi
done

echo; echo "=== ESP8266 interrupt delivery (irq_delivery.cpp, host stub): the pins that compile, the ones rejected ==="
DINC="$INC -I ../support/arduino_stub"
g++ -std=c++17 -Wall -Wextra -Werror $DINC irq_delivery.cpp -o "$OUT/idl" && "$OUT/idl" && echo "OK: Sampled<16> and IsrFlag<5> build" || { echo "FAIL: the delivery components do not build"; rc=1; }
for pair in "NEG_D3:GPIO0, GPIO2 or GPIO15 (D3, D4, D8)" "NEG_D4:GPIO0, GPIO2 or GPIO15 (D3, D4, D8)" "NEG_D8:GPIO0, GPIO2 or GPIO15 (D3, D4, D8)" \
            "NEG_ISR16:GPIO16 has no interrupt: use Sampled" "NEG_PAST:an ESP8266 has GPIO0 to GPIO16"; do
  def=${pair%%:*}; msg=${pair#*:}
  if g++ -std=c++17 -D$def $DINC -fsyntax-only irq_delivery.cpp 2>&1 | grep -qF "$msg"; then echo "OK: -D$def rejected: $msg"
  else echo "FAIL: -D$def was not rejected with '$msg'"; rc=1; fi
done

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
  AVRFL="-std=gnu++17 -Os -mmcu=atmega328p -DF_CPU=16000000UL -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -Wall -Wextra"
  avr-g++ $AVRFL $INC -I ../../../OneChip/include avr_spi.cpp -Wl,--gc-sections -o "$OUT/spi.elf" \
    && avr-size "$OUT/spi.elf" || { echo "FAIL: the AVR SPI image does not build"; rc=1; }
  # the driver with no interrupt part is what it was before the interrupt part existed: the same size, to the byte
  got=$(avr-size "$OUT/spi.elf" | awk 'NR==2{print $1" "$2" "$3}')
  want=$(cat ../baselines/rc522_noirq_avr.size)
  if [ "$got" = "$want" ]; then echo "OK: no interrupt part: text/data/bss $got, as before the interrupt part (../baselines/rc522_noirq_avr.size)"
  else echo "FAIL: no interrupt part: text/data/bss $got, the baseline says $want"; rc=1; fi
  for v in 1 2; do
    avr-g++ $AVRFL -DWITH_IRQ=$v $INC -I ../../../OneChip/include avr_spi.cpp -Wl,--gc-sections -o "$OUT/spi_irq$v.elf" \
      && echo "with the interrupt part ($( [ $v = 1 ] && echo 'sampled line' || echo '+ LineCheck + PollOnLineFault' )): $(avr-size "$OUT/spi_irq$v.elf" | awk 'NR==2{print "text "$1", data "$2", bss "$3}')" \
      || { echo "FAIL: the AVR image with the interrupt part ($v) does not build"; rc=1; }
  done
else
  echo "(avr-g++ not found: AVR build skipped)"
fi
exit $rc
