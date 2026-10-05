#!/usr/bin/env bash
# SPI discovery round 1 verification.
#   native : g++ -O2 (+ASan/UBSan, + clang if present) -- the identification, empty-slot and RC522 card assertions.
#   rules  : each compile-time rule rejects its case with its own message.
#   bmp    : the BMP280 as a machine of ItemDef nodes (bmp_machine.cpp, against a simulated chip with the datasheet's worked example), and its
#            capture and restore under a failure edge: reset behind the host's back, unplugged, replaced (bmp_capture.cpp).
#   link   : the Python consumer of the machine tree (python/onemachine/tree.py) against the air sensor on a simulated chip (test/link/build.sh).
#   wiring : the rig's wiring spec checked, emitted as the App composition and diffed against the device's description (test/wiring/build.sh).
#   irq    : the RC522's interrupt part (spi_irq.cpp) and the ESP8266 delivery components' pin rules (irq_delivery.cpp).
#   AVR    : avr-g++ -Os atmega328p, linked over the real AVR SPI core -- it builds, and its size.
# Exits non-zero if an assertion fails, a rule does not fire, or the AVR image does not build.
set -e
cd "$(dirname "$0")"
INC="-I ../../include -I ../../../HAPI/include -I ../../../OneBus/include"
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
rc=0
# run a test binary: its last line, and a failure (its exit status, not tail's) when it does not exit 0
run() { local o s; o=$("$@" 2>&1); s=$?; printf '%s\n' "$o" | tail -1; if [ $s -ne 0 ]; then echo "FAIL: $* exited $s"; printf '%s\n' "$o" | head -4; rc=1; fi; }

echo "=== native g++ $(g++ -dumpversion) -O2 ==="
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror $INC spi1.cpp -o "$OUT/s1"
"$OUT/s1" || rc=1

echo; echo "=== native -O1 -fsanitize=address,undefined ==="
g++ -std=c++17 -O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=all $INC spi1.cpp -o "$OUT/s1san"
run "$OUT/s1san"

if command -v clang++ >/dev/null; then
  echo; echo "=== native clang++ $(clang++ -dumpversion) -O2 ==="
  clang++ -std=c++17 -O2 -Wall -Wextra $INC spi1.cpp -o "$OUT/s1clang"
  run "$OUT/s1clang"
fi

echo; echo "=== RC522 under the failure edge (spi_fail.cpp): g++ -O2, then ASan/UBSan ==="
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror $INC spi_fail.cpp -o "$OUT/sf"
"$OUT/sf" || rc=1
g++ -std=c++17 -O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=all $INC spi_fail.cpp -o "$OUT/sfsan"
run "$OUT/sfsan"
if command -v clang++ >/dev/null; then
  clang++ -std=c++17 -O2 -Wall -Wextra $INC spi_fail.cpp -o "$OUT/sfclang"
  run "$OUT/sfclang"
fi

echo; echo "=== RC522 under the health monitor (spi_health.cpp): g++ -O2, then ASan/UBSan ==="
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror $INC spi_health.cpp -o "$OUT/sh"
"$OUT/sh" || rc=1
g++ -std=c++17 -O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=all $INC spi_health.cpp -o "$OUT/shsan"
run "$OUT/shsan"
if command -v clang++ >/dev/null; then
  clang++ -std=c++17 -O2 -Wall -Wextra $INC spi_health.cpp -o "$OUT/shclang"
  run "$OUT/shclang"
fi

echo; echo "=== RC522 interrupt part (spi_irq.cpp): fail-fast, -DFALLBACK, -DNOCHECK; g++ -O2, then ASan/UBSan ==="
for cfg in "" "-DFALLBACK" "-DNOCHECK"; do
  g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror $cfg $INC spi_irq.cpp -o "$OUT/si"
  echo "[${cfg:-default}] $("$OUT/si")" || rc=1
  g++ -std=c++17 -O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=all $cfg $INC spi_irq.cpp -o "$OUT/sisan"
  run "$OUT/sisan"
  if command -v clang++ >/dev/null; then
    clang++ -std=c++17 -O2 -Wall -Wextra $cfg $INC spi_irq.cpp -o "$OUT/siclang"
    run "$OUT/siclang"
  fi
done

echo; echo "=== BMP280 as a machine of ItemDef nodes (bmp_machine.cpp): g++ -O1, then ASan/UBSan, then clang++ ==="
MINC="$INC -I ../../../OneData/include -I ../../../OneMenu/include -I ../../../OneItem/include -I ../../../OneOutput/include -I ../../../OneBit/include -I ../../../OnePin/include -I ../../../OneChip/include -I ../../../OneParse/include -I ../../../OneInput/include -I ../../../OneIO/include"
g++ -std=c++17 -O1 -Wall $MINC bmp_machine.cpp -o "$OUT/bm" && { "$OUT/bm" || rc=1; }
g++ -std=c++17 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all $MINC bmp_machine.cpp -o "$OUT/bmsan" && { run "$OUT/bmsan"; }
if command -v clang++ >/dev/null; then
  clang++ -std=c++17 -O1 $MINC bmp_machine.cpp -o "$OUT/bmclang" && { run "$OUT/bmclang"; }
fi

echo; echo "=== BMP280 machine, capture and restore under a failure edge (bmp_capture.cpp): g++ -O1, ASan/UBSan, clang++ ==="
g++ -std=c++17 -O1 -Wall $MINC bmp_capture.cpp -o "$OUT/bc" && { "$OUT/bc" || rc=1; }
g++ -std=c++17 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all $MINC bmp_capture.cpp -o "$OUT/bcsan" && { run "$OUT/bcsan"; }
if command -v clang++ >/dev/null; then
  clang++ -std=c++17 -O1 $MINC bmp_capture.cpp -o "$OUT/bcclang" && { run "$OUT/bcclang"; }
fi

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

echo; echo "=== the Python consumer of the machine tree (test/link): over a pipe and in-process (ctypes) ==="
if command -v python3 >/dev/null; then bash ../link/build.sh 2>&1 | grep -E "^OK|^FAIL|Error|Traceback" || rc=1; else echo "(python3 not found: skipped)"; fi

echo; echo "=== the wiring spec dry run (test/wiring): the rig's spec, its composition, the device's description; the IRQ on D4 refused ==="
if python3 -c "import tomllib" 2>/dev/null; then bash ../wiring/build.sh > "$OUT/wiring.txt" 2>&1 || rc=1; grep -E "^OK|^FAIL|^emitted|error:|C\+\+ build|Traceback" "$OUT/wiring.txt"; else echo "(python3 3.11+ not found: skipped)"; fi

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
