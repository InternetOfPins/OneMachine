#!/usr/bin/env bash
# The size of the machine tree and its link, as a full balance: every variant against the bare image (no link) on the same target.
#   AVR      test/link/avr_tree.cpp, avr-g++ -Os atmega328p: flash = text + data, RAM = data + bss
#   ESP8266  examples/spi with PlatformIO (env d1_mini is bare: no link, the reader's plain driver; d1_mini_link carries the link, the reader as a machine
#            and the tree over both machines; d1_mini_link_text sends the description as text): flash and RAM as PlatformIO reports them
# Variants: "name:avr flags:pio env" per line in VARIANTS (an empty env: no ESP8266 build for that variant). ESP=0 skips the ESP8266 builds.
set -e
cd "$(dirname "$0")"
INC="-I ../../include -I ../../../HAPI/include -I ../../../OneBus/include -I ../../../OneChip/include -I ../../../OneData/include -I ../../../OneMenu/include -I ../../../OneItem/include -I ../../../OneOutput/include -I ../../../OneBit/include -I ../../../OnePin/include -I ../../../OneParse/include -I ../../../OneInput/include -I ../../../OneIO/include"
AVRFL="-std=gnu++17 -Os -mmcu=atmega328p -DF_CPU=16000000UL -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics"
VARIANTS=${VARIANTS:-"bare::d1_mini
reader:-DREADER:
reader_machine:-DREADER -DMACHINE:
link:-DLINK:d1_mini_link
link_text:-DLINK -DONEMACHINE_DESC_TEXT:d1_mini_link_text"}
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
printf '%-14s %10s %8s %12s %10s\n' variant avr_flash avr_ram esp_flash esp_ram
while IFS=: read -r name flags env; do
  [ -z "$name" ] && continue
  af=- ar=- ef=- er=-
  if command -v avr-g++ >/dev/null; then
    avr-g++ $AVRFL $flags $INC avr_tree.cpp -Wl,--gc-sections -o "$OUT/$name.elf" 2>"$OUT/$name.err" || { echo "FAIL: avr $name"; cat "$OUT/$name.err" | grep error | head; exit 1; }
    read -r t d b _ < <(avr-size "$OUT/$name.elf" | awk 'NR==2')
    af=$((t + d)); ar=$((d + b))
  fi
  if [ -n "$env" ] && [ "${ESP:-1}" = 1 ] && command -v pio >/dev/null; then
    log=$(cd ../../examples/spi && pio run -e "$env" 2>&1) || { echo "FAIL: pio $env"; echo "$log" | grep -E "error" | head; exit 1; }
    er=$(echo "$log" | sed -n 's/^RAM:.*used \([0-9]*\) bytes.*/\1/p'); ef=$(echo "$log" | sed -n 's/^Flash:.*used \([0-9]*\) bytes.*/\1/p')
  fi
  printf '%-14s %10s %8s %12s %10s\n' "$name" "$af" "$ar" "$ef" "$er"
done <<< "$VARIANTS"
