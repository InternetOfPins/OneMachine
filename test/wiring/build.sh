#!/usr/bin/env bash
# The wiring spec dry run (OneMachine Redrawn, experiment 4): test/wiring/check_wiring.py against the simulated device (test/link/tree_device.cpp).
# Needs python3 (3.11+: tomllib).
set -e
cd "$(dirname "$0")"
INC="-I ../../include -I ../../../HAPI/include -I ../../../OneBus/include -I ../../../OneData/include -I ../../../OneMenu/include -I ../../../OneItem/include -I ../../../OneOutput/include -I ../../../OneBit/include -I ../../../OnePin/include -I ../../../OneChip/include -I ../../../OneParse/include -I ../../../OneInput/include -I ../../../OneIO/include"
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
g++ -std=c++17 -O1 $INC ../link/tree_device.cpp -o "$OUT/tree_device"
python3 check_wiring.py "$OUT/tree_device" ../support/arduino_stub
