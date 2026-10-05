#!/usr/bin/env bash
# The Python consumer of a machine tree against the spi example's air sensor on a simulated chip: the device built as a process (a pipe, as over a
# serial port) and as a shared library (in-process through ctypes), and test/link/check_tree.py over each. Needs python3.
set -e
cd "$(dirname "$0")"
INC="-I ../../include -I ../../../HAPI/include -I ../../../OneBus/include -I ../../../OneData/include -I ../../../OneMenu/include -I ../../../OneItem/include -I ../../../OneOutput/include -I ../../../OneBit/include -I ../../../OnePin/include -I ../../../OneChip/include -I ../../../OneParse/include -I ../../../OneInput/include -I ../../../OneIO/include"
rc=0
# the description by hash (the default): the build output holds the text, written by the generator from the same types
DESC=$(mktemp -d); trap 'rm -rf "$DESC"' EXIT
g++ -std=c++17 -O1 -Wall $INC ../../examples/spi/describe.cpp -o describe && ./describe "$DESC" >/dev/null || rc=1
for mode in "" "-DONEMACHINE_DESC_TEXT"; do
  g++ -std=c++17 -O1 -Wall $mode $INC tree_device.cpp -o tree_device
  g++ -std=c++17 -O1 -Wall -shared -fPIC -DSIM_LIB $mode $INC tree_device.cpp -o tree_device.so
  python3 check_tree.py pipe "$DESC" || rc=1
  python3 check_tree.py ctypes "$DESC" || rc=1
  # and the device under ASan/UBSan, over the pipe
  g++ -std=c++17 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all $mode $INC tree_device.cpp -o tree_device
  python3 check_tree.py pipe "$DESC" | sed 's/^OK: python Tree over a pipe/OK: python Tree over a pipe (ASan+UBSan)/'; [ "${PIPESTATUS[0]}" = 0 ] || rc=1
done
rm -f tree_device tree_device.so describe
exit $rc
