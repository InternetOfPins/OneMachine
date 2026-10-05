#!/usr/bin/env bash
# The Python consumer of a machine tree against the spi example's air sensor on a simulated chip: the device built as a process (a pipe, as over a
# serial port) and as a shared library (in-process through ctypes), and test/link/check_tree.py over each. Needs python3.
set -e
cd "$(dirname "$0")"
INC="-I ../../include -I ../../../HAPI/include -I ../../../OneBus/include -I ../../../OneData/include -I ../../../OneMenu/include -I ../../../OneItem/include -I ../../../OneOutput/include -I ../../../OneBit/include -I ../../../OnePin/include -I ../../../OneChip/include -I ../../../OneParse/include -I ../../../OneInput/include -I ../../../OneIO/include"
rc=0
g++ -std=c++17 -O1 -Wall $INC tree_device.cpp -o tree_device
g++ -std=c++17 -O1 -Wall -shared -fPIC -DSIM_LIB $INC tree_device.cpp -o tree_device.so
python3 check_tree.py pipe || rc=1
python3 check_tree.py ctypes || rc=1
# and the device under ASan/UBSan, over the pipe
g++ -std=c++17 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all $INC tree_device.cpp -o tree_device
python3 check_tree.py pipe | sed 's/^OK: python Tree over a pipe$/OK: python Tree over a pipe (ASan+UBSan)/' ; [ "${PIPESTATUS[0]}" = 0 ] || rc=1
rm -f tree_device tree_device.so
exit $rc
