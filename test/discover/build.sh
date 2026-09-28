#!/usr/bin/env bash
# discoverCompose: every round's verification. Exit 0 = all claims hold.
cd "$(dirname "$0")"
rc=0
./build_r1.sh || rc=1
echo; echo "################ Round 2 ################"
./build_r2.sh || rc=1
echo; echo "################ Round 3 ################"
./build_r3.sh || rc=1
echo; echo "################ Round 3b ################"
./build_r3b.sh || rc=1
exit $rc
