#!/usr/bin/env bash
# OneMachine fail:: verification: F1 (the controller and its components on one edge), F5 (failure meets discovery,
# with F5b's per-kind slots), F2 (the return path, and the policies on it), F3 (the delivery edge, per consumer
# class) and F7 (a health monitor over every row's status: flap rate, bus cost, quarantine/disconnect/escalate).
# F4 (MQTT on the return path) stays in R&D: it needs OneBus/mqtt, which is not part of this library.
# Exits non-zero if any claim does not hold.
cd "$(dirname "$0")"
rc=0
./build_f1.sh || rc=1
echo
echo "################ F5 (and F5b) ################"
./build_f5.sh || rc=1
echo
echo "################ F2 ################"
./build_f2.sh || rc=1
echo
echo "################ F3 ################"
./build_f3.sh || rc=1
echo
echo "################ F7 ################"
./build_f7.sh || rc=1
exit $rc
