#!/bin/sh
# Builds the MuJoCo-free host tests with g++ or clang++ (Linux CI, macOS); on Windows use build.cmd (MSVC),
# which also builds the MuJoCo simulators and the socket tools.
#   test_model  firmware model + controller unit test
#   test_rti    PAR6 link: wire formats against the RCB runtime's bytes, link state machine
#   test_app    application layer, settings, leader frame, teleoperation fault injection
#   test_web    web UI server
# Usage: tools/sim/build.sh [--run]      (CXX=clang++ to pick another compiler)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC="$HERE/../../src/main"
OUT="$HERE/out"
CXX=${CXX:-g++}
mkdir -p "$OUT"
FW=""
for f in kinematics dynamics control model_selftest rti_protocol rti_link teleop config_store calibration leader_frame \
         json_lite app web_server web_bridge servo_info boot loop_stats; do
    FW="$FW $SRC/$f.cpp"
done
FLAGS="-std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -I$SRC"
for t in test_model test_rti test_app test_web; do
    # shellcheck disable=SC2086
    $CXX $FLAGS "$HERE/$t.cpp" $FW -o "$OUT/$t" -lm
done
echo "Built test_model, test_rti, test_app, test_web in $OUT"
if [ "$1" = "--run" ]; then
    for t in test_model test_rti test_app test_web; do
        echo "=== $t"
        "$OUT/$t"
    done
fi
