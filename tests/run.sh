#!/bin/bash
# Usage: tests/run.sh full|demo <tag>
# Starts a fresh headless demo.launch.py, runs harness.py against it, then stops it with SIGINT
# (like Ctrl+C) and records how the shutdown went. Output: tests/out/<tag>/
set -m
D=$(cd "$(dirname "$0")" && pwd); MODE=$1; TAG=$2; OUT=$D/out/$TAG
rm -rf "$OUT"; mkdir -p "$OUT"
cd "$D/.."
source /opt/ros/jazzy/setup.bash; source install/setup.bash

ros2 launch tb3_nav2_demo demo.launch.py gui:=False rviz:=False > "$OUT/launch.log" 2>&1 &
LPID=$!
timeout -k 10 1800 python3 "$D/harness.py" "$MODE" "$OUT/results.json" > "$OUT/harness.log" 2>&1
echo "harness exit $?" >> "$OUT/harness.log"

t0=$(date +%s%N)
kill -INT -$LPID; wait $LPID; rc=$?
ms=$(( ($(date +%s%N) - t0) / 1000000 ))
sleep 2
PROCS='gz sim|nav2_|/amcl|parameter_bridge|robot_state_publisher|map_server|collision_monitor'
left=$(pgrep -f "$PROCS" | wc -l)
crashed=$(grep -oE '\[[a-z_]+-[0-9]+\]: process has died \[pid [0-9]+, exit code -(11|6)' "$OUT/launch.log" \
  | sed -E 's/\[([a-z_]+)-.*code (-[0-9]+)/\1 (\2)/' | paste -sd, -)
echo "{\"launch_exit\": $rc, \"shutdown_ms\": $ms, \"leftover_procs\": $left, \"crashed_on_shutdown\": \"$crashed\"}" > "$OUT/shutdown.json"
[ "$left" -gt 0 ] && pkill -9 -f "$PROCS"
exit 0
