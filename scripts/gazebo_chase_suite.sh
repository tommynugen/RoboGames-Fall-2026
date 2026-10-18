#!/bin/bash
# Chase scenarios on test4 in headless Gazebo, four at a time.
#
#   scripts/gazebo_chase_suite.sh OUT_DIR [SCENARIO ...]
#
# Each scenario gets a sim of its own (sim.launch.py robots:=1,2 chase:=1,2
# layout:=test4, no window, no FPV cameras) on its own ROS domain and
# Gazebo partition, with 2 ms physics steps as fast as the computer goes,
# and gazebo_chase_test.py placing the robots and logging. A sim that never
# sends anything is started again, up to three times. With no scenarios it
# runs them all. Results are OUT_DIR/SCENARIO.csv and .log, the launch's
# output in OUT_DIR/SCENARIO.launch.log, and a line each from
# gazebo_chase_score.py at the end. RUN_SECONDS (default 60) caps each run in
# sim time, PARALLEL (default 4) how many run at once.
#
# Run from the workspace, built and sourced:
#   source /opt/ros/lyrical/setup.bash && source install/setup.bash
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${1:?usage: $0 OUT_DIR [SCENARIO ...]}
shift
mkdir -p "$OUT"
ALL="slot_sw slot_se slot_ne slot_nw slot_in_ne slot_in_sw slot_follow
     tower_e tower_w tower_n tower_ring tower_nudge tower_nudge2 tower_follow"
SCENARIOS=${*:-$ALL}
SECONDS_CAP=${RUN_SECONDS:-60}
PARALLEL=${PARALLEL:-4}

run() {
    local name=$1 domain=$2
    export ROS_DOMAIN_ID=$domain
    export GZ_PARTITION=chase_suite_$domain
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    for attempt in 1 2 3; do
        setsid ros2 launch robogame_sim sim.launch.py gui:=false fpv:=false \
            robots:=1,2 chase:=1,2 layout:=test4 partition:=$GZ_PARTITION \
            speed:=0 step:=0.002 > "$OUT/$name.launch.log" 2>&1 &
        local launch=$!
        sleep 6
        timeout $(( ${SECONDS_CAP%.*} * 3 + 60 )) python3 \
            "$HERE/gazebo_chase_test.py" "$name" "$SECONDS_CAP" \
            "$OUT/$name.csv" > "$OUT/$name.log" 2>&1
        local code=$?
        kill -INT -- -$launch 2>/dev/null
        for _ in $(seq 20); do
            kill -0 $launch 2>/dev/null || break
            sleep 0.5
        done
        kill -KILL -- -$launch 2>/dev/null
        [ $code -eq 3 ] || break
        sleep 2
    done
}

slot=0
for name in $SCENARIOS; do
    run "$name" $((81 + slot)) &
    # Not all at once: sims started in the same second sometimes never
    # hear each other's nodes.
    sleep 2
    slot=$(( (slot + 1) % PARALLEL ))
    [ $slot -eq 0 ] && wait
done
wait

for name in $SCENARIOS; do
    python3 "$HERE/gazebo_chase_score.py" "$OUT/$name.csv"
    echo "  freed itself $(grep -c 'freeing itself' "$OUT/$name.launch.log") times"
done
