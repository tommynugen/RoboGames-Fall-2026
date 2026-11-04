#!/usr/bin/env bash
# Starts the RoboGame Gamemaster with ROS and this workspace sourced, in
# its own window (closing it quits the app). If it is already running,
# this just opens another window on it. Output goes to
# ~/.cache/robogame/gamemaster.log. The desktop entry runs this script.
repo="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
log="${XDG_CACHE_HOME:-$HOME/.cache}/robogame/gamemaster.log"
mkdir -p "$(dirname "$log")"

say() {
    echo "$1" >>"$log"
    command -v notify-send >/dev/null && notify-send "RoboGame Gamemaster" "$1"
}

# Already up: just show it, without waiting for ROS to source.
# app_window.py needs no ROS; it ends with 2 if it cannot show a window.
if command -v curl >/dev/null && curl -s -o /dev/null -m 1 http://localhost:8766/; then
    python3 "$repo/src/robogame_sim/robogame_sim/app_window.py" \
        http://localhost:8766/ >>"$log" 2>&1 && exit 0
    exec xdg-open http://localhost:8766/
fi

ros="/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash"
[ -f "$ros" ] || { say "No ROS found at $ros."; exit 1; }
[ -f "$repo/install/setup.bash" ] || {
    say "Build the workspace first: colcon build --symlink-install in $repo."
    exit 1
}
set +u
source "$ros"
source "$repo/install/setup.bash"
cd "$repo"
echo "--- $(date) ---" >>"$log"
exec ros2 run robogame_sim robogame_gamemaster >>"$log" 2>&1
