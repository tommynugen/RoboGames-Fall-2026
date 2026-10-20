#!/usr/bin/env bash
# Copies arena_perception out of the ROBOGAME-2.0 checkout so the sim can run
# controller_input and autonomy_node. Only reads the checkout, never writes
# to it. The copy is not kept in git. Run it again to pick up changes, then
# rebuild. Usage: sync_arena_perception.sh [PATH/TO/ROBOGAME-2.0]
set -euo pipefail

REPO="${1:-$HOME/Documents/PlatformIO/Projects/RoboGames/ROBOGAME-2.0}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

[ -d "$REPO/src/arena_perception" ] || { echo "no arena_perception in $REPO" >&2; exit 1; }

rsync -a --delete --exclude __pycache__ --exclude '*.egg-info' \
    "$REPO/src/arena_perception/" "$HERE/src/arena_perception/"

echo "copied arena_perception from $REPO at $(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo unknown)"
