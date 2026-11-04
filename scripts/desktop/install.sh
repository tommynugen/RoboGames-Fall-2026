#!/usr/bin/env bash
# Adds RoboGame Gamemaster to the desktop's app menu, for this user.
# Run it from anywhere; it fills in where this repo is. Run it again if
# the repo moves. --remove takes the entry off again.
set -e
here="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
apps="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
icons="${XDG_DATA_HOME:-$HOME/.local/share}/icons/hicolor/scalable/apps"
entry="$apps/robogame-gamemaster.desktop"

if [ "$1" = "--remove" ]; then
    rm -f "$entry" "$icons/robogame-gamemaster.svg"
    echo "Removed $entry."
    exit 0
fi

mkdir -p "$apps" "$icons"
chmod +x "$here/gamemaster.sh"
cp "$here/robogame-gamemaster.svg" "$icons/"
cat >"$entry" <<END
[Desktop Entry]
Type=Application
Name=RoboGame Gamemaster
Comment=Arena editor, robot console and camera for the RoboGame sim
Exec="$here/gamemaster.sh"
Icon=robogame-gamemaster
Terminal=false
Categories=Development;Robotics;
StartupNotify=false
END
chmod +x "$entry"
command -v update-desktop-database >/dev/null && update-desktop-database -q "$apps" || true
echo "Added $entry. Look for RoboGame Gamemaster in the app menu."
