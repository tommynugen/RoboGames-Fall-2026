"""The console's BLE bridge: its log lines read, and Start and Stop with a
stand-in for ros2 that prints what the bridge does."""

import stat
import time

import pytest

from robogame_sim import app_log, ble_bridge

# Lines as `ros2 launch arena_perception bridge.launch.py` prints them,
# from ble_fleet.py, ble_link.py and microcontroller_node.py. If the
# bridge's wording changes, change these and ble_bridge.LINES together.
PREFIX = "[microcontroller_node-1] [INFO] [1762236000.1] " \
    "[microcontroller_node]: "
LINES = [
    ("Microcontroller bridge started, sending speed drive commands",
     ("started", None, "")),
    ("BLE scan failed: No Bluetooth adapters found.",
     ("scan_failed", None, "No Bluetooth adapters found.")),
    ("Discovered robot 3 at AA:BB:CC:DD:EE:03", ("found", 3, "")),
    ("Connecting to robot 3 at AA:BB:CC:DD:EE:03", None),
    ("Connected to robot 3", ("connected", 3, "")),
    ("Robot 3 disconnected", ("lost", 3, "")),
    ("Robot 3 is advertising again", ("found", 3, "")),
    ("Robot 12 connection failed, waiting for it to advertise again: "
     "TimeoutError", ("lost", 12, "")),
    ("Relaying /robot_3/cmd_vel and /robot_3/arm_command", None),
    ("Robot 3 arm command: +", None),
]


@pytest.mark.parametrize("line, expected", LINES)
def test_parses_the_bridges_lines(line, expected):
    assert ble_bridge.parse(PREFIX + line) == expected


FAKE = """#!/bin/sh
echo "args $@"
echo "%sMicrocontroller bridge started, sending speed drive commands"
sleep 0.3
echo "%sDiscovered robot 3 at AA:BB:CC:DD:EE:03"
echo "%sConnected to robot 3"
sleep 0.3
echo "[WARN] %sRobot 3 disconnected"
exec sleep 30
""" % ((PREFIX,) * 4)


@pytest.fixture
def bridge(tmp_path):
    fake = tmp_path / "ros2"
    fake.write_text(FAKE)
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    runner = ble_bridge.BridgeRunner(launcher=str(fake),
                                     book=app_log.LogBook())
    yield runner
    runner.stop()


def wait_for(check, seconds=5.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(0.02)
    raise AssertionError("timed out")


def test_start_follow_and_stop(bridge):
    assert bridge.status()["state"] == "stopped"
    code, status = bridge.post("/api/bridge",
                               lambda: {"expected_robots": 2})
    assert code == 200 and status["running"]
    assert "bridge.launch.py expected_robots:=2" in status["command"]

    wait_for(lambda: bridge.status()["state"] == "found")
    assert bridge.robots() == {3: "connected"}
    wait_for(lambda: bridge.status()["state"] == "lost")
    assert bridge.status()["robots"] == {"3": "lost"}

    # Every line is on the Console tab, under Bridge.
    lines = bridge.book.since()["lines"]
    assert all(line["source"] == "bridge" for line in lines)
    assert any("Discovered robot 3" in line["text"] for line in lines)
    assert any(line["level"] == "warn" for line in lines)

    assert bridge.post("/api/bridge", lambda: {})[0] == 400  # one at a time
    code, status = bridge.post("/api/bridge/stop", lambda: {})
    assert code == 200 and status["state"] == "stopped"
    assert bridge.robots() == {}  # none while it is stopped


def test_bad_asks(bridge, tmp_path):
    assert bridge.post("/api/bridge",
                       lambda: {"expected_robots": -1})[0] == 400
    assert bridge.post("/api/bridge",
                       lambda: {"expected_robots": "x"})[0] == 400
    missing = ble_bridge.BridgeRunner(launcher=str(tmp_path / "missing"))
    code, body = missing.post("/api/bridge", lambda: {})
    assert code == 400 and "PATH" in body["error"]
    assert bridge.get("/api/other") is None
