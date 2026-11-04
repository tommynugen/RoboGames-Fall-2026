"""The app's Launch and Stop, with a stand-in for ros2."""

import os
import stat
import time

import pytest

from robogame_sim import sim_runner


@pytest.fixture
def runner(tmp_path):
    fake = tmp_path / "ros2"
    fake.write_text("#!/bin/sh\necho launched \"$@\"\nexec sleep 30\n")
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    sims = sim_runner.SimRunner(launcher=str(fake))
    sims.log = str(tmp_path / "sim.log")
    yield sims
    sims.stop()


def test_launch_and_stop(runner):
    status = runner.start("default", test=True)
    assert status["running"] and status["layout"] == "default"
    assert "layout:=default" in status["command"]
    assert "controller:=hotplug" in status["command"]
    assert status["test"] and not status["ready"]  # still launching
    for _ in range(50):
        if "launched" in runner.status()["tail"]:
            break
        time.sleep(0.05)
    assert "launch robogame_sim sim.launch.py" in runner.status()["tail"]
    with open(runner.log, "ab") as log:
        log.write(b"[create-9] Entity creation successful.\n")
    assert runner.status()["ready"]  # a robot is in: running
    with pytest.raises(ValueError):
        runner.start("default")  # one at a time
    status = runner.stop()
    assert not status["running"] and status["exit"] is not None
    assert status["stopped"]  # by Stop, not a failure


def test_refuses_bad_names(runner):
    for code, body in (runner.post("/api/sim", lambda: {"layout": "../x"}),
                       runner.post("/api/sim",
                                   lambda: {"layout": "no_such_layout"})):
        assert code == 400 and "error" in body
    assert not runner.running()


def test_no_launcher(tmp_path):
    sims = sim_runner.SimRunner(launcher=str(tmp_path / "missing"))
    with pytest.raises(ValueError, match="PATH"):
        sims.start("default")
    assert not os.path.exists(sims.log) or not sims.running()


def test_launch_options(runner):
    status = runner.start("default", options={
        "robots": "1, 2,3", "chase": "", "gui": False, "fpv": True,
        "speed": "3", "step": 0.002, "controller": "classic",
        "drive": "3", "autonomy": True})
    command = status["command"]
    for part in ("robots:=1,2,3", "gui:=false", "fpv:=true", "speed:=3",
                 "step:=0.002", "controller:=classic", "drive:=3",
                 "autonomy:=true"):
        assert part in command.split()
    assert "chase:=" not in command  # cleared: no chase
    assert status["options"]["robots"] == "1,2,3"


def test_test_run_options_over_test():
    used = sim_runner.launch_options(True, {"controller": "joy"})
    assert used["controller"] == "joy" and used["drive"] == "2"


@pytest.mark.parametrize("options", [
    {"robots": ""}, {"robots": "1,x"}, {"chase": "1"}, {"chase": "1,1"},
    {"chase": "1,3"}, {"drive": "4"}, {"controller": "wii"},
    {"speed": "-1"}, {"step": "1"}, {"gui": "maybe"}, {"colour": "red"},
    {"partition": "a b"}, {"motion_device": "/etc/passwd"},
    {"joy_device_id": "-1"}, "robots:=1"])
def test_refuses_bad_options(runner, options):
    code, body = runner.post("/api/sim", lambda: {
        "layout": "default", "options": options})
    assert code == 400 and "error" in body
    assert not runner.running()


def test_camera_without_gazebo(runner):
    code, body = runner.get("/api/sim/view")
    assert code == 200 and not body["available"] and body["error"]
    code, body = runner.post("/api/sim/view", lambda: {"view": "sideways"})
    assert code == 400
