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
