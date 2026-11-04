"""The Chase tab's runs: 2D chase_sim for real, the Gazebo suite with
stand-in scripts."""

import os
import stat
import time

import pytest

from robogame_sim import chase_runs

SCORE = os.path.join(os.path.dirname(__file__), "..", "..", "..", "scripts",
                     "gazebo_chase_score.py")


def wait(runner, kind, seconds=60):
    for _ in range(int(seconds / 0.1)):
        if not getattr(runner, kind).running():
            return
        time.sleep(0.1)
    raise AssertionError(f"{kind} still running")


def test_2d_runs_and_scores(tmp_path):
    runner = chase_runs.ChaseRunner(scripts=str(tmp_path), folder=tmp_path)
    code, body = runner.post("/api/chase/2d", lambda: {
        "layouts": ["default"], "behaviours": ["still"], "seeds": 2,
        "seconds": 5})
    assert code == 200 and body["2d"]["running"]
    assert runner.brief()["2d"][1] == 2
    wait(runner, "flat")
    status = runner.status()["2d"]
    assert status["exit"] == 0, status["tail"]
    assert [r["seed"] for r in status["runs"]] == [0, 1]
    assert status["summary"][0]["behaviour"] == "still"
    assert status["summary"][0]["runs"] == 2


@pytest.mark.parametrize("body", [
    {"layouts": ["../x"]}, {"layouts": ["no_such_layout"]},
    {"behaviours": ["dance"]}, {"seeds": 0}, {"seconds": "soon"}])
def test_2d_refuses(tmp_path, body):
    runner = chase_runs.ChaseRunner(scripts=str(tmp_path), folder=tmp_path)
    code, reply = runner.post("/api/chase/2d", lambda: body)
    assert code == 400 and "error" in reply
    assert not runner.flat.running()


@pytest.fixture
def suite(tmp_path, monkeypatch):
    """A suite script that writes a scored run for each scenario."""
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    (scripts / "gazebo_chase_score.py").write_text(open(SCORE).read())
    (scripts / chase_runs.SUITE).write_text(
        '#!/bin/bash\nOUT=$1; shift\nmkdir -p "$OUT"\n'
        'ALL="one two"\n'
        'for name in "$@"; do\n'
        '  echo "freeing itself" > "$OUT/$name.launch.log"\n'
        '  printf "t,hx,hy,hz,hyaw,hroll,hpitch,tx,ty,tz,gap\\n" > '
        '"$OUT/$name.csv"\n'
        '  for t in 0 1 2 3; do printf "$t,0,0,0.01,0,0,0,0.1,0,0.01,0.03\\n"'
        ' >> "$OUT/$name.csv"; done\n'
        '  echo "3.0 s of sim time in 1.0 s" > "$OUT/$name.log"\n'
        'done\n'
        '[ "$1" = slow ] || exit 0\n'
        'sleep 30\n')
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    ros2 = bin_dir / "ros2"
    ros2.write_text("#!/bin/sh\n")
    ros2.chmod(ros2.stat().st_mode | stat.S_IEXEC)
    monkeypatch.setenv("PATH", f"{bin_dir}:{os.environ['PATH']}")
    runner = chase_runs.ChaseRunner(scripts=str(scripts),
                                    folder=tmp_path / "out")
    yield runner
    runner.stop()


def test_gazebo_suite_scores(suite):
    assert suite.scenarios == ["one", "two"]
    code, body = suite.post("/api/chase/gazebo",
                            lambda: {"scenarios": ["two"], "seconds": 20})
    assert code == 200, body
    wait(suite, "gazebo")
    rows = suite.status()["gazebo"]["results"]
    assert [r["scenario"] for r in rows] == ["two"]
    assert rows[0]["state"] == "done" and rows[0]["freed"] == 1
    assert rows[0]["score"]["first"] == pytest.approx(2.0)
    assert rows[0]["score"]["falls"] == []


def test_gazebo_refuses_and_stops(suite):
    code, body = suite.post("/api/chase/gazebo",
                            lambda: {"scenarios": ["three"]})
    assert code == 400
    suite.scenarios.append("slow")
    code, body = suite.post("/api/chase/gazebo",
                            lambda: {"scenarios": ["slow"]})
    assert code == 200 and body["gazebo"]["running"]
    code, body = suite.post("/api/chase/gazebo", lambda: {})
    assert code == 400  # one at a time
    body = suite.stop("gazebo")
    assert not body["gazebo"]["running"] and body["gazebo"]["stopped"]
