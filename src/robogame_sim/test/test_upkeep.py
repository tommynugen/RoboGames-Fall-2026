"""The Upkeep tab's jobs, run on a stand-in checkout."""

import json
import stat
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest

from robogame_sim import app_log, gamemaster, upkeep


def wait(runner):
    for _ in range(100):
        if not runner.running():
            return runner.status()
        time.sleep(0.05)
    raise AssertionError("the job did not end")


@pytest.fixture
def runner(tmp_path, monkeypatch):
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    (scripts / "export_onshape_arena.py").write_text("print('exported')\n")
    (scripts / "sync_arena_perception.sh").write_text(
        "echo copied from \"${1:-default}\"\n")
    colcon = tmp_path / "colcon"
    colcon.write_text("#!/bin/sh\necho built \"$@\"\nexec sleep 30\n")
    colcon.chmod(colcon.stat().st_mode | stat.S_IEXEC)
    monkeypatch.setattr(upkeep, "KEYS", str(tmp_path / "keys.env"))
    monkeypatch.delenv("ONSHAPE_ACCESS_KEY", raising=False)
    jobs = upkeep.UpkeepRunner(root=str(tmp_path), colcon=str(colcon))
    jobs.log = str(tmp_path / "upkeep.log")
    yield jobs
    jobs.stop()


def test_finds_this_checkout():
    root = upkeep.find_root()
    assert root and (upkeep.os.path.isdir(upkeep.os.path.join(root, "src")))


def test_onshape_needs_keys(runner, tmp_path):
    code, body = runner.post("/api/upkeep", lambda: {"job": "onshape"})
    assert code == 400 and "keys" in body["error"]
    (tmp_path / "keys.env").write_text("ONSHAPE_ACCESS_KEY=a\n")
    assert runner.status()["keys"]
    runner.start("onshape")
    status = wait(runner)
    assert status["exit"] == 0 and "exported" in status["tail"]


def test_perception_path(runner, tmp_path):
    with pytest.raises(ValueError, match="arena_perception"):
        runner.start("perception", str(tmp_path / "nowhere"))
    checkout = tmp_path / "ROBOGAME-2.0"
    (checkout / "src" / "arena_perception").mkdir(parents=True)
    runner.start("perception", str(checkout))
    assert f"copied from {checkout}" in wait(runner)["tail"]
    runner.start("perception")
    assert "copied from default" in wait(runner)["tail"]


def test_one_at_a_time_and_stop(runner):
    status = runner.start("build")
    assert status["running"] and "--symlink-install" in status["command"]
    with pytest.raises(ValueError, match="still running"):
        runner.start("perception")
    status = runner.stop()
    assert not status["running"] and status["stopped"]
    code, body = runner.post("/api/upkeep", lambda: {"job": "nope"})
    assert code == 400


def test_no_checkout():
    jobs = upkeep.UpkeepRunner(root=None)
    jobs.root = None
    with pytest.raises(ValueError, match="checkout"):
        jobs.start("build")
    assert jobs.status()["cad"] is None


def test_app_routes(runner):
    book = app_log.LogBook()
    server = ThreadingHTTPServer(
        ("127.0.0.1", 0), gamemaster.handler(book=book, upkeep=runner))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        with urllib.request.urlopen(base + "/upkeep") as reply:
            assert b"Export from Onshape" in reply.read()
        request = urllib.request.Request(
            base + "/api/upkeep", data=b'{"job": "perception"}',
            method="POST")
        assert json.load(urllib.request.urlopen(request))["job"] == \
            "perception"
        wait(runner)
        with urllib.request.urlopen(base + "/api/app") as reply:
            app = json.load(reply)
        assert app["upkeep"]["job"] == "perception"
        assert app["upkeep"]["exit"] == 0
        texts = [line["text"] for line in book.since()["lines"]
                 if line["source"] == "upkeep"]
        assert any("copied from default" in text for text in texts)
    finally:
        server.shutdown()
        server.server_close()
