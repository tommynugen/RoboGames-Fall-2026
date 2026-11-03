"""The Console tab's log lines: sources, the sim log, the app's routes."""

import json
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

from robogame_sim import app_log, gamemaster


def test_lines_after():
    book = app_log.LogBook(keep=3)
    book.add("editor", "one\ntwo")
    book.add("sim", "three", "warn")
    book.add("app", "four")
    data = book.since(0)
    assert [line["text"] for line in data["lines"]] == \
        ["two", "three", "four"]  # only the last 3 kept
    assert data["last"] == 4
    assert [line["n"] for line in book.since(3)["lines"]] == [4]
    assert book.get("/api/logs?after=x")[1]["last"] == 4
    assert book.get("/api/other") is None


def test_follows_a_file(tmp_path):
    path = tmp_path / "sim.log"
    path.write_text("before the app\n")
    book = app_log.LogBook()
    book.follow("sim", str(path))  # from the end
    assert book.since()["lines"] == []
    with open(path, "a") as file:
        file.write("[ERROR] [gz]: no world\nhalf")
    lines = book.since()["lines"]
    assert [(line["text"], line["level"]) for line in lines] == \
        [("[ERROR] [gz]: no world", "error")]
    with open(path, "a") as file:
        file.write(" a line\n")
    assert book.since(1)["lines"][0]["text"] == "half a line"
    path.write_text("new run\n")  # shorter: started again
    assert book.since(2)["lines"][0]["text"] == "new run"


def test_rosout_sources():
    assert app_log.rosout_source("robot_console", "robot_console") == \
        "console"
    assert app_log.rosout_source("/controller_robots", "robot_console") == \
        "teleop"
    assert app_log.rosout_source("racing_teleop.sub", "x") == "teleop"
    assert app_log.rosout_source("chase_node", "robot_console") is None


def test_app_logs_requests():
    book = app_log.LogBook()
    server = ThreadingHTTPServer(("127.0.0.1", 0),
                                 gamemaster.handler(book=book))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        request = urllib.request.Request(
            base + "/api/check", data=b'{"modules": []}', method="POST")
        urllib.request.urlopen(request).read()  # quiet: not logged
        try:
            urllib.request.urlopen(base + "/api/nothing")
        except urllib.error.HTTPError as error:
            assert error.code == 404
        with urllib.request.urlopen(base + "/api/logs?after=0") as reply:
            lines = json.load(reply)["lines"]
        assert [(line["source"], line["level"]) for line in lines] == \
            [("app", "error")]
        assert "GET /api/nothing: 404" in lines[0]["text"]
        with urllib.request.urlopen(base + "/logs") as reply:
            assert b"Gazebo sim" in reply.read()
    finally:
        server.shutdown()
        server.server_close()
