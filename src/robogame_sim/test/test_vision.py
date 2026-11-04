"""The Vision tab: settings, tag positions, the test camera, the routes."""

import json
import threading
import time
import urllib.request
from http.server import ThreadingHTTPServer
from types import SimpleNamespace

import pytest

from robogame_sim import gamemaster, vision


def test_settings_checked():
    settings = vision.clean_settings({"fps": "15", "draw_grid": True},
                                     vision.DEFAULTS)
    assert settings["fps"] == 15 and settings["draw_grid"] is True
    for bad in ({"fps": 0}, {"nope": 1}, {"draw_grid": "yes"},
                {"source": " "}, {"thresh_min": 60, "thresh_max": 53}):
        with pytest.raises(ValueError):
            vision.clean_settings(bad, vision.DEFAULTS)


def test_corners_need_all_four():
    centres = {1: (900, 600), 2: (900, 100), 3: (100, 100)}
    assert vision.corner_points(centres, 40) is None
    centres[4] = (100, 600)
    assert vision.corner_points(centres, 40) == \
        [(940, 640), (940, 60), (60, 60), (60, 640)]


def test_topic_tags_shown_until_stale():
    seen = vision.Vision()
    seen.take_topic(SimpleNamespace(data=json.dumps(
        [{"id": 7, "x": 1, "y": 2}, "junk"])))
    seen.take_topic(SimpleNamespace(data="not json"))
    state = seen.state()
    assert [t["id"] for t in state["tags"]] == [7]
    assert state["tags_from"] == "topic" and not state["arena_found"]
    seen.tags_at -= vision.STALE + 0.1
    assert seen.state()["tags"] == []


def test_lists_cameras(tmp_path):
    for entry, name, index in (("video2", "Arena cam", "0"),
                               ("video3", "Arena cam", "1"),  # metadata
                               ("video0", "Laptop", "0"), ("media0", "x", "")):
        (tmp_path / entry).mkdir()
        (tmp_path / entry / "name").write_text(name + "\n")
        (tmp_path / entry / "index").write_text(index)
    assert vision.list_cameras(str(tmp_path)) == [
        {"source": "0", "name": "Laptop (/dev/video0)"},
        {"source": "2", "name": "Arena cam (/dev/video2)"}]


def test_camera_choices_keep_the_one_in_use(monkeypatch):
    monkeypatch.setattr(vision, "list_cameras",
                        lambda: [{"source": "0", "name": "Laptop"}])
    seen = vision.Vision()
    seen.change({"source": "rtsp://cam/1"})
    status, body = seen.get("/api/vision/cameras")
    assert status == 200 and [c["source"] for c in body["cameras"]] == \
        ["0", "rtsp://cam/1", "test"]


cv2 = pytest.importorskip("cv2")


def square(x, y, size, turned=False):
    """A tag's four px corners, top edge first, as the detector gives."""
    points = [(x, y), (x + size, y), (x + size, y + size), (x, y + size)]
    return points[1:] + points[:1] if turned else points


def test_positions_in_cm():
    settings = dict(vision.DEFAULTS, corner_push=0, arena_cm=100)
    corners = [square(990, 590, 20), square(990, 90, 20), square(90, 90, 20),
               square(90, 590, 20), square(540, 340, 20, turned=True)]
    tags, arena = vision.tags_from_corners(corners, [1, 2, 3, 4, 9],
                                           settings)
    robot = tags[-1]
    assert arena == [(1000, 600), (1000, 100), (100, 100), (100, 600)]
    assert robot["x"] == 550 and robot["y"] == 350
    assert robot["x_cm"] == pytest.approx(50) and \
        robot["y_cm"] == pytest.approx(50)
    assert robot["yaw"] == pytest.approx(90)  # top edge points down, +y


def test_test_camera_finds_its_tags():
    seen = vision.Vision()
    seen.change({"source": "test", "width": 640, "height": 480})
    sent = []
    seen.publish = sent.append
    seen.start()
    try:
        number, jpeg = seen.wait_frame(0, timeout=5)
        assert jpeg[:2] == b"\xff\xd8"  # a JPEG
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not seen.state()["arena_found"]:
            time.sleep(0.05)
        state = seen.state()
        assert state["running"] and state["tags_from"] == "camera"
        assert sorted(t["id"] for t in state["tags"]) == [1, 2, 3, 4, 5]
        robot = next(t for t in state["tags"] if t["id"] == 5)
        assert 0 < robot["x_cm"] < 112 and 0 < robot["y_cm"] < 112
        assert sent and sent[-1] == state["tags"]
        seen.take_topic(SimpleNamespace(data="[]"))  # our own, ignored
        assert seen.state()["tags"]
    finally:
        seen.stop()
    assert not seen.state()["running"]


def test_routes():
    server = ThreadingHTTPServer(("127.0.0.1", 0), None)
    seen = vision.Vision()
    server.RequestHandlerClass = gamemaster.handler(vision=seen)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_address[1]}"

    def post(path, body=None):
        request = urllib.request.Request(
            base + path, json.dumps(body or {}).encode(), method="POST")
        try:
            with urllib.request.urlopen(request) as response:
                return response.status, json.load(response)
        except urllib.error.HTTPError as error:
            return error.code, json.load(error)

    try:
        with urllib.request.urlopen(base + "/vision") as response:
            assert b"Vision" in response.read()
        assert post("/api/vision", {"fps": 500})[0] == 400
        status, state = post("/api/vision", {"source": "test",
                                             "width": 320, "height": 240})
        assert status == 200 and state["settings"]["source"] == "test"
        assert post("/api/vision/start")[1]["running"]
        with urllib.request.urlopen(base + "/vision/stream.mjpg",
                                    timeout=5) as response:
            assert response.headers["Content-Type"].startswith(
                "multipart/x-mixed-replace")
            assert response.read(200).startswith(b"--frame")
        with urllib.request.urlopen(base + "/vision/frame.jpg") as response:
            assert response.headers["Content-Type"] == "image/jpeg"
        assert not post("/api/vision/stop")[1]["running"]
    finally:
        seen.stop()
        server.shutdown()
        server.server_close()
