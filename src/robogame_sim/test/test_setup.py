"""The Gamemaster's Setup tab: its page and the camera on the status bar."""
import json
import threading
import urllib.request
from http.server import ThreadingHTTPServer

from robogame_sim import app_log, gamemaster


def test_setup_tab_and_camera_status():
    server = ThreadingHTTPServer(
        ("127.0.0.1", 0), gamemaster.handler(book=app_log.LogBook()))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        with urllib.request.urlopen(base + "/setup") as reply:
            page = reply.read()
        for step in (b"Build", b"BLE bridge", b"Pair controllers",
                     b"Camera and arena size", b"Map robot tags",
                     b"Assign controllers"):
            assert step in page
        with urllib.request.urlopen(base + "/") as reply:
            assert b'data-tab="setup"' in reply.read()
        with urllib.request.urlopen(base + "/api/app") as reply:
            camera = json.load(reply)["camera"]
        assert camera["running"] is False and camera["arena_found"] is False
        with urllib.request.urlopen(base + "/api/setup") as reply:
            checks = json.load(reply)
        # No console without ROS: its parts are null, the rest are there.
        assert checks["robots"] is None and checks["pairing"] is None
        assert "settings" in checks["vision"] and "map" in checks["tags"]
    finally:
        server.shutdown()
    assert gamemaster.TABS[0] == "setup"
