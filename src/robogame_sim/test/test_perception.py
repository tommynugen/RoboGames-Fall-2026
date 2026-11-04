"""The camera's robot tags and arena_perception's nodes in the Gamemaster:
tag places to arena poses, the tag map, odometry out with a stand-in ROS,
controllers.yaml, and the nodes started with a stand-in for ros2."""

import json
import math
import stat
import sys
import threading
import time
import urllib.request
from http.server import ThreadingHTTPServer
from types import ModuleType, SimpleNamespace

import pytest

from robogame_sim import app_log, gamemaster, perception


def test_camera_pose_centres_the_arena_and_turns_y_up():
    side = 120.0
    # The middle of the picture is the middle of the arena.
    x, y, yaw = perception.camera_pose(
        {"id": 5, "x_cm": 60, "y_cm": 60, "yaw": 0}, side)
    assert (x, y) == pytest.approx((0, 0))
    # A tag upright on the picture (top edge along +x) faces up it: +y.
    assert yaw == pytest.approx(math.pi / 2)
    # Top left of the picture is -x, +y on the arena.
    x, y, _ = perception.camera_pose({"x_cm": 0, "y_cm": 0}, side)
    assert (x, y) == pytest.approx((-0.6, 0.6))
    # Top edge pointing down the picture: the tag faces +x.
    _, _, yaw = perception.camera_pose(
        {"x_cm": 10, "y_cm": 10, "yaw": 90}, side)
    assert yaw == pytest.approx(0)
    _, _, yaw = perception.camera_pose(
        {"x_cm": 10, "y_cm": 10, "yaw": 90}, side, turn=90)
    assert yaw == pytest.approx(math.pi / 2)
    assert perception.camera_pose({"id": 5, "x": 3, "y": 4}, side) is None


def test_tag_map_checked_saved_and_one_tag_per_robot(tmp_path):
    path = tmp_path / "share" / "tag_robots.json"
    tags = perception.TagRobots(path=str(path), side=lambda: 100.0)
    with pytest.raises(ValueError):
        tags.set(2, 1)  # a corner
    with pytest.raises(ValueError):
        tags.set(5, -1)
    tags.set(5, 1)
    tags.set(6, 2, 45)
    tags.set(7, 1)  # robot 1's tag is now 7
    assert tags.tag_of(1) == 7 and tags.tag_of(2) == 6
    again = perception.TagRobots(path=str(path))
    assert again.map == {6: {"robot": 2, "turn": 45.0},
                         7: {"robot": 1, "turn": 0.0}}
    tags.set(6, None)
    assert tags.tag_of(2) is None
    poses = tags.poses([{"id": 7, "x_cm": 50, "y_cm": 50, "yaw": 0},
                        {"id": 9, "x_cm": 1, "y_cm": 1}, {"id": 3}])
    assert list(poses) == [1]


class Odometry:
    def __init__(self):
        point = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        turn = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.child_frame_id = ""
        self.pose = SimpleNamespace(pose=SimpleNamespace(
            position=point, orientation=turn))


class FakeNode:
    def __init__(self):
        self.sent = {}
        self.others = set()

    def create_publisher(self, kind, topic, depth):
        sent = self.sent.setdefault(topic, [])
        return SimpleNamespace(publish=sent.append, topic=topic)

    def destroy_publisher(self, publisher):
        pass

    def count_publishers(self, topic):
        return (topic in self.sent) + (topic in self.others)

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: "now"))


def test_odometry_published_for_mapped_tags_unless_the_sim_has_it(
        tmp_path, monkeypatch):
    nav = ModuleType("nav_msgs")
    nav.msg = ModuleType("nav_msgs.msg")
    nav.msg.Odometry = Odometry
    monkeypatch.setitem(sys.modules, "nav_msgs", nav)
    monkeypatch.setitem(sys.modules, "nav_msgs.msg", nav.msg)

    tags = perception.TagRobots(path=str(tmp_path / "t.json"),
                                side=lambda: 100.0)
    tags.set(5, 1)
    tags.set(6, 2)
    node = FakeNode()
    tags.node = node
    message = SimpleNamespace(data=json.dumps([
        {"id": 5, "x_cm": 75, "y_cm": 25, "yaw": 90},
        {"id": 1, "x_cm": 0, "y_cm": 0}]))
    tags.on_tags(message)
    [odometry] = node.sent["/model/robot_1/odometry"]
    pose = odometry.pose.pose
    assert (pose.position.x, pose.position.y) == pytest.approx((0.25, 0.25))
    assert pose.orientation.w == pytest.approx(1.0)  # facing +x
    assert odometry.header.frame_id == "arena"
    state = tags.state()
    assert state["robots"]["1"]["state"] == "seen"
    assert state["robots"]["2"]["state"] == "not seen"

    # The sim publishes robot 1's odometry too: the tag is left alone.
    node.others.add("/model/robot_1/odometry")
    tags.check_topics()
    tags.on_tags(message)
    assert len(node.sent["/model/robot_1/odometry"]) == 1
    assert tags.state()["robots"]["1"]["state"] == "sim"
    tags.on_tags(SimpleNamespace(data="not json"))


def test_controllers_file_read_checked_and_written(tmp_path):
    path = tmp_path / "controllers.yaml"
    path.write_text('robots:\n  0: "a0:5a:5e:ee:f8:fa"\n'
                    '  1: "D0:BC:C1:8B:EC:62"\n')
    nodes = perception.Perception(path=str(path), book=app_log.LogBook())
    assert nodes.controllers()["robots"] == {
        "0": "A0:5A:5E:EE:F8:FA", "1": "D0:BC:C1:8B:EC:62"}
    code, body = nodes.post("/api/perception/controllers", lambda: {
        "robots": {"2": "14:3a:9a:97:b8:e9", "0": "A0:5A:5E:EE:F8:FA"}})
    assert code == 200
    assert body["controllers"]["robots"] == {
        "0": "A0:5A:5E:EE:F8:FA", "2": "14:3A:9A:97:B8:E9"}
    text = path.read_text()
    assert text.startswith("# Which PS4 controller") and '  2: "14:3A' in text
    for bad in ({"1": "nope"}, {"1": "A0:5A:5E:EE:F8:FA",
                                "2": "A0:5A:5E:EE:F8:FA"}, {"-1": "x"}):
        code, _ = nodes.post("/api/perception/controllers",
                             lambda bad=bad: {"robots": bad})
        assert code == 400
    assert perception.read_controllers(path) == {
        0: "A0:5A:5E:EE:F8:FA", 2: "14:3A:9A:97:B8:E9"}


FAKE = """#!/bin/sh
echo "args $@"
echo "[INFO] [1762236000.1] [autonomy_node]: started"
echo "Traceback (most recent call last):"
exec sleep 30
"""


def test_nodes_started_and_stopped(tmp_path):
    fake = tmp_path / "ros2"
    fake.write_text(FAKE)
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    book = app_log.LogBook()
    nodes = perception.Perception(launcher=str(fake), book=book,
                                  path=str(tmp_path / "c.yaml"))
    code, body = nodes.post("/api/perception/autonomy_node/start",
                            lambda: {})
    assert code == 200 and body["nodes"]["autonomy_node"]["running"]
    assert nodes.post("/api/perception/autonomy_node/start",
                      lambda: {})[0] == 400
    code, _ = nodes.post("/api/perception/assign", lambda: {
        "robots": [1, 2]})
    assert code == 200
    assert nodes.post("/api/perception/assign", lambda: {
        "robots": [1, 1]})[0] == 400
    assert nodes.post("/api/perception/nope/start", lambda: {})[0] == 404
    deadline = time.time() + 5
    while time.time() < deadline:
        status = nodes.status()["nodes"]
        if (len(status["autonomy_node"]["lines"]) >= 3
                and status["assign_controllers"]["lines"]):
            break
        time.sleep(0.05)
    assert status["autonomy_node"]["lines"][0] == \
        "args run arena_perception autonomy_node"
    assert status["assign_controllers"]["lines"][0] == \
        "args run arena_perception assign_controllers 1 2"
    texts = [line["text"] for line in book.lines]
    # Its ROS log comes from /rosout instead, the rest from here.
    assert not any("[autonomy_node]: started" in t for t in texts)
    assert any("Traceback" in t for t in texts)
    nodes.stop()
    assert not nodes.status()["nodes"]["autonomy_node"]["running"]
    assert nodes.post("/api/perception/goal", lambda: {"x": 1, "y": 2})[0] \
        == 400  # no ROS


def test_routes_over_http(tmp_path):
    tags = perception.TagRobots(path=str(tmp_path / "t.json"))
    nodes = perception.Perception(path=str(tmp_path / "c.yaml"))
    server = ThreadingHTTPServer(("127.0.0.1", 0), gamemaster.handler(
        tags=tags, nodes=nodes))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_address[1]}"

    def post(path, body):
        request = urllib.request.Request(
            base + path, data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request) as reply:
            return json.load(reply)
    try:
        assert post("/api/tag_robots", {"tag": 5, "robot": 3})["map"] == {
            "5": {"robot": 3, "turn": 0.0}}
        with urllib.request.urlopen(base + "/api/tag_robots") as reply:
            assert json.load(reply)["robots"]["3"]["tag"] == 5
        with urllib.request.urlopen(base + "/api/perception") as reply:
            assert set(json.load(reply)["nodes"]) == {
                "autonomy_node", "gazebo_viz_node", "assign_controllers"}
    finally:
        server.shutdown()
        server.server_close()
