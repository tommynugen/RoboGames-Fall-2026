"""The robot console's node and page API, against a stand-in for the sim
and for controller_robots, on a ROS domain of its own."""

import json
import os
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from robogame_sim import active_arena  # noqa: E402
from robogame_sim.controller_robots import LATCHED  # noqa: E402
from robogame_sim.robot_console import (  # noqa: E402
    RobotConsole,
    handler,
    height_map,
    newest_layout,
)
from robogame_sim.terrain import Terrain  # noqa: E402

CAD = os.path.join(os.path.dirname(__file__), "..", "models",
                   "robogame_arena_cad", "model.sdf")


class FakeArena(Node):
    """Two robots standing still, the arena the sim says it runs, and a
    controller_robots with one controller on robot 1."""

    def __init__(self):
        super().__init__("fake_arena")
        self.poses = {1: (-0.45, -0.45), 2: (0.45, -0.45)}
        self.twists = []
        self.arm = []
        self.assigned = []
        self.controllers = [{"address": "AA:BB", "name": "DualSense",
                             "robot": 1, "connected": True,
                             "virtual": False}]

        self.odometry = {
            n: self.create_publisher(Odometry, f"/model/robot_{n}/odometry",
                                     10) for n in self.poses}
        self.joints = self.create_publisher(
            JointState, "/robot_1/joint_states", 10)
        self.arena = self.create_publisher(
            String, active_arena.TOPIC, LATCHED)
        self.arena.publish(String(data=active_arena.message(
            "default", "gazebo")))
        self.state = self.create_publisher(
            String, "/controller_robots/state", LATCHED)
        self.publish_controllers()

        self.create_subscription(Twist, "/robot_1/cmd_vel",
                                 self.twists.append, 10)
        self.create_subscription(String, "/robot_1/arm_command",
                                 lambda m: self.arm.append(m.data), 10)
        self.create_subscription(String, "/controller_robots/assign",
                                 self.on_assign, 10)
        self.create_timer(0.05, self.tick)

    def publish_controllers(self):
        self.state.publish(String(data=json.dumps(
            {"controllers": self.controllers, "robots": [1, 2]})))

    def on_assign(self, message):
        asked = json.loads(message.data)
        self.assigned.append(asked)
        self.controllers[0]["robot"] = asked["robot"]
        self.publish_controllers()

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        for n, (x, y) in self.poses.items():
            message = Odometry()
            message.header.stamp = stamp
            message.pose.pose.position.x = x
            message.pose.pose.position.y = y
            message.pose.pose.orientation.w = 1.0
            self.odometry[n].publish(message)
        # Robot 1's arm is up already, so stowing it is quick.
        self.joints.publish(JointState(
            name=["shoulder_joint", "elbow_joint"], position=[1.6, 1.6]))


@pytest.fixture(scope="module")
def console():
    os.environ["ROS_DOMAIN_ID"] = "87"
    rclpy.init(args=["--ros-args", "-p", "pose_timeout:=1.0"])
    node = RobotConsole()
    fake = FakeArena()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(fake)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    server = ThreadingHTTPServer(("127.0.0.1", 0), handler(node))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{server.server_address[1]}"

    yield url, node, fake

    server.shutdown()
    executor.shutdown()
    node.destroy_node()
    fake.destroy_node()
    rclpy.try_shutdown()


def call(url, method="GET", body=None):
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(
        url, data=data, method=method,
        headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read())


def wait_for(check, seconds=10.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        result = check()
        if result:
            return result
        time.sleep(0.05)
    raise AssertionError("timed out")


def robot_state(url, n):
    state = call(url + "/api/state")[1]
    return next((r for r in state.get("robots", []) if r["id"] == n), None)


def test_console_finds_robots_and_the_sims_arena(console):
    url, _, _ = console

    def found():
        state = call(url + "/api/state")[1]
        ids = [r["id"] for r in state.get("robots", [])]
        return ids == [1, 2] and state["arena"]["version"] > 0 and state
    state = wait_for(found)
    assert state["arena"]["name"] == "default"
    assert state["arena"]["source"] == "gazebo"
    assert state["controllers"][0]["robot"] == 1
    assert robot_state(url, 1)["controller"] == "AA:BB"

    status, arena = call(url + "/api/arena")
    assert status == 200 and arena["size"] == 12 and arena["modules"]


def test_page_and_drawing_code_are_served(console):
    url, _, _ = console
    with urllib.request.urlopen(url + "/") as response:
        assert b"Robot console" in response.read()
    with urllib.request.urlopen(url + "/arena_draw.js") as response:
        assert b"function drawModule" in response.read()


def test_an_autonomous_robot_takes_commands_and_lets_its_controller_go(
        console):
    url, _, fake = console
    wait_for(lambda: robot_state(url, 1) and robot_state(url, 1)["fresh"])

    assert call(url + "/api/robots/1", "POST", {"autonomous": True})[0] == 200
    wait_for(lambda: fake.assigned)
    assert fake.assigned[-1] == {"address": "AA:BB", "robot": None}
    wait_for(lambda: fake.arm)
    assert set(fake.arm) <= {"+", "X"}

    # Nobody can drive it by hand while it is autonomous.
    status, body = call(url + "/api/controllers", "POST",
                        {"address": "AA:BB", "robot": 1})
    assert status == 400 and "autonomous" in body["error"]

    status, body = call(url + "/api/robots/1", "POST",
                        {"go": {"kind": "point", "x": 2.0, "y": 0.0}})
    assert status == 400 and "off the arena" in body["error"]
    status, _ = call(url + "/api/robots/1", "POST",
                     {"go": {"kind": "point", "x": 0.3, "y": -0.45}})
    assert status == 200
    status, _ = call(url + "/api/robots/1", "POST",
                     {"go": {"kind": "robot", "robot": 2}, "append": True})
    assert status == 200

    def driving():
        robot = robot_state(url, 1)
        pilot = robot["autopilot"]
        return (pilot["path"] and pilot["status"].startswith("to 0.30")
                and robot)
    robot = wait_for(driving)
    assert [c["kind"] for c in robot["autopilot"]["queue"]] == [
        "point", "robot"]
    assert robot["autopilot"]["path"][-1] == pytest.approx([0.3, -0.45],
                                                           abs=0.02)
    wait_for(lambda: any(t.linear.x > 0.05 or abs(t.angular.z) > 0.05
                         for t in fake.twists))


def test_chase_mode_goes_after_the_robot_picked(console):
    url, _, _ = console
    assert call(url + "/api/robots/1", "POST",
                {"mode": "chase", "chase": 2})[0] == 200
    wait_for(lambda: robot_state(url, 1)["autopilot"]["status"].startswith(
        "chasing robot 2"))
    assert call(url + "/api/robots/1", "POST", {"chase": 1})[0] == 400
    assert call(url + "/api/robots/1", "POST", {"chase": 9})[0] == 400


def test_back_to_manual_gives_the_controller_back(console):
    url, _, fake = console
    fake.twists.clear()
    assert call(url + "/api/robots/1", "POST",
                {"autonomous": False})[0] == 200
    wait_for(lambda: fake.assigned[-1] == {"address": "AA:BB", "robot": 1})
    wait_for(lambda: fake.twists)
    assert fake.twists[-1].linear.x == 0.0
    assert robot_state(url, 1)["autopilot"] is None
    status, body = call(url + "/api/robots/1", "POST", {"mode": "chase"})
    assert status == 400 and "not autonomous" in body["error"]


def test_an_arena_can_be_picked_over_the_sims(console):
    url, _, _ = console
    assert call(url + "/api/arena", "POST", {"layout": "nothere"})[0] == 400
    assert call(url + "/api/arena", "POST", {"layout": "cad"})[0] == 200
    wait_for(lambda: call(url + "/api/state")[1]["arena"]["name"] == "cad"
             and not call(url + "/api/state")[1]["arena"]["loading"], 30.0)
    arena = call(url + "/api/arena")[1]
    assert arena["modules"] is None and arena["heights"]["mm"]

    assert call(url + "/api/arena", "POST", {"layout": None})[0] == 200
    wait_for(lambda: call(url + "/api/state")[1]["arena"]["source"]
             == "gazebo", 30.0)


def test_newest_layout_is_the_one_saved_last(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBOGAME_LAYOUTS", str(tmp_path))
    monkeypatch.setattr("robogame_sim.arena_layout.layout_dirs",
                        lambda: [str(tmp_path)])
    assert newest_layout() is None
    for i, name in enumerate(["old", "new"]):
        path = tmp_path / f"{name}.json"
        path.write_text('{"size": 12, "modules": []}')
        os.utime(path, (1000 + i, 1000 + i))
    assert newest_layout() == ("new", str(tmp_path / "new.json"))


def test_height_map_puts_the_top_row_first():
    terrain = Terrain.from_sdf(CAD)
    heights = height_map(terrain)
    nx, ny = heights["shape"]
    assert len(heights["mm"]) == nx * ny
    # Row 0 of the page's image is the arena's +y edge.
    top_left = round(terrain.height[0, ny - 1] * 1000)
    assert heights["mm"][0] == top_left
