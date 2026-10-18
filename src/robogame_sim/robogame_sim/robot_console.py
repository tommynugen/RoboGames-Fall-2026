"""Every robot in the arena on one page: who drives it, and where it goes.

    ros2 run robogame_sim robot_console
    ros2 run robogame_sim robot_console --port 8768 --no-browser

Serves a page on http://localhost:8767 with the arena drawn as the arena
editor draws it and every robot on it, found by its
/model/robot_N/odometry, as the sim and the arena camera both publish it.
Only this computer can reach the page.

On the page:

- Bind controllers to robots. That goes through controller_robots, so it
  works in a sim launched with controller:=hotplug.
- Make a robot autonomous. Its controller is taken off it while it is, and
  this node drives it on /robot_N/cmd_vel, stowing its arm first on
  /robot_N/arm_command, as chase_node does. Its planned path is drawn on
  the arena.
- In command mode, send it to a spot, by clicking the arena or typing
  where, or to another robot, one command after another. In chase mode it
  chases a robot, as chase_node does.

The arena is the one in use: in the sim, the layout sim.launch.py built
the world from, which it publishes on /arena/layout (active_arena.py).
With nothing published there, on the real arena, it is the layout last
saved in the arena editor, which maps the modules as they are set up on
the table, or one picked on the page.
"""

import argparse
import json
import math
import os
import queue
import re
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robogame_sim import arena_layout, autopilot
from robogame_sim.active_arena import LATCHED, TOPIC as ARENA_TOPIC
from robogame_sim.autopilot import Autopilot
from robogame_sim.chase import yaw_from_quaternion
from robogame_sim.robot_sdf import COLOURS
from robogame_sim.terrain import HALF_LENGTH, HALF_WIDTH, Terrain

HERE = os.path.dirname(os.path.realpath(__file__))
PAGE = os.path.join(HERE, "robot_console.html")
DRAW = os.path.join(HERE, "arena_draw.js")

ODOMETRY = re.compile(r"^/model/robot_(\d+)/odometry$")
CONTROLLER_STATE = "/controller_robots/state"
CONTROLLER_ASSIGN = "/controller_robots/assign"

# A spot clicked must leave the robot's centre this far inside the floor.
EDGE_MARGIN = 0.04
# A robot whose odometry has gone, as when the sim stops, is left off the
# page after this long, unless it is autonomous.
FORGET_AFTER = 10.0


class Robot:
    """What the console knows of one robot."""

    def __init__(self, robot_id):
        self.id = robot_id
        self.pose = None
        # When the pose was taken, by its stamp, and when it came, by this
        # computer's clock, to tell a stale one.
        self.stamp = None
        self.seen = None
        self.arm = None
        self.autopilot = None
        self.cmd_publisher = None
        self.arm_publisher = None
        self.subscriptions = []
        self.lost = True
        # Other nodes publishing its cmd_vel, such as racing_teleop or
        # chase_node, which would fight the autopilot.
        self.drivers = []
        # The controller taken off it when it was made autonomous, given
        # back when it is not.
        self.released = None

    def fresh(self, timeout):
        return self.seen is not None and time.monotonic() - self.seen < timeout


def cad_model():
    """The Onshape export, installed or in the source folder."""
    for root in (arena_layout.share_dir(), arena_layout.source_dir()):
        if root:
            path = os.path.join(root, "models", arena_layout.CAD_MODEL,
                                "model.sdf")
            if os.path.isfile(path):
                return path

    raise FileNotFoundError(f"no {arena_layout.CAD_MODEL} model")


def newest_layout():
    """(name, path) of the layout last saved in the editor, or None."""
    found = []
    for folder in arena_layout.layout_dirs():
        if not os.path.isdir(folder):
            continue
        for name in os.listdir(folder):
            if name.endswith(".json"):
                path = os.path.join(folder, name)
                found.append((os.path.getmtime(path), name[:-5], path))

    if not found:
        return None
    _, name, path = max(found)
    return name, path


def height_map(terrain):
    """The terrain's heights for the page, in millimetres, for an arena
    with no layout to draw modules from."""
    heights = np.round(terrain.height * 1000).astype(int)
    return {"origin": [float(v) for v in terrain.origin],
            "resolution": float(terrain.resolution),
            "shape": list(heights.shape),
            "mm": heights.T[::-1].ravel().tolist()}


def build_arena(choice):
    """Loads the arena a choice names: the terrain to plan over, and what
    the page draws."""
    name, source, layout = choice["name"], choice["source"], choice["layout"]

    if layout is None:
        terrain = Terrain.from_sdf(cad_model())
        drawing = {"size": arena_layout.SIZE, "modules": None,
                   "heights": height_map(terrain)}
    else:
        layout = arena_layout.normalise(layout)
        terrain = arena_layout.terrain(layout)
        drawing = {"size": layout["size"], "modules": layout["modules"],
                   "heights": None}

    return terrain, {"name": name, "source": source, **drawing}


class RobotConsole(Node):

    def __init__(self):
        super().__init__("robot_console")

        defaults = {
            # Layout to use whatever is published, by name or path, or cad.
            "layout": "",
            "pose_timeout": 0.5,
            "rate": 20.0,
            # As chase_node has them.
            "clearance": 0.03,
            "max_linear_speed": 0.3,
            "max_angular_speed": 1.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in defaults}
        self.timeout = p["pose_timeout"]
        self.chaser_options = {
            "clearance": p["clearance"],
            "max_linear": p["max_linear_speed"],
            "max_angular": p["max_angular_speed"],
        }

        self.robots = {}
        self.controllers = None
        self.published_arena = None
        self.override = p["layout"] or None

        # The arena in use, and the one loading.
        self.arena_key = None
        self.loading = None
        self.terrain = None
        self.arena = None
        self.arena_error = ""
        self.arena_version = 0

        # Changes asked for on the page, made in the executor's thread.
        self.requests = queue.Queue()
        self.lock = threading.Lock()
        self.snapshot = {}

        self.assign_publisher = self.create_publisher(
            String, CONTROLLER_ASSIGN, 10)
        self.create_subscription(
            String, CONTROLLER_STATE, self.on_controllers, LATCHED)
        self.create_subscription(
            String, ARENA_TOPIC, self.on_arena, LATCHED)

        self.create_timer(1.0, self.discover)
        self.create_timer(1.0 / p["rate"], self.tick)
        self.discover()

    # -- robots ----------------------------------------------------------

    def discover(self):
        for topic, _ in self.get_topic_names_and_types():
            match = ODOMETRY.match(topic)
            if match and int(match.group(1)) not in self.robots:
                self.add_robot(int(match.group(1)))

        now = time.monotonic()
        for robot in list(self.robots.values()):
            gone = self.count_publishers(
                f"/model/robot_{robot.id}/odometry") == 0
            if (gone and robot.autopilot is None
                    and (robot.seen is None
                         or now - robot.seen > FORGET_AFTER)):
                self.forget(robot)
                continue

            robot.drivers = sorted({
                info.node_name for info in self.get_publishers_info_by_topic(
                    f"/robot_{robot.id}/cmd_vel")
                if info.node_name != self.get_name()})

        if self.count_publishers(CONTROLLER_STATE) == 0:
            self.controllers = None
        if self.count_publishers(ARENA_TOPIC) == 0:
            self.published_arena = None

        self.check_arena()

    def add_robot(self, robot_id):
        robot = Robot(robot_id)
        self.robots[robot_id] = robot

        def on_pose(message, robot=robot):
            pose = message.pose.pose
            robot.pose = (pose.position.x, pose.position.y,
                          yaw_from_quaternion(pose.orientation))
            robot.stamp = Time.from_msg(message.header.stamp).nanoseconds \
                * 1e-9
            robot.seen = time.monotonic()

        def on_joints(message, robot=robot):
            joints = dict(zip(message.name, message.position))
            if "shoulder_joint" in joints and "elbow_joint" in joints:
                robot.arm = (joints["shoulder_joint"], joints["elbow_joint"])
                if robot.autopilot is not None:
                    robot.autopilot.set_arm(*robot.arm)

        robot.subscriptions = [
            self.create_subscription(
                Odometry, f"/model/robot_{robot_id}/odometry", on_pose, 10),
            self.create_subscription(
                JointState, f"/robot_{robot_id}/joint_states", on_joints, 10),
        ]
        self.get_logger().info(f"Found robot {robot_id}")

    def forget(self, robot):
        for subscription in robot.subscriptions:
            self.destroy_subscription(subscription)
        del self.robots[robot.id]
        self.get_logger().info(f"Robot {robot.id} has gone")

    def controller_of(self, robot_id):
        for controller in self.controllers or []:
            if controller["robot"] == robot_id:
                return controller["address"]
        return None

    def assign(self, address, robot_id):
        self.assign_publisher.publish(String(data=json.dumps(
            {"address": address, "robot": robot_id})))

    def set_autonomous(self, robot, on):
        if on == (robot.autopilot is not None):
            return

        if on:
            robot.autopilot = Autopilot(robot.id, self.terrain,
                                        **self.chaser_options)
            robot.autopilot.stow()
            if robot.arm is not None:
                robot.autopilot.set_arm(*robot.arm)
            robot.lost = True
            if robot.cmd_publisher is None:
                robot.cmd_publisher = self.create_publisher(
                    Twist, f"/robot_{robot.id}/cmd_vel", 10)
                robot.arm_publisher = self.create_publisher(
                    String, f"/robot_{robot.id}/arm_command", 10)
            # Its controller would fight the autopilot, so it lets go.
            robot.released = self.controller_of(robot.id)
            if robot.released is not None:
                self.assign(robot.released, None)
            self.get_logger().info(f"Robot {robot.id} is autonomous")
            return

        robot.autopilot = None
        robot.cmd_publisher.publish(Twist())
        address, robot.released = robot.released, None
        for controller in self.controllers or []:
            if controller["address"] == address and controller["robot"] is None:
                self.assign(address, robot.id)
        self.get_logger().info(f"Robot {robot.id} is driven by hand again")

    # -- the arena -------------------------------------------------------

    def on_arena(self, message):
        try:
            self.published_arena = json.loads(message.data)
        except ValueError:
            self.get_logger().warning("Bad message on /arena/layout")
        self.check_arena()

    def on_controllers(self, message):
        try:
            self.controllers = json.loads(message.data)["controllers"]
        except (ValueError, KeyError):
            self.get_logger().warning(f"Bad message on {CONTROLLER_STATE}")

    def arena_choice(self):
        """The arena to use: the one picked on the page, else the one the
        sim says it runs, else the layout last saved in the editor. With a
        key that changes when it does."""
        if self.override:
            name = self.override
            if name == "cad":
                return {"name": "cad", "source": "picked", "layout": None,
                        "key": ("cad",)}
            path = arena_layout.find_layout(name)
            return {"name": name, "source": "picked", "path": path,
                    "key": ("file", path, os.path.getmtime(path))}

        if self.published_arena is not None:
            arena = self.published_arena
            return {"name": arena.get("name", "?"),
                    "source": arena.get("source", "published"),
                    "layout": arena.get("layout"),
                    "key": ("topic", json.dumps(arena, sort_keys=True))}

        newest = newest_layout()
        if newest is None:
            return {"name": "cad", "source": "editor", "layout": None,
                    "key": ("cad",)}
        name, path = newest
        return {"name": name, "source": "editor", "path": path,
                "key": ("file", path, os.path.getmtime(path))}

    def check_arena(self):
        try:
            choice = self.arena_choice()
        except (OSError, ValueError) as error:
            self.arena_error = str(error)
            return

        if choice["key"] in (self.arena_key, self.loading):
            return

        self.loading = choice["key"]
        threading.Thread(target=self.load_arena, args=(choice,),
                         daemon=True).start()

    def load_arena(self, choice):
        try:
            if "path" in choice:
                choice["layout"] = arena_layout.load(choice["path"])
            terrain, drawing = build_arena(choice)
            error = ""
        except Exception as error_:  # A bad layout file, say.
            terrain = drawing = None
            error = f"Cannot load arena {choice['name']}: {error_}"

        self.requests.put((self.arena_loaded,
                           (choice["key"], terrain, drawing, error), None))

    def arena_loaded(self, key, terrain, drawing, error):
        if key != self.loading:
            return
        self.loading = None
        self.arena_key = key
        self.arena_error = error
        if error:
            self.get_logger().error(error)
            return

        self.terrain = terrain
        self.arena = drawing
        self.arena_version += 1
        for robot in self.robots.values():
            if robot.autopilot is not None:
                robot.autopilot.set_terrain(terrain)
        self.get_logger().info(
            f"Arena {drawing['name']}, from {drawing['source']}")

    # -- driving ---------------------------------------------------------

    def tick(self):
        self.run_requests()

        seen = {
            robot.id: (*robot.pose, robot.stamp, robot.arm)
            for robot in self.robots.values() if robot.fresh(self.timeout)}

        for robot in self.robots.values():
            pilot = robot.autopilot
            if pilot is None:
                continue

            if robot.id not in seen:
                if not robot.lost:
                    robot.cmd_publisher.publish(Twist())
                    self.get_logger().warning(
                        f"Lost robot {robot.id}'s pose, stopping it")
                robot.lost = True
                pilot.pose_lost()
                continue

            if robot.lost:
                # Back after losing its pose, perhaps restarted.
                robot.lost = False
                pilot.stow()

            if self.loading is not None or self.terrain is None:
                pilot.status = "waiting for the arena to load"
                robot.cmd_publisher.publish(Twist())
                continue

            others = {k: v for k, v in seen.items() if k != robot.id}
            linear, angular, arm = pilot.update(
                robot.stamp, *robot.pose, others)
            command = Twist()
            command.linear.x = linear
            command.angular.z = angular
            robot.cmd_publisher.publish(command)
            if arm:
                robot.arm_publisher.publish(String(data=arm))

        self.take_snapshot()

    def run_requests(self):
        answers = []
        while True:
            try:
                function, args, reply = self.requests.get_nowait()
            except queue.Empty:
                break

            try:
                result = (True, function(*args))
            except (ValueError, KeyError, TypeError, OSError) as error:
                result = (False, str(error))

            if reply is not None:
                answers.append((reply, result))

        if answers:
            # So the page, reading the state once answered, sees the change.
            self.take_snapshot()
        for reply, result in answers:
            reply.put(result)

    def call(self, function, *args):
        """Runs function in the executor's thread, for the page."""
        reply = queue.Queue()
        self.requests.put((function, args, reply))
        try:
            return reply.get(timeout=2.0)
        except queue.Empty:
            return False, "the console is not answering"

    def take_snapshot(self):
        robots = []
        for robot in sorted(self.robots.values(), key=lambda r: r.id):
            colour = COLOURS[robot.id % len(COLOURS)]
            robots.append({
                "id": robot.id,
                "colour": "#%02x%02x%02x" % tuple(
                    int(255 * c) for c in colour),
                "pose": robot.pose,
                "fresh": robot.fresh(self.timeout),
                "autonomous": robot.autopilot is not None,
                "autopilot": (robot.autopilot.state()
                              if robot.autopilot is not None else None),
                "drivers": robot.drivers,
                "controller": self.controller_of(robot.id),
            })

        snapshot = {
            # A robot's footprint with its wheels, length then width.
            "body": [2 * HALF_LENGTH, 2 * HALF_WIDTH],
            "robots": robots,
            "controllers": self.controllers,
            "arena": {
                "name": self.arena and self.arena["name"],
                "source": self.arena and self.arena["source"],
                "version": self.arena_version,
                "loading": self.loading is not None,
                "error": self.arena_error,
                "picked": self.override,
                "published": self.published_arena is not None,
            },
        }
        with self.lock:
            self.snapshot = snapshot

    def state(self):
        with self.lock:
            return self.snapshot

    # -- changes from the page -------------------------------------------

    def robot(self, robot_id):
        if robot_id not in self.robots:
            raise KeyError(f"no robot {robot_id}")
        return self.robots[robot_id]

    def change_robot(self, robot_id, asked):
        robot = self.robot(robot_id)

        if "autonomous" in asked:
            self.set_autonomous(robot, bool(asked["autonomous"]))

        pilot = robot.autopilot
        if pilot is None:
            if set(asked) - {"autonomous"}:
                raise ValueError(f"robot {robot_id} is not autonomous")
            return

        if "mode" in asked:
            pilot.set_mode(asked["mode"])
        if "chase" in asked:
            target = asked["chase"]
            if target is not None:
                self.robot(int(target))
            pilot.chase(target)
        if "go" in asked:
            pilot.go(self.command(robot_id, asked["go"]),
                     bool(asked.get("append")))
        if "remove" in asked:
            pilot.remove(int(asked["remove"]))
        if asked.get("stop"):
            pilot.stop()

    def command(self, robot_id, asked):
        if asked.get("kind") == "point":
            command = autopilot.point(asked["x"], asked["y"])
            size = self.arena["size"] if self.arena else arena_layout.SIZE
            limit = size * arena_layout.MODULE / 2 - EDGE_MARGIN
            if max(abs(command["x"]), abs(command["y"])) > limit:
                raise ValueError("that spot is off the arena")
            return command

        if asked.get("kind") == "robot":
            command = autopilot.robot(asked["robot"])
            if command["robot"] == robot_id:
                raise ValueError("a robot cannot go to itself")
            self.robot(command["robot"])
            return command

        raise ValueError("a command is a point or a robot")

    def change_controller(self, address, robot_id):
        if robot_id is not None:
            robot = self.robot(int(robot_id))
            if robot.autopilot is not None:
                raise ValueError(
                    f"robot {robot_id} is autonomous, make it manual first")
            robot_id = robot.id
        if not any(c["address"] == address for c in self.controllers or []):
            raise KeyError(f"no controller {address}")
        self.assign(address, robot_id)

    def pick_arena(self, name):
        if name is not None and name != "cad":
            arena_layout.find_layout(name)
        self.override = name or None
        self.check_arena()

    def arena_drawing(self):
        return self.arena

    def stop_all(self):
        for robot in self.robots.values():
            if robot.autopilot is not None:
                robot.cmd_publisher.publish(Twist())


def handler(console):

    class Handler(BaseHTTPRequestHandler):
        server_version = "robot_console"

        def log_message(self, format, *args):
            pass

        def reply(self, status, body, kind="application/json"):
            data = body if isinstance(body, bytes) else \
                json.dumps(body).encode()
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def file(self, path, kind):
            with open(path, "rb") as file:
                self.reply(200, file.read(), kind)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.file(PAGE, "text/html; charset=utf-8")
            elif self.path == "/arena_draw.js":
                self.file(DRAW, "text/javascript; charset=utf-8")
            elif self.path == "/api/state":
                self.reply(200, console.state())
            elif self.path == "/api/arena":
                ok, arena = console.call(console.arena_drawing)
                self.reply(200 if ok and arena else 503,
                           arena if ok and arena
                           else {"error": "no arena loaded yet"})
            elif self.path == "/api/modules":
                self.reply(200, arena_layout.modules_info())
            elif self.path == "/api/layouts":
                self.reply(200, {"layouts": arena_layout.list_layouts()})
            else:
                self.reply(404, {"error": "not found"})

        def do_POST(self):
            try:
                length = int(self.headers.get("Content-Length", 0))
                asked = json.loads(self.rfile.read(length) or b"{}")
            except ValueError:
                return self.reply(400, {"error": "bad JSON"})

            path = unquote(self.path)
            match = re.match(r"^/api/robots/(\d+)$", path)
            if match:
                ok, result = console.call(
                    console.change_robot, int(match.group(1)), asked)
            elif path == "/api/controllers":
                ok, result = console.call(
                    console.change_controller, asked.get("address"),
                    asked.get("robot"))
            elif path == "/api/arena":
                ok, result = console.call(
                    console.pick_arena, asked.get("layout"))
            else:
                return self.reply(404, {"error": "not found"})

            self.reply(200 if ok else 400,
                       {"ok": True} if ok else {"error": result})

    return Handler


def main(args=None):
    parser = argparse.ArgumentParser(description="Robot console")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--no-browser", action="store_true",
                        help="do not open a browser window")
    # ros2 run passes --ros-args on, which rclpy takes.
    options, _ = parser.parse_known_args(args)

    rclpy.init(args=args)
    console = RobotConsole()

    server = ThreadingHTTPServer(("127.0.0.1", options.port),
                                 handler(console))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = f"http://localhost:{options.port}/"
    console.get_logger().info(f"Robot console on {url}. Ctrl-C to stop.")

    if not options.no_browser:
        threading.Timer(0.5, webbrowser.open, [url]).start()

    try:
        rclpy.spin(console)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        if rclpy.ok():
            # The firmware's 750 ms timeout stops them otherwise.
            console.stop_all()
        console.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
