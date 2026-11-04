"""The real arena's camera robots and arena_perception's nodes, for the
Gamemaster.

Robot tags. The arena camera sees one AprilTag on each real robot, on
/arena/tags (from the Vision tab or apriltag_node). TagRobots says which
tag is on which robot and publishes each mapped robot's place, from its
tag, on /model/robot_N/odometry, the topic the sim's robots use. The Robot
console finds robots and their poses there, so a real robot the BLE bridge
has found can be made autonomous once its tag is mapped. The map is kept
in ~/.local/share/robogame_sim/tag_robots.json.

Places: the camera gives cm from the arena's top left corner (corner tag
3), x to the right and y down the picture. The sim's arena has its middle
at 0, 0 in m, with y up the picture, so

    x = (x_cm - side / 2) / 100        y = (side / 2 - y_cm) / 100

with side the Vision tab's arena side. A robot faces the way its tag's top
points; turn (degrees, anticlockwise) is for a tag stuck on another way.
A robot whose odometry topic another node already publishes, such as the
sim's, is left to it.

arena_perception's nodes, started and stopped here as from a terminal:
autonomy_node (drives robot 1 to the goal sent on /robot_1/goal, from its
/model/robot_1/odometry) and gazebo_viz_node (draws the tags as markers),
and assign_controllers, which binds a PS4 controller to each robot by a
button press on each, and controllers.yaml, the file it writes, which can
be read and changed here too. Changes there take effect when the bridge's
robot_control.launch.py starts again.

Routes, served by gamemaster.py:

- GET /api/tag_robots: {"map", "robots", "side_cm"}. POST /api/tag_robots
  {"tag", "robot", "turn"} maps a tag (robot null takes it off).
- GET /api/perception: each node's state and controllers.yaml. POST
  /api/perception/<node>/start and /stop, /api/perception/goal {"x", "y"},
  /api/perception/controllers {"robots": {N: address}} and
  /api/perception/assign {"robots": [N, ...]}.
"""

import json
import math
import os
import re
import shutil
import signal
import subprocess
import threading
import time

TAGS_TOPIC = "/arena/tags"
CORNER_TAGS = (1, 2, 3, 4)
FILE_NAME = "tag_robots.json"
# A mapped robot whose tag was not seen for this long shows as not seen.
STALE = 1.0
STOP_WAIT = 5.0  # s, from Ctrl-C to SIGKILL
ADDRESS = re.compile(r"^[0-9A-F]{2}(:[0-9A-F]{2}){5}$")
# The ROS log lines a node prints, which the Logs tab has from /rosout.
ROS_LINE = re.compile(r"\[(DEBUG|INFO|WARN|ERROR|FATAL)\] \[\d")


def tag_file():
    return os.path.join(os.path.expanduser("~/.local/share/robogame_sim"),
                        FILE_NAME)


def camera_pose(tag, side_cm, turn=0.0):
    """(x, y, yaw) in m and radians on the sim's arena, from a tag in
    apriltag_node's form with the Vision tab's yaw, or None without its
    place in cm."""
    if "x_cm" not in tag or "y_cm" not in tag:
        return None
    x = (float(tag["x_cm"]) - side_cm / 2) / 100
    y = (side_cm / 2 - float(tag["y_cm"])) / 100
    # yaw is the way the tag's top edge runs on the picture, y down; its
    # top points 90 degrees from that, and y up turns it round.
    yaw = math.radians(90.0 - float(tag.get("yaw", 0.0)) + turn)
    return x, y, math.atan2(math.sin(yaw), math.cos(yaw))


def clean_entry(tag, robot, turn):
    """(tag, robot or None, turn), checked."""
    tag = int(tag)
    if tag < 0 or tag in CORNER_TAGS:
        raise ValueError(f"tag {tag} cannot be a robot's: 1 to 4 are the "
                         "arena's corners")
    if robot is not None and robot != "":
        robot = int(robot)
        if robot < 0:
            raise ValueError("robot numbers start at 0")
    else:
        robot = None
    turn = float(turn or 0.0)
    if not -360 <= turn <= 360:
        raise ValueError("turn is in degrees, -360 to 360")
    return tag, robot, turn


class TagRobots:
    """Which tag is on which robot, and their poses from the camera."""

    def __init__(self, path=None, side=None, book=None):
        self.path = path or tag_file()
        # The arena's side in cm, from the Vision tab's settings.
        self.side = side or (lambda: 112.0)
        self.book = book
        self.lock = threading.Lock()
        self.map = {}  # tag: {"robot", "turn"}
        self.seen = {}  # robot: (x, y, yaw, monotonic time)
        self.blocked = set()  # robots whose odometry another node has
        self.node = None
        self.publishers = {}
        self.load()

    def say(self, text, level="info"):
        if self.book is not None:
            self.book.add("vision", text, level)

    def load(self):
        try:
            with open(self.path) as f:
                raw = json.load(f)
            self.map = {int(k): {"robot": int(v["robot"]),
                                 "turn": float(v.get("turn", 0.0))}
                        for k, v in raw.items()}
        except FileNotFoundError:
            self.map = {}
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.map = {}
            self.say(f"Could not read {self.path}: {error}", "warn")

    def save(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        with open(self.path, "w") as f:
            json.dump({str(k): v for k, v in sorted(self.map.items())}, f,
                      indent=1)

    def set(self, tag, robot, turn=0.0):
        tag, robot, turn = clean_entry(tag, robot, turn)
        with self.lock:
            if robot is None:
                self.map.pop(tag, None)
            else:
                for other, entry in list(self.map.items()):
                    if entry["robot"] == robot and other != tag:
                        del self.map[other]  # one tag per robot
                self.map[tag] = {"robot": robot, "turn": turn}
            self.save()
        self.say(f"Tag {tag} is on robot {robot}" if robot is not None
                 else f"Tag {tag} is on no robot")
        return self.state()

    def tag_of(self, robot):
        with self.lock:
            for tag, entry in self.map.items():
                if entry["robot"] == robot:
                    return tag
        return None

    def poses(self, tags):
        """{robot: (x, y, yaw)} for the mapped tags among tags."""
        side = float(self.side())
        out = {}
        with self.lock:
            for tag in tags:
                entry = self.map.get(tag.get("id")) if isinstance(
                    tag, dict) else None
                if entry is None:
                    continue
                pose = camera_pose(tag, side, entry["turn"])
                if pose is not None:
                    out[entry["robot"]] = pose
        return out

    def state(self):
        now = time.monotonic()
        with self.lock:
            robots = {}
            for tag, entry in self.map.items():
                robot = entry["robot"]
                seen = self.seen.get(robot)
                robots[str(robot)] = {
                    "tag": tag,
                    "state": "sim" if robot in self.blocked else
                    "seen" if seen and now - seen[3] < STALE else "not seen",
                    "pose": list(seen[:3]) if seen else None}
            return {"map": {str(k): v for k, v in sorted(self.map.items())},
                    "robots": robots, "side_cm": float(self.side()),
                    "ros": self.node is not None}

    # ROS side

    def attach(self, node):
        from std_msgs.msg import String
        self.node = node
        node.create_subscription(String, TAGS_TOPIC, self.on_tags, 10)
        node.create_timer(1.0, self.check_topics)

    def odometry_topic(self, robot):
        return f"/model/robot_{robot}/odometry"

    def check_topics(self):
        """Leaves a robot's odometry to another node publishing it."""
        blocked = set()
        with self.lock:
            robots = {e["robot"] for e in self.map.values()}
        for robot in robots:
            ours = 1 if robot in self.publishers else 0
            if self.node.count_publishers(self.odometry_topic(robot)) > ours:
                blocked.add(robot)
        for robot in blocked - self.blocked:
            self.say(f"Robot {robot}'s odometry comes from another node, "
                     "such as the sim, so its tag is not used", "warn")
        with self.lock:
            self.blocked = blocked
        for robot in blocked & set(self.publishers):
            self.node.destroy_publisher(self.publishers.pop(robot))

    def on_tags(self, message):
        try:
            tags = json.loads(message.data)
        except ValueError:
            return
        if not isinstance(tags, list):
            return
        from nav_msgs.msg import Odometry
        poses = self.poses(tags)
        stamp = self.node.get_clock().now().to_msg()
        now = time.monotonic()
        for robot, (x, y, yaw) in poses.items():
            if robot in self.blocked:
                continue
            with self.lock:
                self.seen[robot] = (x, y, yaw, now)
            publisher = self.publishers.get(robot)
            if publisher is None:
                publisher = self.node.create_publisher(
                    Odometry, self.odometry_topic(robot), 10)
                self.publishers[robot] = publisher
            odometry = Odometry()
            odometry.header.stamp = stamp
            odometry.header.frame_id = "arena"
            odometry.child_frame_id = f"robot_{robot}"
            odometry.pose.pose.position.x = x
            odometry.pose.pose.position.y = y
            odometry.pose.pose.orientation.z = math.sin(yaw / 2)
            odometry.pose.pose.orientation.w = math.cos(yaw / 2)
            publisher.publish(odometry)

    def get(self, path):
        if path == "/api/tag_robots":
            return 200, self.state()
        return None

    def post(self, path, read_body):
        if path != "/api/tag_robots":
            return None
        try:
            asked = read_body()
            return 200, self.set(asked.get("tag"), asked.get("robot"),
                                 asked.get("turn", 0.0))
        except (ValueError, TypeError) as error:
            return 400, {"error": str(error)}


class NodeRunner:
    """One arena_perception program, started and stopped as from a
    terminal. Its ROS log comes to the Logs tab through /rosout; anything
    else it prints, such as a traceback, goes there from here."""

    def __init__(self, name, source, launcher="ros2", book=None):
        self.name = name
        self.source = source
        self.launcher = launcher
        self.book = book
        self.process = None
        self.lines = []
        self.lock = threading.Lock()

    def running(self):
        return self.process is not None and self.process.poll() is None

    def status(self):
        with self.lock:
            return {"running": self.running(),
                    "exit": None if self.process is None
                    else self.process.poll(),
                    "lines": self.lines[-8:]}

    def start(self, *args):
        if shutil.which(self.launcher) is None:
            raise ValueError(f"no {self.launcher} on the PATH: source ROS "
                             "and the workspace, then start the app again")
        with self.lock:
            if self.running():
                raise ValueError(f"{self.name} is already running")
            command = [self.launcher, "run", "arena_perception", self.name,
                       *map(str, args)]
            env = dict(os.environ, PYTHONUNBUFFERED="1",
                       RCUTILS_LOGGING_BUFFERED_STREAM="0")
            self.process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, start_new_session=True, env=env)
            self.lines = []
        self.say(f"Started {' '.join(command)}")
        threading.Thread(target=self.read, args=(self.process,),
                         daemon=True).start()
        return self.status()

    def say(self, text, level="info"):
        if self.book is not None:
            self.book.add(self.source, text, level)

    def read(self, process):
        from robogame_sim.app_log import guess_level
        for raw in process.stdout:
            line = raw.decode(errors="replace").rstrip("\r\n")
            with self.lock:
                self.lines = (self.lines + [line])[-50:]
            if not ROS_LINE.search(line):
                self.say(f"[{self.name}] {line}", guess_level(line))
        process.stdout.close()
        code = process.wait()
        self.say(f"{self.name} stopped (exit {code})",
                 "info" if code in (0, -signal.SIGINT) else "warn")

    def stop(self):
        with self.lock:
            process = self.process
        if process is None or process.poll() is not None:
            return self.status()
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(STOP_WAIT)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        except ProcessLookupError:
            pass
        return self.status()


def controllers_file():
    """controllers.yaml in the arena_perception package, the source copy
    with a symlink install, or None without the package."""
    try:
        from ament_index_python.packages import (
            PackageNotFoundError, get_package_share_directory)
    except ImportError:
        return None
    try:
        share = get_package_share_directory("arena_perception")
    except PackageNotFoundError:
        return None
    return os.path.realpath(os.path.join(share, "config",
                                         "controllers.yaml"))


HEADER = """\
# Which PS4 controller drives which robot.
#
# Robot N is the board flashed with `pio run -e robot_N`, which advertises
# itself as Robogame-N. Robot 0 is the instructor's. Each controller is
# listed by its Bluetooth address, so it always drives the same robot
# whatever order the controllers connect in. Keep the addresses in quotes.
#
# Update this file by pressing a button on each controller in turn:
#     ros2 run arena_perception assign_controllers 0 1 2 3 4 5 6
"""


def read_controllers(path):
    """{robot: address} from controllers.yaml, as arena_perception's
    controller_config reads it."""
    import yaml
    with open(path) as f:
        robots = (yaml.safe_load(f) or {}).get("robots") or {}
    return {int(k): str(v).strip().upper() for k, v in robots.items()}


def clean_controllers(robots):
    """{robot: address} checked as controller_config checks it."""
    out = {}
    for robot, address in (robots or {}).items():
        robot = int(robot)
        address = str(address).strip().upper()
        if robot < 0:
            raise ValueError("robot numbers start at 0")
        if not address:
            continue
        if not ADDRESS.match(address):
            raise ValueError(f"{address!r} for robot {robot} is not a "
                             "Bluetooth address like A0:5A:5E:EE:F8:FA")
        if address in out.values():
            raise ValueError(f"controller {address} is listed for more "
                             "than one robot")
        out[robot] = address
    return out


def write_controllers(path, robots):
    lines = [HEADER, "robots:"]
    lines += [f'  {robot}: "{robots[robot]}"' for robot in sorted(robots)]
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


class Perception:
    """arena_perception's nodes and controllers.yaml, for the console."""

    NODES = {"autonomy_node": "console", "gazebo_viz_node": "vision",
             "assign_controllers": "teleop"}

    def __init__(self, launcher="ros2", book=None, path=None):
        self.book = book
        self.path = path
        self.runners = {name: NodeRunner(name, source, launcher, book)
                        for name, source in self.NODES.items()}
        self.goal_publisher = None
        self.goal = None

    def attach(self, node):
        from geometry_msgs.msg import Point
        self.goal_publisher = node.create_publisher(
            Point, "/robot_1/goal", 10)
        self.Point = Point

    def file(self):
        return self.path or controllers_file()

    def controllers(self):
        path = self.file()
        if path is None:
            return {"path": None, "robots": {},
                    "error": "arena_perception is not built here"}
        try:
            robots = read_controllers(path)
            error = ""
        except FileNotFoundError:
            robots, error = {}, ""
        except (OSError, ValueError, AttributeError) as e:
            robots, error = {}, str(e)
        return {"path": path, "robots": {str(k): v for k, v in
                                         sorted(robots.items())},
                "error": error}

    def status(self):
        return {"nodes": {name: runner.status()
                          for name, runner in self.runners.items()},
                "goal": self.goal, "controllers": self.controllers()}

    def save_controllers(self, robots):
        path = self.file()
        if path is None:
            raise ValueError("arena_perception is not built here")
        robots = clean_controllers(robots)
        write_controllers(path, robots)
        if self.book is not None:
            self.book.add("teleop", f"Wrote {len(robots)} controller(s) to "
                          f"{path}")

    def send_goal(self, x, y):
        if self.goal_publisher is None:
            raise ValueError("ROS is not sourced")
        x, y = float(x), float(y)
        self.goal_publisher.publish(self.Point(x=x, y=y, z=0.0))
        self.goal = [x, y]

    def get(self, path):
        if path == "/api/perception":
            return 200, self.status()
        return None

    def post(self, path, read_body):
        if not path.startswith("/api/perception/"):
            return None
        part = path[len("/api/perception/"):]
        try:
            if part == "goal":
                asked = read_body()
                self.send_goal(asked.get("x"), asked.get("y"))
            elif part == "controllers":
                self.save_controllers(read_body().get("robots"))
            elif part == "assign":
                robots = [int(r) for r in read_body().get("robots") or []]
                if any(r < 0 for r in robots) or len(set(robots)) < len(
                        robots):
                    raise ValueError("give each robot number, from 0, once")
                self.runners["assign_controllers"].start(*robots)
            else:
                name, _, action = part.partition("/")
                runner = self.runners.get(name)
                if runner is None or action not in ("start", "stop"):
                    return 404, {"error": f"no {path}"}
                if action == "start":
                    runner.start()
                else:
                    runner.stop()
        except (ValueError, TypeError) as error:
            return 400, {"error": str(error)}
        return 200, self.status()

    def stop(self):
        for runner in self.runners.values():
            runner.stop()
