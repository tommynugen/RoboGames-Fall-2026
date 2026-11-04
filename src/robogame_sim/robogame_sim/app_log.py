"""The log lines behind the RoboGame Gamemaster's Logs tab.

One LogBook keeps the last lines from every part of the app, each tagged
with where it came from:

- editor: what the arena editor changed (saves, renames, deletes) and its
  errors, from the app's own request handler.
- console: the robot console node's ROS log (/rosout).
- teleop: the ROS log of the controller nodes (controller_robots,
  racing_teleop), wherever they run.
- vision: the Vision tab's camera, and the ROS log of arena_perception's
  apriltag_node and gazebo_viz_node.
- sim: the output of the Gazebo sim started from the editor's Launch
  buttons, read from its log file as it grows.
- bridge: the output of the BLE bridge started from the console's Bridge
  button (ble_bridge.py).
- app: the app itself (start, stop, no ROS).

Route, served by gamemaster.py:

- GET /api/logs?after=N: {"lines": [{"n", "time", "source", "level",
  "text"}, ...], "last": N}, the lines after line N, oldest first.
"""

import collections
import os
import threading
import time
from urllib.parse import parse_qs, urlsplit

SOURCES = ("editor", "console", "teleop", "vision", "sim", "bridge",
           "app")
KEEP = 5000  # lines kept, for all sources together
READ = 1 << 20  # bytes of sim log read at most per poll
# ROS node names whose log goes to the teleop tab.
TELEOP_NODES = ("controller_robots", "racing_teleop")
# And to the vision tab.
VISION_NODES = ("apriltag_node", "gazebo_viz_node")
# rcl_interfaces/msg/Log levels.
ROS_LEVELS = {10: "debug", 20: "info", 30: "warn", 40: "error",
              50: "error"}


class LogBook:
    def __init__(self, keep=KEEP):
        self.lines = collections.deque(maxlen=keep)
        self.count = 0
        self.errors = 0  # error lines so far, for the status bar
        self.last_error = None  # the newest of them
        self.lock = threading.Lock()
        self.follows = {}  # source: [path, offset, partial line]
        self.follow_lock = threading.Lock()

    def add(self, source, text, level="info", when=None):
        with self.lock:
            for part in str(text).rstrip("\n").split("\n"):
                self.count += 1
                line = {"n": self.count, "time": when or time.time(),
                        "source": source, "level": level, "text": part}
                self.lines.append(line)
                if level == "error":
                    self.errors += 1
                    self.last_error = line

    def follow(self, source, path, start=None):
        """Adds the lines written to the file at path to source, read on
        each poll, from byte start on (the end of the file as it is now
        by default). A file that shrinks (started again) is read from the
        top."""
        if start is None:
            try:
                start = os.path.getsize(path)
            except OSError:
                start = 0
        with self.follow_lock:
            self.follows[source] = [path, start, b""]

    def read_follows(self):
        with self.follow_lock:
            for source, state in self.follows.items():
                self.read_follow(source, state)

    def read_follow(self, source, state):
        path, offset, partial = state
        try:
            size = os.path.getsize(path)
        except OSError:
            return
        if size < offset:
            offset, partial = 0, b""
        if size == offset:
            return
        with open(path, "rb") as file:
            file.seek(offset)
            data = partial + file.read(READ)
            offset = file.tell()
        *whole, partial = data.split(b"\n")
        state[1:] = [offset, partial]
        for line in whole:
            text = line.decode(errors="replace").rstrip("\r")
            self.add(source, text, guess_level(text))

    def since(self, after=0):
        self.read_follows()
        with self.lock:
            lines = [line for line in self.lines if line["n"] > after]
            return {"lines": lines, "last": self.count}

    def summary(self):
        """The number of error lines so far and the newest of them (or
        None), for the Gamemaster's status bar."""
        self.read_follows()
        with self.lock:
            return {"errors": self.errors, "error": self.last_error}

    def get(self, path):
        parts = urlsplit(path)
        if parts.path != "/api/logs":
            return None
        try:
            after = int(parse_qs(parts.query).get("after", ["0"])[0])
        except ValueError:
            after = 0
        return 200, self.since(after)


def guess_level(text):
    """The level of a line of launch output, from ROS's [ERROR] and
    [WARN] tags."""
    upper = text.upper()
    if "[ERROR]" in upper or "[FATAL]" in upper or "TRACEBACK" in upper:
        return "error"
    if "[WARN]" in upper or "[WARNING]" in upper:
        return "warn"
    return "info"


def rosout_source(name, own):
    """The tab a /rosout line from node name goes to: own is the console
    node's name. None for other nodes, whose output the sim tab has."""
    name = name.lstrip("/").split(".")[0]
    if name == own:
        return "console"
    if name in TELEOP_NODES:
        return "teleop"
    if name in VISION_NODES:
        return "vision"
    return None


def watch_rosout(node, book):
    """Subscribes node to /rosout, adding the console's and the controller
    nodes' lines to book."""
    from rcl_interfaces.msg import Log

    own = node.get_name()

    def on_log(message):
        source = rosout_source(message.name, own)
        if source is not None:
            when = message.stamp.sec + message.stamp.nanosec * 1e-9
            # Sim time stamps are not wall time: use the time it came.
            if abs(when - time.time()) > 60:
                when = None
            book.add(source, f"[{message.name}] {message.msg}",
                     ROS_LEVELS.get(message.level, "info"), when)

    node.create_subscription(Log, "/rosout", on_log, 100)
